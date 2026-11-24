# Plan: s57_layer does not repaint cells uncovered when a rolling costmap window moves

## Issue

https://github.com/rolker/s57_tools/issues/57

## Context

When a rolling window moves, Nav2's `Costmap2D::updateOrigin` keeps the overlap and resets the uncovered cells to the default value. `S57Layer::updateBounds` reports only tiles flagged `needs_update`, so those cells stay free (local costmap) or unknown (global costmap) until a whole new tile enters. The issue review confirmed this on this branch. The design is the owner's choice at the plan checkpoint. This plan designs, implements on a scratch copy and measures three options, then recommends one.

**Evidence.** The harness is committed beside this plan, in `.agent/work-plans/issue-57/harness/`:
- `layer_variants.patch` adds the designs to a copy of `s57_layer.cpp` as a runtime switch.
- `harness.cpp` compiles that copy into its own executable, so no installed library can stand in for it. `ldd` shows no `libs57_layer`.
- `measure.sh` builds the harness and runs it.

Machine and build: a desktop with an Intel Core Ultra 7 265, a Release build, Rolling's source-built Nav2. Jazzy's `InflationLayer` comes from Nav2's `origin/jazzy`, copied and renamed into the harness. This underlay's `liblayers.so` links no OpenMP, so both inflation layers ran single-threaded; `OMP_NUM_THREADS` changed nothing.

Timing workload:
- BEN's footprint, inflation radius 150 m with scaling 0.05, and 100-cell tiles.
- A chart with 10 m cells, offset 0.37 m from the tile edges so rolker/s57_tools#63 cannot interfere, carrying 40 m islands on a 300 m grid and a coast to the north-east.

**These are desktop timings. No vessel computer was measured.**

## How Nav2 handles a thin strip

Established by reading Rolling's working tree and `origin/jazzy`. The harness confirms it.

1. **What Nav2 resets.** `LayeredCostmap::updateMap` takes the union of all layers' bounds as one rectangle. It resets the master grid inside it and calls every layer's `updateCosts` with that rectangle. This is the same on both distros; Jazzy also returns early when the bounds are empty.
2. **The inflation layer's padding.** `InflationLayer::updateBounds` is identical on both distros. It stores the bounds it receives (the union of the layers before it) and reports `min`/`max(this cycle, last cycle)` padded by the radius. So cycle *t* resets and repaints pad(B_t ∪ B_t−1, r).
3. **Who restores the reset cells.** This layer's `updateCosts` paints every tile cell inside the rectangle it is given, not only flagged tiles, so it restores everything Nav2 reset there. Other layers do the same for their own data:
   - `bathymetry_layer`: a tile blit within the bounds.
   - `ais_layer` and Nav2's static and obstacle layers: `updateWith*` within the bounds.
4. **Inflation around a new strip.** Rolling runs a distance transform over the padded region; Jazzy seeds its queue from the lethal cells in the padded region. Both write only inside the rectangle.
   - Every cell within r of a lethal cell newly painted in a strip lies inside pad(strip, r), so the inflation reaches r inward.
   - A cell outside the rectangle keeps its value, including inflation from lethal cells that have since left the window. A whole-window repaint drops that inflation; a strip repaint keeps it.
5. **The crux.** Because of the last-cycle merge, a rectangle reported in cycle *t* also costs its padded area in cycle *t+1*. An x-strip (full height) followed by a y-strip (full width) in the next cycle merges into the whole window.
   - Measured: alternating axes with no gap cycle costs 295 ms (Rolling) and 371 ms (Jazzy) per global cycle, the same as the whole window. The layer itself reports only 16 000 cells.
   - Today's code hits the same merge. On diagonal travel, a tile column and a tile row entering in the same or consecutive cycles make today's worst global cycle a whole window: 316 to 414 ms, in the table below.

## Designs

The scratch implementation of each is in `layer_variants.patch`; the harness's mode numbers are in brackets. All three replace the clamp at `s57_layer.cpp:344-350`: the layer clips its own rectangle to the window, then widens the shared bounds with `min`/`max`.

- **(a) [1] Whole window, every cycle, on a rolling costmap.** About 10 lines in `updateBounds`, no new state.
- **(c) [2] Whole window only on cycles whose origin moved.** The first cycle, `reset()` and `matchSize()` count as moves. About 15 lines and 3 members.
- **(d) [3] Uncovered strips, never their bounding box, with a gap cycle.** About 75 lines in `updateBounds`, plus a small `Interval` struct and 13 members: the last window (4 doubles), `have_window_`, `whole_pending_`, a low-side and a high-side pending interval per axis, `last_served_` and `last_axis_`.
  - **Recording.** Each cycle on a rolling costmap, the newly uncovered columns `[last_max_x, max_x]` / `[min_x, last_min_x]` are merged into that side's x interval, and likewise for y. Intervals are in world coordinates. Each cycle they are clipped to the current window, so a strip that scrolled out disappears and one still inside stays valid.
  - **Whole window.** A move with no overlap with the last window, the first cycle, `reset()` or `matchSize()` serves the whole window and clears every pending interval.
  - **Serving.** An x-strip is the hull of the x intervals times the current window's full height; a y-strip is the reverse.
  - **The rule.** Never serve y in the cycle after x, or x after y. When both axes are pending, the sequence is x, gap, y, gap. In a gap cycle this layer adds nothing, so the inflation layer re-runs only last cycle's strip. Travel along an axis serves that axis every cycle with no gap and no delay.
  - **Tiles.** A tile that did not overlap last cycle's window has all its in-window cells in this cycle's new strips, so its first-generation `needs_update` is not reported. This removes today's tile-column and tile-row reports and their whole-window spikes. Every other flagged tile (grid arrival, tide, chart set, `reset()`) is reported as today.
  - **Bounded delay.** An uncovered cell is painted at most 3 cycles after the cycle that uncovered it: 3 s on the global costmap and 0.6 s on the local one. Measured: the oldest wrong cell in any scenario had age 2.
  - **Bounded queue.** The state is O(1): two intervals per axis. Each axis is served at least once every 4 cycles, and serving clears that axis. A strip is at most 4 cycles of travel wide: 20 m on the global costmap at 5 m/s.
  - **Steady-state diagonal cost.** Every cycle costs one padded strip. The x cycle is followed by a gap that re-runs x's region through the merge, then the y cycle and its gap. Measured: 22 to 23 ms (Rolling) and 28 to 32 ms (Jazzy) on the global costmap.
  - **Whole-window cycles that remain.** These cost the whole window for 2 cycles, through the merge: the first cycle, `reset()`, a jump past the window, a tide or chart-set change (as today), and a reversal within 3 cycles that leaves strips on both sides of one axis (their hull spans the window).
- **(d without the gap) [4]: ruled out.** Measured above: it costs the whole window every cycle.
- **Other designs considered, and rejected:**
  - Painting the strip in `updateCosts` outside the reported bounds. This breaks Nav2's contract, and no inflation would form around the strip.
  - Rounding the strips to tiles. The 150 m padding dominates the cost, so this saves nothing and adds delay.

  (d) is the best design found.

### Correctness

Scratch harness, `harness correct`:
- **Window.** 20 x 20 cells at 1 m, 4-cell tiles, over a pseudo-random land chart with 1 m cells offset 0.25 m.
- **Oracle.** After every step each cell is compared with a freshly built window at the same origin. With inflation, lethal and unknown cells must match exactly, a cell's cost must never be below the oracle's, and cells farther than r from the window edges must match exactly.
- **Inflation set-ups.** Each scenario runs with no inflation layer, and with Rolling's and Jazzy's `InflationLayer` at r = 2 (under a tile) and r = 6 (at or above a tile), footprint set.

| Scenario | Today | (a) | (c) | (d) |
|---|---|---|---|---|
| east, north 1 m/step | fail (wrong cells up to 14 cycles old) | 0 wrong | 0 | 0 |
| diagonal (1,1), (0.6,0.35), (3,2) | fail, except (3,2) with inflation | 0 | 0 | only cells ≤ 2 cycles old |
| reversal past the start | fail | 0 | 0 | ≤ 2 cycles |
| jump 50 m (window 20 m) and back | fail | 0 | 0 | 0 with inflation, 4 cells for 1 cycle without |
| `reset()` mid-run (as `Costmap2DROS::resetLayers`) | fail | 0 | 0 | ≤ 2 cycles |
| fixed window (control) | 0 | 0 | 0 | 0 |
| tide +2 m above the threshold | fail (the move, not the tide) | 0 | 0 | ≤ 2 cycles |
| chart-set change | fail | 0 | 0 | ≤ 2 cycles |
| grid arrives late for tiles in the window | fail | 0 | 0 | ≤ 2 cycles |
| `track_unknown_space` east | fail | 0 | 0 | 0 |
| `allow_uncharted: false`, no chart, rolling | fail | 0 | 0 | ≤ 2 cycles |
| same, fixed window, no inflation | fail | fail | fail | fail (pass in every design with the one-line fix below) |

Jazzy's inflation gave the same results as Rolling's in every row.

### Measured cost per `updateMap` cycle, mean / worst (ms)

Global costmap: 5000 x 5000 cells at 1 m, 1 Hz, 100 cycles. E is due east; 45° moves both axes every cycle.

| Design | Inflation | E 2 m/s | E 5 m/s | 45° 2 m/s | 45° 5 m/s |
|---|---|---|---|---|---|
| today | Rolling | 11.2 / 24.4 | 12.4 / 25.6 | 17.1 / **316** | 29.4 / **321** |
| today | Jazzy | 11.7 / 36.0 | 13.3 / 36.7 | 18.7 / **414** | 31.8 / **381** |
| **(d)** | Rolling | 22.7 / 24.5 | 23.3 / 25.7 | 22.0 / 27.0 | 22.4 / 28.4 |
| **(d)** | Jazzy | 30.5 / 37.1 | 32.2 / 39.1 | 30.9 / 37.6 | 27.9 / 35.4 |
| (c) | Rolling | 311 / 316 | 317 / 321 | 311 / 318 | 314 / 320 |
| (c) | Jazzy | 374 / 445 | 418 / 447 | 390 / 436 | 371 / 407 |
| (a) | Rolling | 315 / 320 | 316 / 319 | 315 / 319 | 314 / 320 |
| (a) | Jazzy | 402 / 434 | 416 / 447 | 357 / 415 | 392 / 441 |
| (d) without the gap | Rolling / Jazzy | | | 295 / 314; 371 / 426 | |

Without an inflation layer, at 45° and 2 m/s: today 11.1 ms, (d) 10.9 ms, (a) 37.2 ms.

Local costmap: 700 x 700 cells at 0.5 m, 5 Hz, 300 cycles:

| Design | Inflation | E 2 m/s | E 5 m/s | 45° 2 m/s | 45° 5 m/s |
|---|---|---|---|---|---|
| today | Rolling / Jazzy | 0.10/3.1; 0.12/4.9 | 0.17/3.1; 0.26/5.8 | 0.11/4.1; 0.14/7.5 | 0.17/5.6; 0.22/7.5 |
| (d) | Rolling / Jazzy | 2.6/2.9; 4.9/6.1 | 2.6/2.9; 5.0/6.0 | 2.6/3.0; 5.7/7.2 | 2.7/3.2; 5.4/7.1 |
| (c) | Rolling / Jazzy | 3.5/3.7; 5.8/6.7 | 3.4/3.7; 5.7/6.5 | 3.5/3.9; 6.5/7.7 | 3.4/3.8; 6.2/7.6 |
| (a) | Rolling / Jazzy | 3.5/3.7; 5.9/6.7 | 3.5/4.9; 5.7/6.8 | 3.5/4.0; 6.5/7.6 | 3.5/3.9; 6.2/7.5 |

Notes on the numbers:
- **Chart density.** The Jazzy figures depend on how many lethal cells the chart has, since Jazzy's queue scales with them. The issue review's chart gave 493 ms for (a).
- **The global update budget.** BEN's global `chart_layer` has `update_timeout: 0.75`, a tile-generation budget that comes on top of these figures in cycles where tiles are generated.

## Recommendation

1. **Recommended: (d).** It is correct in every scenario, with each uncovered cell painted within 3 cycles (0 cycles on axis-aligned travel). It adds 11 to 20 ms per global cycle over today's typical cost. Its worst cycle (28 ms Rolling, 39 ms Jazzy) is lower than today's diagonal worst (316 to 414 ms). The cost is about 90 lines of new code.
2. **Rule out: (c).** Under way it costs the same as (a) (311 to 418 ms), because the origin moves almost every cycle, and it adds state for nothing. Also ruled out: the alternation without a gap (measured above).
3. **If the owner prefers the simplest code: (a).** It has no delay and about 10 lines. Its price is 0.3 s (Rolling) to 0.45 s (Jazzy) of every 1 s global cycle on this desktop, with the costmap locked against the planner meanwhile. It also forces the `current_` commit below.

## `current_`, the clamp, the uncharted case

### `current_`

Today `updateCosts` sets `current_` to "every tile in the rectangle is complete". When `updateCosts` is not called, the value sticks.

| Design and `current_` rule | Edge tile waits | Tile under robot waits | Fixed window, tile waits |
|---|---|---|---|
| today | 0 of 12 (larger late chart: 6-10 of 12) | 0 / 4 | **6 of 6, forever** |
| (a), (c) with today's rule | **12 of 12** | 12 | 6 of 6 |
| (d) with today's rule | 0-1 of 12 (larger late chart: as today) | 0 / 5 | 6 of 6 |
| any design, 3x3 core rule | 0 | 10 (correct) | 0 |
| any design, flagged-tiles rule | 0 | 0 (no longer a gate) | 6 of 6 |

The table shows cycles not current, from the scratch harness. The scenario is a chart grid that never arrives for one tile, with 12 moving cycles.

- **(a) and (c) force a change.** One tile anywhere in the 5 km window waiting for its grid holds the costmap not current. The planner waits on that (`costmap_update_timeout` 5 s on BEN), and so does the controller (1 s).
- **`bathymetry_layer`** gates instead on a core region around the robot (`ready_radius`, default 200 m). It reports the whole window every cycle.
- **(d) keeps today's behaviour**, measured, so the plan does not change `current_` with (d).
- **If (a) is chosen, `current_` gets its own commit first.** `current_` is computed in `updateBounds` every cycle, as "the robot's tile and its eight neighbours in the window are complete", and `updateCosts` no longer sets it. Its test fails today: on a fixed window with one tile outside the core waiting for its grid, `isCurrent()` is true after the first cycle (today: false forever). A second case, with the waiting tile under the robot, stays not current (the control).
- **For the host.** Today's fixed-window stickiness (6 of 6, forever) is a defect of its own. Under (d) it is not this issue's to fix: a follow-up issue for the core-region rule.

### The clamp

It is not a cell defect.
- **By reading.** It raises `*min` to the window edge and lowers `*max`, so it never moves a bound that lies inside the window. It trims only what lies outside, which Nav2 clips anyway.
- **Run.** With an earlier layer reporting 50 m beyond the window, 0 cells differed over 10 moving cycles between the clamp and min/max widening. Nav2 logs `Illegal bounds change ... offending layer is chart_layer` every cycle.
- **On BEN.** `chart_layer` is the first plugin in both costmaps, so it never happens.

The fix commit replaces the clamp, as every design needs. There is no separate commit and no cell test.

### The uncharted case

One line, `tiles_[id].needs_update = true;` in `generateTile`'s empty-chart early return, in its own commit.
- **Test.** Fixed window, `allow_uncharted: false`, no chart, no inflation layer: every cell `NO_INFORMATION` after the first cycle. It fails today and in every design without the line (the window stays free), and passes with it.
- **Comment.** `UnchartedBelowOriginMarkedUnknown`'s comment ("needs the inflation layer ... a separate defect") is reworded in the same commit. The inflation layer stays as the costmap node's configuration.

## Approach (for (d); the (a) variant noted)

1. **Commit A: the uncharted one-liner.** The fixed-window test and the comment, as above.
2. **Commit B (only if (a) is chosen): `current_` over the core 3x3 tiles**, with its two cases.
3. **Commit C: repaint the cells a rolling window uncovers (#57).**
   - **Code.** (d) in `updateBounds`, with its state in `s57_layer.h`. `reset()` and `matchSize()` set `whole_pending_`. The clamp is replaced by clipping this layer's own rectangle, then widening.
   - **Tests in `test_rolling_window.cpp`.**
     - A multi-cycle helper. It drives `updateMap` through robot positions at x.5 with whole-metre steps, so Jazzy's and Rolling's window formulas give the same origin (re-derived: Rolling asks robot−5, Jazzy robot−4.75 for 10 cells at 1 m; both truncate to the same whole-cell offset in either direction). It asserts the origin each step.
     - It tracks each cell's age and asserts every cell after every move: a cell that has been in the window for 3 or more cycles is lethal; a younger one is lethal or the default value. After 3 stationary cycles, every cell is lethal. With (a), every cell is lethal after every move.
     - The chart is all land, with its cells offset a quarter cell from the tile edges (#63).
     - **Cases:** east, north, diagonal (1,1), a reversal past the start, a jump past the window and back, `reset()` mid-run, a grid arriving late for tiles already in the window (this guards (d)'s tile rule), and a fixed window as control.
     - **Set-ups:** no inflation, then Nav2's `InflationLayer` with r = 2 m (under a 4-cell tile) and r = 6 m (at or above), footprint set.
     - **Without the fix (scratch):** every moving case fails with no inflation and with r = 2 and r = 6, with wrong cells up to 14 cycles old. The fixed window passes. Diagonal (3,2) with inflation passes today, so the cases use (1,1).
     - A coast-chart case compares cell values with a fresh window, not only lethal cells.
   - **Comments.** The "a separate defect" and "must stay one cycle" comments on the two 0.1 m cases are updated. **No multi-cycle variant of them is needed.** They test the offset rounding in `updateCosts`, which every repaint shares; a second cycle would only add (d)'s delay to the assertion.
   - **README.** One line: on a rolling costmap the layer reports the cells a move uncovers, within 3 cycles. (With (a): the whole window every cycle.)
4. **Verification.**
   - On Rolling, by hand on this machine: build `s57_layer` against the underlay and run its tests, checking with `ldd` that the test loads the built library.
   - On Jazzy: the repository's CI job builds the same tests against Jazzy's real `InflationLayer`.
   - Before the backport is deployed, the measurement below on a Jazzy machine.

## What changes for a vessel (BEN, both costmaps the same on rolling and jazzy)

- **Global costmap** (`track_unknown_space: true`, so uncovered cells read unknown today).
  - Today the leading edge 2400 to 2500 m out shows a band up to 99 m deep as unknown. It is repainted once per 100 m of travel: up to 50 s at 2 m/s.
  - With (d), charted land, hazards, depth costs and their 150 m inflation appear at most 3 s after entering the window: a band of at most 6 m (2 m/s) or 15 m (5 m/s). On a return over covered ground, the side being returned to is painted the same way.
  - **What the planner sees.** Smac (`allow_unknown: true`) today can plan through charted land inside that unknown band. `IsPathValid` runs at 2 Hz in `run_tasks.xml`, so land entering the window now invalidates a path crossing it, and the path is replanned within about 3 s of the land entering the window. The land is seen sooner; this is not more replanning.
  - **CPU.** (d) adds about 11-20 ms per 1 Hz cycle, and removes today's whole-window cycles on diagonal travel. (a) adds about 300-400 ms per cycle on this desktop.
- **Local costmap** (`track_unknown_space: false`, so uncovered cells read free today). The band at 125-175 m is painted within 0.6 s.
  - Nothing on BEN reads its costs, by reading: `CrabbingPathFollower` reads only the frame ID; the BT has no Spin, BackUp or DriveOnHeading; `Hover` has no collision check; the docking server has no docks.
  - The change is visible only in the published costmap. Its CPU cost is 2.5-7 ms per 5 Hz cycle in every design.
- **Not established:**
  - The cost on a vessel computer.
  - Rolling's Nav2 built with OpenMP (it would cut (a)'s Rolling cost, but does not affect the Jazzy deployment).
  - How often a tile waits for a grid on a vessel.
  - The tide-update rate.
  - `IsPathValid`'s handling of unknown cells. Not read.

**Measuring on a Jazzy machine** (for example deadpool), from this branch's checkout, in a shell with Jazzy and the workspace overlay (for `s57_msgs`) sourced:

```
ROS_DOMAIN_ID=<a free id above 150> JOBS=4 CORRECT=1 \
  .agent/work-plans/issue-57/harness/measure.sh "$PWD" > measure-jazzy.txt
```

- It prints the CPU model and one line per run.
- `native` there is Jazzy's own `InflationLayer`. Modes: 0 today, 3 (d), 2 (c), 1 (a), 4 (d) without the gap.
- About 25 minutes on this desktop.
- It builds in a temporary directory (`OUT`), which the agent removes afterwards.

## Files to Change

| File | Change |
|---|---|
| `s57_layer/src/s57_layer.cpp` | the uncharted line (A); (d) in `updateBounds`, `reset`, `matchSize`, the clamp replaced (C); with (a): `current_` (B) |
| `s57_layer/src/s57_layer.h` | (d)'s state (C) |
| `s57_layer/test/test_rolling_window.cpp` | the uncharted test and comment (A); the multi-cycle tests and comment updates (C); with (a): the `current_` cases (B) |
| `s57_layer/README.md` | one line on the bounds behaviour (C) |

## Prior Art

- **Taken:**
  - From `StaticLayer` and `bathymetry_layer`: widening to the window with `min`/`max`, which is (a).
  - From `bathymetry_layer`: the core-region readiness gate (`ready_radius`), for (a)'s `current_` commit, as a fixed 3x3 tiles with no parameter.
- **Considered and rejected:** Nav2's own layers that do not report the window (obstacle, `ais_layer`). They roll their own grids, which are empty in the strip, so they offer nothing to reuse.
- **Not found:** a Nav2 layer that reports uncovered strips. Design (d) is new.

## Serves

None written down for the project. In practice it serves the costmaps of the Jazzy vessels showing charted land where it is charted.

## Design Picture

No change to the design picture. The project has no design document. The README gains one line on the layer's update bounds.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Ask about what matters, and show how much | The design goes to the owner with measured costs on both distros' inflation layers; desktop only, with a command to measure on Jazzy hardware |
| Test what breaks | Every scenario was run on a scratch build. The planned tests fail without the fix, as listed above |
| Only what's needed | The clamp is replaced only because the fix rewrites those lines. The uncharted line is its own commit. `current_` changes only if (a) is chosen |
| Verify before claiming | Vessel CPU, OpenMP, `IsPathValid`'s handling of unknown cells: listed as not established |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0013 (progress entries) | Yes | This plan's `## Plan Authored` entry |
| Project rule: rolling first, then a hand backport by `git cherry-pick -x` | Yes | One fix per commit, no `.agent/` file in a code commit. The layer source is identical on `jazzy` |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `updateBounds` reporting | `test_rolling_window.cpp` comments, README | Yes |
| `current_` (with (a) only) | README note on readiness | Yes, in commit B |
| fixed-window `current_` stickiness | a follow-up issue | No: for the host |

## Open Questions

- **The owner chooses (d), (a) or (c)**, with the tables above. Recommendation: (d). Ruled out: (c).
- **The Jazzy measurement.** Should the backport wait for a run on deadpool or a vessel computer before it is deployed?
- **The fixed-window `current_` stickiness.** A follow-up issue for the host.
- **#59 and #63.** No design here constrains them; (d)'s tile rule does not depend on how `generateTile` fills a tile.

## Estimated Scope

A single pull request into `rolling` with 2 commits for (d) (A, C) or 3 for (a) (A, B, C). Then a hand backport to `jazzy` by `git cherry-pick -x` of the same commits.
