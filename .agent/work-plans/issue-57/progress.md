---
issue: 57
---

# Issue #57 — s57_layer does not repaint cells uncovered when a rolling costmap window moves

## Issue Review
**Status**: complete
**When**: 2026-11-24 13:44 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**Issue**: #57

### Scope Assessment

**Well-scoped?** Yes. The fix is one decision in `S57Layer::updateBounds` and a test file. #59 and #63 (both in `generateTile`) stay out. The fix has one design choice the issue leaves open. It is set out below for the owner, with measured costs.
**Right repo?** Yes. The defect is in `s57_layer`. Nav2's behaviour matches its own contract (each layer reports what it needs to write).
**Serves**: none written down for the project. In practice: the costmaps of the Jazzy vessels should show charted land where it is charted.
**Design picture?** No change. The project has no design document. The layer's README does not describe its update bounds.
**Dependencies**: #56 is merged. #59 and #63 are queued after this one and are independent of it in code (see Recommendations for the order).

#### The defect re-established on this branch (`afa7403`, after #56)

Scratch harness outside the tree. It compiles a copy of `s57_layer.cpp` into the test executable, so no installed library can stand in for it (checked with `ldd`), and links Rolling's source-built Nav2. Each step runs `LayeredCostmap::updateMap`. The window is 10 x 10 cells at 1 m with 4-cell tiles, over an all-land chart. Every robot position is at x.5, and at each step the harness also computes the origin Jazzy's `getSizeInMetersX` would give: it matched Rolling's origin at every step of every case below. Each count is the number of cells that are not LETHAL after that step, starting with the first cycle:

| Scenario | Counts per step |
|---|---|
| No inflation, east 1 m/step | 0 10 20 20 30 40 50 50 60 (north: the same; diagonal: 0 19 36 0 19 36 51 0 19) |
| Inflation 2 m (< tile), footprint set, east from 20.5 | 0 10 20 0 0 10 20 0 0 |
| Same, east from 23.5 | 0 10 20 30 10 10 20 30 10 (one column is never repainted) |
| Inflation 6 m (> tile), footprint set, east from 23.5 | 0 10 20 30 0 0 10 20 0 (band of at most one tile less one cell) |
| Inflation 6 m + footprint, 10 m east then back | east 0 10 20 0 0 10 20 0 0 10 20; west 20 20 20 30 40 50 60 70 80 90; one step further west, 100 |
| No inflation, 10 m east then back | ends at 100 of 100 |
| Jump 24 m east (past the window) and back | 0 10 0 10 **100** (with inflation 6 m: 0 10 0 0 **100**) |
| `allow_uncharted: false`, no chart, `track_unknown_space: false`, no inflation | 100 of 100 not unknown at every step, from the first cycle |
| Same with inflation 6 m + footprint | 0 10 20 30 40 50 60 70 80 |
| Reversal with **Jazzy's** `InflationLayer` (compiled from `origin/jazzy` into the harness) | identical to Rolling's |
| Fix options (a) and (c) below, every scenario above | 0 at every step |

#### Each statement in the issue, checked against the source

- `updateBounds` reports only tiles flagged `needs_update` (`s57_layer.cpp:321-339`): **confirmed**. One correction: a tile it already holds is also flagged when a chart grid arrives for a tile that is still incomplete (`generateTile`, `:414-465`), as well as by the tide threshold (`> tide_invalidate_threshold_`, 1 cm by default, `:194`), a change in the chart set (`:742-750`) and `reset()`.
- `updateCosts` writes every tile overlapping the bounds it is given, not only the flagged ones, then clears their flags (`:483-523`). That is why another layer's wider bounds hide the defect. **Confirmed**.
- `Costmap2D::updateOrigin` keeps the overlap and resets the rest to the default value. `LayeredCostmap::updateMap` resets only the union of the reported bounds. Neither widens the bounds when the origin moves, on either distro. **Confirmed**.
- `InflationLayer::updateBounds` widens the bounds by its radius around this cycle's and the last cycle's bounds. It reports the whole map only after a footprint change, and is identical on both distros. It does **not** force a whole-window update on a rolling costmap.
- "Stock rolling-capable layers report the whole window every cycle": **too broad**. `StaticLayer` does when rolling, identically on both distros. `bathymetry_layer` does so unconditionally (`:896-901`, the same on `jazzy`). `ObstacleLayer`/`VoxelLayer` and `ais_layer` do not, and need not: they roll their own grid, which has nothing in the uncovered strip, so the default value there is right for them. `s57_layer` holds static data that the uncovered cells should show.
- The case counts in "What the costmap shows": confirmed, with two notes. The permanent column with an inflation radius under a tile depends on where the tile edges fall: from 20.5 it is repainted, from 23.5 it is not. The reversal ends at 90 of 100 at the starting position and 100 of 100 one step further. Nav2 truncates the origin toward zero, so the first step back does not move the window.
- The BEN figures (bands up to 49.5 m deep at 125.5-175 m on the local costmap and 99 m at 2401-2500 m on the global one, each repainted once per tile width): consistent with the table above for 100-cell tiles at 0.5 m and 1 m. The config was read, not run. `ben_project11` has the same costmap config on `rolling` and `jazzy`: `chart_layer` + `inflation_layer` only, default `tile_size` 100, a 1 cm tide threshold, local track_unknown_space off (so free) and global on (so unknown), and the planner has `allow_unknown: true`.
- Not established, now partly answered by reading: BEN's controller, `CrabbingPathFollower`, reads the costmap only for its frame ID (`crabbing_path_follower.cpp:787`). That is also true of the 12 unimported follower commits on gitcloud's `jazzy`. Nothing in it checks collisions against the costmap. `AvoidanceController` does read costs, but BEN does not configure it. The local costmap is read by the behaviour server's Spin/BackUp/DriveOnHeading/AssistedTeleop collision checks, and none of the marine BT trees I searched invoke them. The tide-update frequency on a vessel is still not established.
- Jazzy differences that matter:
  - The layer source is identical on `origin/jazzy` and this branch, and gitcloud's `jazzy` of s57_tools has nothing more.
  - `getSizeInMetersX/Y` differs (Jazzy: `(size - 0.5) * res`). It changes only which robot positions give the same origin, so tests should keep the x.5 positions. With fix (a), the reported window still covers every cell on both distros.
  - Jazzy's early return in `updateMap` on empty bounds does not change the defect or a fix.
  - `InflationLayer::updateCosts` is a different algorithm: an OpenMP distance transform on Rolling, the older queue on Jazzy. A whole-window update costs 1.7x (global) to 3x (local) more on Jazzy (below).
- The related `allow_uncharted: false` case is **confirmed and wider than stated**. A tile created while there are no charts is never reported (the early return at `:363-371` does not set `needs_update`), so with no inflation layer the window is never marked unknown, not even on the first cycle.

#### Fix options (timings measured on this desktop, Release build, mean of 25-40 `updateMap` cycles; BEN's computer not measured)

Workload: BEN's footprint, inflation radius 150 m, scaling 0.05, 100-cell tiles, a coast and 40 m islands. The global costmap moves 2.75 m per cycle (BEN's `default_speed` at 1 Hz) and the local one 0.55 m per cycle (5 Hz), both on a 30° heading.

| Costmap | Now (typical / tile-column entry) | (a) whole window every cycle, Rolling inflation | (a), Jazzy inflation |
|---|---|---|---|
| local 700 x 700 @ 0.5 m, 5 Hz | 0.07 ms / 3.2 ms (Jazzy: 7.3 ms) | 3.5 ms | 10.3 ms |
| global 5000 x 5000 @ 1 m, 1 Hz | 8-11 ms (Nav2's own `updateOrigin` copies) / 23 ms (Jazzy: 34 ms) | 298 ms | 493 ms |

On the global costmap without an inflation layer, (a) costs 37 ms against 11 ms. Almost all of the cost of (a) is the inflation layer re-running over the whole window, not this layer's copy.

- **(a) Whole window on a rolling costmap every cycle** (as `StaticLayer` and `bathymetry_layer` do). Correct in every scenario above: forward, sideways, diagonal, reversal, a jump past the window, and the uncharted case on a rolling costmap. On a fixed window it changes nothing; there is no defect there for moved cells. The tide and chart-set paths still work through the tile flags, and a whole-window update was already what each of them triggered. Cost: negligible on the local costmap. On the global one it takes 0.3 s (Rolling) to 0.5 s (Jazzy) of each 1 s cycle on this machine, with the costmap locked against the planner meanwhile.
- **(b) Report only the uncovered strip.** Nav2's bounds are a single rectangle, and the rectangle around an L-shaped strip is the whole window. That happens whenever the window moves on both axes in one cycle: on the global costmap, every cycle on any heading more than about 21° off an axis, and on some cycles on nearly every other heading. So (b) costs about what (a) costs in practice. It saves only on axis-aligned runs: an estimated 20-30 ms there (the inflation radius pads the strip by 150 m each side; estimated from the tile-entry cost, not measured), at the price of tracking the previous window. A variant that defers the minor axis's strip by one cycle would always be cheap, but it leaves edge cells unpainted for a cycle and adds state. I do not recommend it without a measured need.
- **(c) Whole window only on cycles where the origin moved.** Under way it is the same as (a): the origin moves almost every cycle on both of BEN's costmaps. It saves only when the window stays put, and the first cycle must count as a move. It is correct in every scenario above. It costs three members.

Interactions any option must handle:
1. `updateCosts` sets `current_` from every tile it visits (`:494, 525`). With the whole window visited, one tile at the edge still waiting for its grid (or one the `update_timeout` budget skipped) holds `isCurrent()` false. The planner and controller wait on that with `costmap_update_timeout`. `bathymetry_layer` gates readiness on a core region around the robot for exactly this reason (its comment above `:896`).
2. The existing clamp at `:344-350` assigns rather than widens, so it can shrink bounds an earlier layer reported. Report the window with `min`/`max`, as `bathymetry_layer` does.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Ask about what matters, and show how much | Action needed | The owner chooses between (a), (c) and a strip variant. Under way, (a) and (c) take 0.3-0.5 s of each 1 s global-costmap cycle on this desktop, more on a slower vessel computer. It is easy to undo (one function), but it ships to the Jazzy vessels. |
| Test what breaks | Watch | The issue names the right test. Make it assert every cell after every step, and include a jump, a reversal and the uncharted case (see Recommendations). |
| A change includes its consequences | Action needed | Comments in `test_rolling_window.cpp` call this "a separate defect" (`UnchartedBelowOriginMarkedUnknown`, which needs the inflation layer only because of it, and the "must stay one cycle" notes on the two non-binary tests). They become false with the fix and should change in the same PR. |
| Verify before claiming | Watch | A few statements in the issue are corrected above (stock layers, the phase dependence, 90 vs 100). The body stays as written. |
| Small steps | OK | #59 and #63 stay separate. This fix does not touch `generateTile`. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0013 — progress.md entries | Yes | This entry and the later phases' entries. |
| Project rule: rolling first, then a hand backport by `git cherry-pick -x` | Yes | The source is identical on `jazzy`, so the cherry-pick should apply cleanly. The Jazzy CI job builds and tests `s57_layer` against Jazzy's real `InflationLayer`, which the tests should therefore include. |

### Consequences

- The `test_rolling_window.cpp` comments named above.
- The `s57_layer` README: one line saying the layer repaints the whole window each cycle on a rolling costmap (if (a) or (c) is chosen), since that sets the cost of a large rolling costmap.

### Recommendations

- Recommend (a), applied only when `layered_costmap_->isRolling()`, widening the bounds with `min`/`max`. It matches Nav2's convention and is the simplest to show correct. Take (c) instead only if the owner wants the saving when the window stays put, which is small under way. Either way, the owner accepts the measured global-costmap cost, or has it measured on a vessel computer (or deadpool) before the Jazzy backport is deployed.
- Decide in the plan what `current_` means once the whole window is visited. Either keep it computed over the tiles that needed an update, as today, or test that an edge tile waiting for its grid does not hold the costmap not-current.
- Tests: extend `test_rolling_window.cpp`'s fixture to several `updateMap` cycles that assert every cell after each step. Cover east, north, diagonal, a 10-cell reversal past the start, and a jump past the window and back. Run each without an inflation layer and with Nav2's `InflationLayer` (footprint set), at radii under and over a tile. Add a rolling `allow_uncharted: false` case with no chart and no inflation layer. Keep robot positions at x.5 so Jazzy and Rolling give the same origin, and assert the origin at each step. A small tile (4 cells) at 1 m is enough. BEN's resolutions add nothing to the window arithmetic beyond the existing 0.1 m tests. Each fails today (counts above).
- Give those tests a chart whose cell edges do not lie on tile edges (for example, offset by a quarter cell), so they do not run into #63. In this harness, #63 is no longer occasional: with 100-cell tiles and 10 m chart cells aligned to the tile grid, the global-size run aborted in glibc's allocator 2 runs of 2, and ran cleanly 2 of 2 with the chart offset by 0.37 m.
- Order: 57, then 59, then 63 works. This fix is in `updateBounds`, theirs are in `generateTile`, and none needs another for honest tests once #57's tests avoid tile-aligned charts. Fixing #57 first lets #59's tests move a window over several cycles. Whether #63 should move ahead, now that it reproduces deterministically as heap corruption, is the owner's call.
- Scope: the uncharted case on a **fixed** window is not covered by (a) or (c). By reading, not run: a tile created while the chart set is empty is reported only if the chart set later changes (`:363-371`, `:742-750`). Its only other chance to be painted is the inflation layer's full repaint after a footprint change. Leave it out of this fix and open it as its own issue, or fold it in as the one-line `needs_update = true` in that early return with its own test if the owner prefers.

### Actions
- [ ] The owner chooses between (a), (c) and a strip variant. Under way, (a) and (c) take 0.3-0.5 s of each 1 s global-costmap cycle on this desktop, more on a slower vessel computer. It is easy to undo (one function), but it ships to the Jazzy vessels.
- [ ] Comments in `test_rolling_window.cpp` call this "a separate defect" (`UnchartedBelowOriginMarkedUnknown`, which needs the inflation layer only because of it, and the "must stay one cycle" notes on the two non-binary tests). They become false with the fix and should change in the same PR.
- [ ] Recommend (a), applied only when `layered_costmap_->isRolling()`, widening the bounds with `min`/`max`. It matches Nav2's convention and is the simplest to show correct. Take (c) instead only if the owner wants the saving when the window stays put, which is small under way. Either way, the owner accepts the measured global-costmap cost, or has it measured on a vessel computer (or deadpool) before the Jazzy backport is deployed.
- [ ] Decide in the plan what `current_` means once the whole window is visited. Either keep it computed over the tiles that needed an update, as today, or test that an edge tile waiting for its grid does not hold the costmap not-current.
- [ ] Tests: extend `test_rolling_window.cpp`'s fixture to several `updateMap` cycles that assert every cell after each step. Cover east, north, diagonal, a 10-cell reversal past the start, and a jump past the window and back. Run each without an inflation layer and with Nav2's `InflationLayer` (footprint set), at radii under and over a tile. Add a rolling `allow_uncharted: false` case with no chart and no inflation layer. Keep robot positions at x.5 so Jazzy and Rolling give the same origin, and assert the origin at each step. A small tile (4 cells) at 1 m is enough. BEN's resolutions add nothing to the window arithmetic beyond the existing 0.1 m tests. Each fails today (counts above).
- [ ] Give those tests a chart whose cell edges do not lie on tile edges (for example, offset by a quarter cell), so they do not run into #63. In this harness, #63 is no longer occasional: with 100-cell tiles and 10 m chart cells aligned to the tile grid, the global-size run aborted in glibc's allocator 2 runs of 2, and ran cleanly 2 of 2 with the chart offset by 0.37 m.
- [ ] Order: 57, then 59, then 63 works. This fix is in `updateBounds`, theirs are in `generateTile`, and none needs another for honest tests once #57's tests avoid tile-aligned charts. Fixing #57 first lets #59's tests move a window over several cycles. Whether #63 should move ahead, now that it reproduces deterministically as heap corruption, is the owner's call.
- [ ] Scope: the uncharted case on a **fixed** window is not covered by (a) or (c). By reading, not run: a tile created while the chart set is empty is reported only if the chart set later changes (`:363-371`, `:742-750`). Its only other chance to be painted is the inflation layer's full repaint after a footprint change. Leave it out of this fix and open it as its own issue, or fold it in as the one-line `needs_update = true` in that early return with its own test if the owner prefers.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 13:45 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the issue review confirmed the defect and the scope; the design choice it raises is not settled here: the plan measures the options and the choice goes to the owner at the plan checkpoint, with numbers)
**After**: issue-actions
**Decision**: proceed

Read: the Issue Review, status complete. The defect is re-established on this branch after rolker/s57_tools#56. Scope stands: cells a rolling window's move uncovers are painted from the chart. For the planner: (1) **the design is open and the owner chooses it at the plan checkpoint.** The review measured, on a desktop and not on a vessel's computer, that reporting the whole window every cycle makes the global costmap's update take about 0.3 s (Rolling's inflation) to 0.5 s (Jazzy's) of each 1 s cycle, against 8 to 34 ms now, nearly all of it the inflation layer redoing the whole window. The plan designs and measures at least these: (a) whole window every cycle when rolling; (c) whole window only on cycles where the origin moved; and (d) the uncovered region reported as thin rectangles and never as their bounding box: when a move uncovers a strip on both axes, report one rectangle this cycle and carry the other to the next (or an equivalent that keeps each cycle's bounds to a strip plus what the inflation layer adds), with correctness on a reversal, a jump larger than the window, `resetMap`, a non-rolling costmap, and the tide and chart-set paths; with the bounded delay stated (cells uncovered may wait one cycle). Each with its measured cost on BEN's two costmap sizes under both distros' inflation layers, and the planner's recommendation. (2) `current_`: a wider report must not let one edge tile that is still waiting for its grid hold the layer not current and stall the planner; say how each design behaves and what `bathymetry_layer` does. (3) The existing clamp at `:344-350` assigns; a fix widens. (4) The uncharted case on a fixed window (a tile created while there are no charts is never reported, with `allow_uncharted: false`) is in this issue's "Related" text: cover it if it is one line with a test that fails without it, as its own commit; otherwise list it. (5) Tests use a chart whose cell edges are off the tile edges, so rolker/s57_tools#63 does not interfere, and robot positions at x.5 so Jazzy's and Rolling's window formulas agree; the comments in `test_rolling_window.cpp` that call this "a separate defect" are updated in the fix's commit; a README line if the layer's bounds behaviour changes. (6) Severity as now read: BEN's controller (`CrabbingPathFollower`) reads the costmap only for its frame, so on BEN the costs that matter are the global costmap's, read by the planner. Queue: the host moves rolker/s57_tools#63 ahead of #59, since the review reproduced it as heap corruption.

## Plan Authored
**Status**: complete
**When**: 2026-11-24 14:21 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Plan**: `.agent/work-plans/issue-57/plan.md` at `021ebe4`

Three designs were implemented on a scratch copy of the layer and run through the full scenario set: (a) whole window every cycle, (c) whole window when the origin moved, (d) uncovered strips with a gap cycle, so Nav2's inflation merge never joins an x-strip and a y-strip. All are correct; (d) paints each uncovered cell within 3 cycles. On the global costmap on this desktop, (d) costs 22-32 ms per cycle against 311-418 ms for (a)/(c), and its worst cycle is below today's diagonal worst (316-414 ms). Recommends (d), rules out (c); the owner chooses. Plus a one-line commit for the uncharted case, and the clamp is replaced in the fix (not a cell defect). `current_` changes only if (a) is chosen. The harness and a Jazzy measuring script are committed beside the plan.

## Plan Review
**Status**: complete
**When**: 2026-11-24 14:36 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: needs-work

**Issue**: #57 — s57_layer does not repaint cells uncovered when a rolling costmap window moves
**Plan**: `.agent/work-plans/issue-57/plan.md` at `021ebe4`
**Branch**: `feature/issue-57`
**Serves**: "None written down for the project. In practice it serves the costmaps of the Jazzy vessels showing charted land where it is charted." (checked: the project has no goals or design document; no stated rule broken)

The planner's cost table and its correctness table can be trusted: I rebuilt the committed harness and reproduced both. (d) held under 1480 adversarial runs. The needs-work is about the planned tests. Under the planned assertion several of the cases pass on today's code, and the plan says they fail.

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | One PR: commit A (uncharted one-liner), C (the design), B only with (a). #59 and #63 stay out. |
| Issue alignment | Good | Covers every item of the checkpoint brief: (a), (c) and (d) measured on both inflation layers, `current_`, the clamp, the uncharted case, chart offset off the tile edges, x.5 positions, the comments and the README. |
| File targeting | Good | `s57_layer.cpp`, `s57_layer.h`, `test_rolling_window.cpp`, `README.md`. Nothing else consumes the bounds. |
| Consequences | Good | Comments, README, and the `current_` follow-up for the host. One gap: the harness is not reproducible after commit C (finding 3). |
| Principle alignment | Needs work | "Test what breaks": the planned assertion lets today's code pass several of the named cases (finding 1). The other principles are met. |
| ADR compliance | Good | ADR-0013 entries. Rolling first with a `cherry-pick -x` backport, one fix per commit, no `.agent/` file in a code commit. |
| ROS conventions | Good | Bounds are widened with `min`/`max` per Nav2's layer contract. No topic, QoS or parameter change. |

### Independent checks (run on this desktop, Intel Core Ultra 7 265, Release, Rolling's source-built Nav2; scratch only)

**1. The numbers.** I built the committed harness (`measure.sh`'s build steps) and ran a subset: today, (d) and (a), global costmap, 100 cycles, under both inflation layers, plus the local costmap at 45°. My figures are on the right of each pair (mean / worst ms per `updateMap`):

| Global 5000², 1 Hz | Planner | Mine |
|---|---|---|
| today R, E 2 m/s | 11.2 / 24.4 | 13.0 / 27.9 |
| today R, 45° 2 / 5 m/s | 17.1 / 316; 29.4 / 321 | 18.8 / 324; 29.3 / 317 |
| today J, E 2 m/s | 11.7 / 36.0 | 11.3 / 35.5 |
| today J, 45° 2 / 5 m/s | 18.7 / 414; 31.8 / 381 | 19.5 / 438; 35.6 / 436 |
| (d) R, E 2; 45° 2; 45° 5 | 22.7/24.5; 22.0/27.0; 22.4/28.4 | 25.7/27.8; 23.0/28.6; 21.6/29.7 |
| (d) J, E 2; 45° 2; 45° 5 | 30.5/37.1; 30.9/37.6; 27.9/35.4 | 30.7/37.5; 31.1/37.7; 31.1/40.3 |
| (a) R, E 2; 45° 2 | 315/320; 315/319 | 319/322; 315/324 |
| (a) J, E 2; 45° 2 | 402/434; 357/415 | 379/469; 401/431 |
| local 700², 45° 2 m/s, R: today / (d) / (a) | 0.11/4.1; 2.6/3.0; 3.5/4.0 | 0.11/4.2; 2.63/3.03; 3.57/3.96 |

These agree within about 10%. Some runs overlapped my correctness runs on this 20-core machine.
- **What the harness drives.** Each cycle runs Nav2's real `LayeredCostmap::updateMap` and `Costmap2D::updateOrigin` (Rolling's), and the layer is a patched copy compiled in. `ldd` shows the underlay's `liblayers.so` and no `libs57_layer`. Jazzy's `InflationLayer` is copied from `origin/jazzy` at build time. Jazzy's `LayeredCostmap` and `Costmap2D` are not exercised, and neither are `Costmap2DROS`, publishing or lock contention.
- **Single-threaded inflation is right for Jazzy.** Jazzy's `nav2_costmap_2d/CMakeLists.txt` has no OpenMP at all, and its inflation is the serial queue. Rolling's has `option(ENABLE_OPENMP ... OFF)`, so it is off by default too.
- **The chart.** 10 m cells offset 0.37 m, with 40 m islands on a 300 m lattice and a NE coast. Jazzy's cost scales with lethal-cell density, as the plan notes.
- **BEN's merged bounds are not already whole-window.** Both of BEN's Nav2 costmaps list only `["chart_layer", "inflation_layer"]`. That holds in `ben_project11/config/nav2_params.yaml` on `rolling`, on `origin/jazzy` (identical) and on gitcloud's `field/ben_project11` `jazzy` (same plugin lists). There is no `bathymetry_layer`, AIS or obstacle layer. The `costmap*.yaml` files with voxel and radar layers are ROS 1 `costmap_2d` launch leftovers that the Nav2 bringup does not use. No other platform config loads `S57Layer`.
  - `InflationLayer::updateBounds` (identical on both distros) reports the whole map only when `need_reinflation_` is set: a footprint change, a parameter change, `reset()` or `matchSize()`. Otherwise it reports pad(this ∪ last cycle, r).
  - So today's typical global cycle really is 11-13 ms. Whole-window cycles happen only on diagonal tile-column plus tile-row entries: about 2 in 100 cycles at 45° and 2 m/s, about 6 in 100 at 5 m/s (from the mean and worst above). They also come with tide steps over 1 cm and chart-set changes.
  - (a)'s ~0.3-0.4 s is therefore a real addition to almost every cycle. The recommendation is not reversed.
- **Context for the owner.** Jazzy's `SmacPlannerHybrid::createPlan` holds the costmap mutex while it plans (`smac_planner_hybrid.cpp:355`). So does the publisher, and BEN's global costmap publishes the full 25 M-cell map at 1 Hz (`always_send_full_costmap: True`). (a)'s extra 0.3-0.45 s per second is lock time the planner waits behind.

**2. (d) attacked.** I extended the harness with an adversarial mode:
- **Checks.** Each cell is compared after every cycle with a freshly built window. I track how long each world cell stays wrong (streak) and the age of any lethal or unknown mismatch.
- **Scenarios (59).**
  - Random walks of 150 cycles with steps up to ±3 m and ±1.2 m on both axes.
  - Constant non-integer velocities (0.7, 1.3), (2.3, −1.7) and (0.37, 0.11) for up to 200 cycles.
  - Zig-zags on x and on x+y, starting at each of the 4 phases of the x/gap/y/gap cycle.
  - Reversals at each phase, and stops at 5 phases.
  - One-tile steps.
  - Jumps of more than a window (diagonal) and of half a window, then back.
  - A path crossing the origin from (−25.5, −18.5), and a random walk around 0.
  - Window sizes of 21 and 22 cells with 4-cell tiles.
  - The tide step, the chart-set change and `reset()` at each of cycles 3-7.
  - A late grid for a tile at the edge, under the robot, and ahead of the window, each arriving at several phases.
  - `track_unknown_space` on a fixed window and on a random walk, and `allow_uncharted: false` on a random walk.
- **Setups.** Each scenario ran with no inflation, with Rolling's inflation at r 2 and 6, and with Jazzy's at r 2 and 9. That was repeated with no other layer, with a layer reporting a 3 m box around the robot before or after `chart_layer`, and with a layer reporting a thin rectangle at the leading edge after it: 1180 runs. 300 more ran over an all-land chart.
- **Result.** No cell was wrong for more than 3 consecutive cycles in any run, and no lethal or unknown mismatch was older than age 2. That is "painted within 3 cycles", as stated. The pending intervals stayed bounded: at most 12 m, which is 3 cycles of 4 m steps. Today's code fails 457 of the 1180.
- **The first-generation tile rule holds.** By reading, every cell outside last cycle's window lies in an x- or y-interval that is served on the current window's full extent. A tile that the `update_timeout` budget leaves unvisited is never cleared, and it overlaps the window by the next cycle. No path left a new tile's cells in neither a strip nor a tile report.
- **Jazzy's half-cell-short window.** Jazzy's `getSizeInMetersX = (size − 0.5)·res`: by reading, the strip's half-cell pad and Nav2's `xn + 1` still cover the edge column. The overlap test errs on the safe side. Not run on Jazzy's `Costmap2D`.
- **One anomaly, not (d)'s.** With Jazzy's inflation (r 4 and r 9; Rolling's never), one cell at the trailing corner held cost 63 against the fresh window's 69 for 2 cycles, with no young cell nearby. Jazzy's queue inflation does not give the same costs over a sub-rectangle as over the whole window. This matters for the planned value-comparison test (finding 2).

**3-5.** See findings 4-6 and the Summary.

### Findings

1. **[Principle alignment: Test what breaks] Major.** Under the planned assertion ("a cell in the window for 3 or more cycles is lethal"), several planned cases pass on today's code. The plan says every moving case fails without the fix, with and without inflation.
   - **What I ran.** An all-land chart with 4-cell tiles, robots at x.5, today's code, in windows of 20 and 10 cells.
   - **Diagonal (1,1)** passes in every setup. Today repaints on each tile entry, every 4 cycles at 1 cell per cycle, so no cell is wrong for more than 3 cycles.
   - **`reset()` mid-run and jump+back** pass in every setup: the path ends within 3 cycles of the jump back.
   - **East and north with inflation r 2 or r 6** pass: the inflation layer's padding of the edge-tile report covers 1 m of travel per cycle. The exception is J r2 from 23.5 in the 10-cell window.
   - **What fails today.** The no-inflation east and north cases (wrong cells up to 14 cycles old) and the reversal in every setup.
   - **Hiding inside the allowance.** A blanket 3-cycle allowance would also hide a regression that serves axis-aligned strips 1-2 cycles late, though (d) has 0 delay there.
   - **Resolution.**
     - Assert every cell lethal after every move on axis-aligned travel.
     - On diagonals, use a slow step such as (0.6, 0.35) so that today's tile-entry period exceeds 3 cycles. Run several cycles after the jump back.
     - Show each case failing on `rolling` under the exact assertion before it is called a regression test. Label the inflation variants as guards of the merge and gap logic if they cannot fail today.
     - Consider one cheap assertion that guards (d)'s cost property, which nothing else will catch if it is lost: on steady diagonal travel, `LayeredCostmap::getBounds` is never the whole window.
2. **[Principle alignment: Test what breaks] Minor.** The planned "coast-chart case compares cell values with a fresh window" will be flaky or fail on the Jazzy CI job if it runs under Jazzy's `InflationLayer`: a partial re-inflation differs from a full one by a few cost units (above). Compare values exactly only with no inflation layer. With inflation, compare lethal and unknown cells.
3. **[Consequences] Minor.** `layer_variants.patch` applies only to `s57_layer.cpp` as it stands at `021ebe4` (identical on `jazzy` today). After commit C, the plan's deadpool command ("from this branch's checkout") fails at `patch`.
   - **Resolution.** Pin the source in the command, for example a `git worktree` at `021ebe4` or `jazzy` before the backport, and say so in `measure.sh`'s usage.
   - **What is committed.** `harness.cpp` (21 KB), `layer_variants.patch` (11 KB, this repository's own code, BSD), `measure.sh` and `CMakeLists.txt`. No Nav2 file is committed: `measure.sh` extracts Jazzy's Apache-2.0 `inflation_layer.{hpp,cpp}` from a local navigation2 checkout at measure time into `$OUT`. Carrying it in the pull request is fine.
4. **[Scope: code size of (d)] Suggestion.** The state can be smaller with no change in behaviour.
   - Serving already takes the hull of the low and high intervals, so one interval per axis suffices. The only loss is a wider strip after a reversal within 3 cycles, which already goes whole-window.
   - `have_window_` can be folded into `whole_pending_`.
   - That gives about 9 members instead of 13. Worth doing if (d) is chosen, since each member is review surface on hazard-marking code.
5. **[Issue alignment: `current_`] Note, confirmed.** I reproduced the plan's `current_` table exactly.
   - **The stall is real on both distros.** Jazzy's `PlannerServer::waitForCostmap` and its controller loop, and Rolling's `waitUntilCurrent`, throw `PlannerTimedOut`/`ControllerTimedOut` after `costmap_update_timeout` (BEN: 5 s planner, 1 s controller). So (a) or (c) without commit B would fail every plan while any tile in the 5 km window waits for its grid.
   - **(d) keeps today's semantics.** Those semantics are weak too: today does not hold not-current while the tile under the robot waits (0 of 12 with no inflation).
   - **The fixed-window stickiness is real and pre-existing.** It would stall the planner permanently on a fixed-window costmap. BEN's costmaps are both rolling, so no deployed vessel is affected. Raising a follow-up issue is right.
6. **[Approach: commit A and the clamp] Good.** The empty-chart early return is the only uncharted branch that does not set `needs_update`; the other (`grids.empty() && all_charts_available`) already does. With `complete = true` it fires once per tile, so it causes no repeated reports. The fixed-window test fails today. The clamp check reproduces: 0 cells differ, and `chart_layer` is first on both of BEN's costmaps.

### Summary

The design work is sound and its numbers are reproducible. (d) is correct with a bounded 3-cycle delay under everything I threw at it. It is the only option that does not add 0.3-0.45 s of locked costmap time to almost every global cycle on BEN, whose merged bounds are not already whole-window.

The plan needs its test section corrected before implementation: several named cases do not fail on today's code under the planned assertion (finding 1). The design choice remains the owner's.

**Options for the owner (global costmap, this desktop):**

| Option | Correctness | Delay | Mean / worst ms (Rolling; Jazzy inflation) | Code |
|---|---|---|---|---|
| (d) strips + gap | all scenarios | ≤ 3 cycles (3 s global, 0.6 s local; 0 axis-aligned) | 22-26 / 28-30; 31 / 38-40 | ~75 lines, 9-13 members, 4-phase rule |
| (a) whole window + commit B | all scenarios | 0 | 315-319 / 322-324; 379-401 / 431-469 | ~10 lines + `current_` core rule (~15) |
| (f) whole window every N cycles + B (measured, N=10; N=5) | all, within N cycles | ≤ N cycles (10 s) | 78 / 318; 102 / 466 (N=5: 137 / 321; 174 / 480) | ~8 lines, 1 member, + B |

Notes on the other alternatives:
- (f) costs 2 whole-window cycles per report because of the inflation layer's last-bounds merge. It keeps today's spikes and needs commit B, so (d) dominates it except on code size.
- (g), the strips' bounding box, is (a)'s cost on any non-axis heading.
- (e), both strips when small, otherwise whole, reduces to (g) or (a) under way on a diagonal.

**My choice: (d).**
- The delay is irrelevant where it applies (cells 2.4-2.5 km away on the global costmap; nothing on BEN reads the local one's costs).
- Its mean cost is at or below today's on diagonal travel at 5 m/s, and its worst cycle is a tenth of today's.
- (a)'s cost is unmeasured on the vessel computer, and it lands under a mutex the planner needs.
- Take (d) with finding 4's smaller state and finding 1's tests.
- Choose (a) + B instead only if the owner rates the review burden of (d)'s state machine above that risk. In that case, measure (a) on deadpool or a vessel computer before the Jazzy backport deploys.

**What changes for a vessel**, as the plan states it: the global costmap's leading-edge band and the far side after a reversal show charted land within 3 s instead of up to 50 s or never.

**Not established:**
- Vessel CPU.
- How often the tide steps over 1 cm. Each step regenerates every tile and reports the whole window today.
- Jazzy's `LayeredCostmap` and `Costmap2D` under (d), which only the Jazzy CI tests will exercise.
- `IsPathValid` on unknown cells.

### Recommended Actions

- [ ] Rewrite the planned test assertions so that each named case fails on today's code. Assert every cell lethal on axis-aligned travel, use a slow diagonal and longer post-jump runs, and show each failing on `rolling`. Optionally assert that `getBounds` is never the whole window on steady diagonal travel.
- [ ] Restrict exact cost-value comparison to runs without an inflation layer (Jazzy's queue inflation differs between partial and full updates).
- [ ] Pin `measure.sh`'s source to a pre-fix commit in the plan's deadpool command and in its usage text.
- [ ] If (d) is chosen: one pending interval per axis, and `have_window_` folded into `whole_pending_`.
- [ ] Owner: choose (d), (a)+B or (f)+B from the table above. Decide whether the Jazzy backport waits for a deadpool or vessel measurement.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 14:37 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: owner
**After**: plan
**Decision**: proceed

The owner, 2026-11-24, asked which design fixes the defect, with the planner's and the plan reviewer's measured table in the dialog (today; strips; whole window every cycle; whole window every 10th cycle; measure on his Jazzy machine first): "Repaint only the uncovered strips (Recommended)". So design (d): the uncovered region is reported as thin strips with a gap cycle between the x and the y strip, a cell uncovered at the window's edge is painted within 3 cycles, and how the layer computes `current_` is left as it is. Commit B (the `current_` core rule) is not made.

The Plan Review's verdict is needs-work, on the test section only; the host does not send the plan back for that, because each resolution is concrete and is checked where it can be checked, in the implementation and its review: every test is shown failing on `rolling` under its exact assertion before it is called a regression test. Applied as follows. (1) Tests: on axis-aligned travel every cell is asserted after every move, with no allowance, since the design has no delay there; diagonal cases use a slow step such as (0.6, 0.35) so today's tile-entry repaint does not fall inside the 3-cycle allowance; several cycles are run after a jump back and after `reset()`; a case that cannot fail on today's code is kept only as a guard of the merge and gap logic and is labelled so in its comment, not counted as a regression test; one assertion guards the design's cost property: on steady diagonal travel the layered costmap's bounds are never the whole window. (2) Cell values are compared exactly only with no inflation layer; with one, lethal and unknown cells are compared (Jazzy's inflation over a sub-rectangle differs from a full pass by a few cost units). (3) The measurement harness names the source it patches (a worktree at `021ebe4`, or `jazzy` before the backport) in its usage text. (4) The state is reduced as the review suggests where behaviour is unchanged: one interval per axis, `have_window_` folded into `whole_pending_`; the implementer re-runs the review's adversarial scenarios (59 scenarios under 5 inflation and 4 other-layer set-ups) against the reduced form. (5) Commit A (a tile created while no chart is loaded is reported) goes in first, with its fixed-window test. Follow-up for the host to put to the owner: a waiting tile on a fixed-window costmap holds the layer not current indefinitely (pre-existing; BEN's costmaps are both rolling).

## Implementation
**Status**: complete
**When**: 2026-11-24 15:17 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Branch**: feature/issue-57 at `a65d4c9`

Design (d), as the owner chose at the plan checkpoint, with the plan review's reduced state. No `current_` change (commit B not made).

### Commits (each standalone for `git cherry-pick -x`; the three code commits touch nothing under `.agent/`, end "Backport to jazzy.", and each builds and passes the package's tests on its own)

1. `5b50b5e` s57_layer: report a tile created while no chart is loaded. The one line in `generateTile`'s empty-chart return; new test `UnchartedFixedWindowMarkedUnknown`; `UnchartedBelowOriginMarkedUnknown`'s comment reworded.
2. `8df1a96` s57_layer: widen the bounds earlier layers report, never narrow them. The clamp at `s57_layer.cpp:344-350` replaced by clip-own-rectangle-then-widen (a small `Rect` helper), split out as the checkpoint allowed; new test `BoundsFromEarlierLayersAreNotNarrowed`.
3. `3231db9` s57_layer: repaint the cells a rolling window uncovers (#57). Design (d), 10 `MovingWindowTest` cases, comment updates on the two one-cycle 0.1 m tests, the test CMake comment, one README line.
4. `a65d4c9` harness for #57 (records, `.agent/` only): builds the final source beside today's.

`s57_layer.cpp` +139/-18 and `s57_layer.h` +88/-0 against `origin/rolling`, comments included.

### The code
- **State (6 members, the review's reduced form):** `whole_pending_` (first cycle, `reset()`, `matchSize()`; replaces `have_window_`), `last_window_` (a `Rect`, cell edges), `pending_x_`/`pending_y_` (one `Span` per axis: the hull of uncovered columns/rows not yet reported, clipped to the window), `last_reported_` (none/x/y/whole; none in a gap) and `last_axis_`. Each member's invariant is in a comment in `s57_layer.h`.
- **Phases in `updateBounds`:** (1) `recordUncovered` adds this move's uncovered columns and rows to the spans and clips them, or returns false when the whole window must be reported (`whole_pending_`, or no overlap); (2) the tile loop, where a tile wholly outside last cycle's window is left to the spans; (3) `chooseStrip` (x, gap, y, gap while both pending; the same axis every cycle on travel along one axis); (4) clip the layer's own rectangle to the window, widen the shared bounds with min/max.
- **One place** (the comment above `chooseStrip`) says why x and y strips are never served in consecutive cycles: `InflationLayer::updateBounds` merges this and last cycle's bounds on both distros.
- The window's max edge comes from the cell count, not `getSizeInMetersX` (half a cell short on Jazzy).

### Tests (`test_rolling_window.cpp`; 20 x 20 window at 1 m, 4-cell tiles; each moving case under no inflation, `InflationLayer` r 2 m and r 6 m with `setFootprint`)
Every cell is compared after every cycle with a fresh costmap moved to the same origin in its first cycle: exact with no inflation layer, lethal and unknown cells with one. Charts are 200 m square with cell edges at x.25 (#63) and cover every tile reached (#59). Each path is checked step by step to give the same origin under Rolling's and Jazzy's size formulas, and the origin is asserted each cycle. (0.6, 0.35) m per cycle is impossible under that constraint: any step whose fraction is not 0 or 0.5 brings the offset's fraction into [0.75, 1), where the formulas truncate differently. The diagonals use (0.5, 0.5), (1.5, 0.5) and a reversal back at (-1, -1); a tile column or row enters only every 8 cycles at 0.5 m.

Failing output on `origin/rolling`'s layer source with the test file present (scratch build; `ldd` showed the scratch `libs57_layer.so`):
- `UnchartedFixedWindowMarkedUnknown`: "100 of 100 cells are not 255; window origin (0, 0); first wrong cell (0, 0) ... has cost 0".
- `BoundsFromEarlierLayersAreNotNarrowed`: the recorded bounds are 15 / 25 (clamped to the window), not -35 / 75.
- East, North, Reversal, Jump and back, Reset while moving, Late grid (all three set-ups each): "cycle 1, robot at (21.5, 20.5): 20 of 400 cells differ from the oracle ... first (30.5, 10.5) has cost 0 where the oracle has 254" (North: (20.5, 21.5), (10.5, 30.5)). They fail today at the first move, not at the feature each names; what each guards in (d) is below.
- `SlowDiagonalEveryCellWithinThreeCycles`: all set-ups, e.g. none "cycle 8, robot at (24.5, 24.5): 37 of 400 cells differ ... allowance 3 cycles, the oldest wrong cell has been in the window 3 cycles"; r6 on (1.5, 0.5) "cycle 8 ... 7 of 400".
- `SlowDiagonalTrackingUnknownMatchesAFreshWindow` (scattered land, `track_unknown_space`): none "cycle 4 ... 33 of 400 ... cost 255 where the oracle has 0"; r2 11, r6 7 cells at cycle 8.
- `DiagonalTravelNeverRepaintsTheWholeWindow` (the cost property, `LayeredCostmap::getBounds`, with a layer at the end recording whether the cycle had bounds at all, since `getBounds` keeps last cycle's through an empty cycle on both distros): today repaints the whole window at cycles "3 11 19 27 35" (0.5, 0.5, none), "2 3 6 7 10 11 ..." ((1,1), r2 and r6), "4 12 20 28 36" ((1.5, 0.5), r2 and r6). (1.5, 0.5) with no inflation does not fail today.
- `FixedWindowControl`: passes today, kept as the requested control; it guards nothing in (d).

Deliberate breaks of (d) (same scratch method), and which test fails:
- x and y served in consecutive cycles (no gap): only `DiagonalTravelNeverRepaintsTheWholeWindow`, "cycles 2 3 4 ... 39" (every cycle) under r2 and r6; cells stay correct, so this is the test that guards the gap.
- a gap after every strip (axis travel served late): East, North, Reversal, Jump, Reset, Late grid ("cycle 2 ... 20 of 400").
- the tile rule applied to every tile, or inverted: `LateGridForTilesInTheWindow` ("cycle 4 ... 64 of 400 ... cost 254 where the oracle has 0"); inverted also fails the cost test. The tile rule removed: only the cost test.
- no-overlap (jump) detection removed: `JumpPastTheWindowAndBack` ("cycle 2 ... 400 of 400").
- the low-side span not recorded: `ReversalPastTheStart` ("cycle 10 ... 20 of 400") and the slow diagonal.
- Not caught by any test or by the adversarial run: `reset()` not setting `whole_pending_` (its tile flags already repaint the window; the line covers tiles not generated yet), the span clip in `recordUncovered` (phase 4 clips anyway; it bounds the hull), and the half-cell margin on strips (Nav2's conversion covers cell-edge strips either way, also at 0.1 m). Kept as defensive, and said so in the comments.

Run time: the 12 new tests (with one existing uncharted test) take 0.38 s in one process; `test_rolling_window` 0.69 s under ctest. Each of the 12 new tests passed 30 of 30 as separate processes (99 s for all 360).

Full suite on Rolling (`colcon test --packages-select s57_layer --parallel-workers 4 --ctest-args -j4`, `ROS_DOMAIN_ID` 175): 9 of 9 ctest entries passed, including cppcheck, lint_cmake and xmllint (copyright, cpplint and uncrustify are skipped by the package's CMake); colcon test-result: 73 tests, 0 errors, 0 failures, 8 skipped (cppcheck's, as before). `ldd` showed the worktree's own installed `libs57_layer.so`, built from this source.

### Adversarial re-run (harness at `a65d4c9`, final source beside `afa7403`)
69 scenarios x 4 other-layer set-ups (none; a 3 m box before; a box after; a one-cell strip at the leading edge after) x 5 inflation set-ups (none; Rolling r2, r6; Jazzy's `InflationLayer` copied from `origin/jazzy`, r2, r9) = 1380 runs, 63 300 cycles, 91 s on 5 processes. Scenarios: 6 random walks (steps up to 3 m and 1.2 m), velocities (0.7, 1.3), (2.3, -1.7), (0.37, 0.11), zig-zags on x and x+y and reversals at each of 4 phases, stops at 5 phases, one-tile steps on 3 headings, a jump past the window and one half a window and back, a path across the origin and a walk around it, windows of 21 and 22 cells, tide step, chart-set change and `reset()` at each of cycles 3-7, late grids at the edge, under the robot and ahead at cycles 3-6, `track_unknown_space` fixed and moving, `allow_uncharted: false` moving, and two at 0.1 m cells.
- **Final:** oldest wrong cell age 2 in every set-up (painted within 3 cycles; no run had a wrong cell older), longest streak of one world cell wrong 3 cycles, 0 origin mismatches. With inflation, other value differences all lie within the radius of a cell younger than 4 cycles, except 2 cell-cycles under Jazzy r9 (the Jazzy partial-versus-full queue inflation the plan review noted).
- **Today:** wrong after age 2 in 213 (none), 130 (r2), 109 (r6), 130 (J r2), 54 (J r9) of 276 runs each; oldest wrong cell 134 cycles.
- The widest pending span reached the full window width (22 cells, a random walk back and forth within 3 cycles): that costs a whole-window strip, not a missed cell.
- **Before/after, cell by cell:** cell-cycles lethal today and not lethal with the fix: 14 647 (none), 6 659 (r2 both distros), 3 200 (r6), 1 745 (J r9). Every one of them is lethal in the fresh window, and none is in a cell that had been in the window 3 cycles or more. So the fix never removes a hazard the chart has; but a charted hazard that today's whole-tile report showed in the cycle its tile entered can, with the fix, appear up to 2 cycles later (the tile is left to the strips, which wait for their turn). The same counts hold for cells at or above the inscribed cost. Nothing marked today stays unmarked: no such cell is older than age 2.

### Measurements (desktop: Intel Core Ultra 7 265, Release, Rolling's source-built Nav2, single-threaded inflation), mean / worst ms per `updateMap`
Global 5000 x 5000 at 1 m, 1 Hz, 100 cycles:

| | E 2 m/s | E 5 m/s | 45° 2 m/s | 45° 5 m/s |
|---|---|---|---|---|
| today, Rolling inflation | 8.7 / 21.0 | 9.3 / 21.1 | 14.5 / **307** | 26.1 / **309** |
| final, Rolling inflation | 19.3 / 22.3 | 19.5 / 27.3 | 18.3 / 21.5 | 18.8 / 22.5 |
| today, Jazzy inflation | 8.8 / 31.6 | 10.2 / 32.1 | 14.8 / **345** | 28.9 / **359** |
| final, Jazzy inflation | 26.6 / 32.8 | 27.0 / 34.1 | 26.6 / 32.3 | 26.5 / 35.6 |

Local 700 x 700 at 0.5 m, 5 Hz, 300 cycles: today 0.10-0.18 / 3.0-4.1 (Rolling), 0.13-0.26 / 5.6-7.4 (Jazzy); final 2.6-2.7 / 2.9-3.6 (Rolling), 4.8-5.6 / 6.1-7.1 (Jazzy). No inflation, global, 45° 2 m/s: today 8.2 / 9.2, final 8.5 / 10.1.

The final code costs what the owner chose on, slightly less in absolute terms (18-19.5 / 21-27 against 22-26 / 28-30 Rolling; 26.5-27 / 32-36 against about 31 / 38-40 Jazzy), but today's figures in this run are lower by about as much (8.7 against 11.2 ms), so the difference is the machine's, not the code's: the fix adds about 10 ms (Rolling) and 17 ms (Jazzy inflation) per global cycle over today's typical cycle and removes today's 0.3-0.36 s diagonal spikes.

### Nav2 calls the tests depend on, `origin/jazzy` against Rolling's working tree (bodies read)
- `LayeredCostmap::updateMap`: same, except Jazzy returns early on empty bounds where Rolling skips the block; both leave `bx0_..byn_` unchanged then.
- `LayeredCostmap::getBounds`: same (inline, returns the last non-empty cycle's bounds on both; the test records emptiness with a last layer for that reason).
- `LayeredCostmap::resizeMap`, `addPlugin`, `isRolling`, `getCostmap`: same. `setFootprint`: same behaviour (atomic store spelled differently), calls `onFootprintChanged` on each plugin.
- `Costmap2D::updateOrigin`, `resetMap`, `worldToMapEnforceBounds`, `mapToWorld`, `getCost`: same. `getSizeInMetersX/Y`: differs (Jazzy `(size - 0.5) * res`); the layer no longer uses it for the window edge, and every test path gives the same origin under both formulas (checked in the test).
- `InflationLayer::onInitialize`: parameters declared if not declared on both, so the pre-declared `inflation_radius` is used; Rolling also has `custom_inscribed_radius` (unset). `updateBounds`: identical. `onFootprintChanged`: both set `need_reinflation_` (Rolling via `updateInscribedRadius`, Jazzy from the layered costmap; a warning versus an error when the radius is under the inscribed radius, not reached here). `reset()`: same (`matchSize`, not current, reinflate). `updateCosts`: different algorithms (Rolling distance transform, Jazzy queue), which is why the tests compare only lethal and unknown cells under inflation.
- `Layer` (the tests' `BoundsLayer`): the same pure virtuals and a protected `current_` on both. `Layer::initialize` takes the lifecycle node the existing tests already handle.
Jazzy CI on the pull request will be the first Jazzy compile and run of these tests.

### Differed from the plan or checkpoint
- The clamp is its own commit (`8df1a96`) with a contract test that fails today; the plan had it inside C with no test.
- (0.6, 0.35) m per cycle cannot keep Jazzy's and Rolling's origins equal; (0.5, 0.5) and (1.5, 0.5) are used, slow enough that today fails.
- The window edge uses the cell count, not `getSizeInMetersX`.
- `layer_variants.patch` is dropped from the harness (it is at `021ebe4`); designs (a) and (c) were not re-measured.
- The before/after comparison shows hazards can appear up to 2 cycles later than today's tile-entry report in the cycle a tile enters (above). The plan did not state this.

### Statements for the pull request
- **Design:** on a rolling costmap the layer reports the cells each move of the window uncovers as thin strips, columns or rows, never both in consecutive cycles (the inflation layer would merge them into the whole window). On travel along one axis they are painted in the same cycle; otherwise within 3 cycles. The first cycle, a reset and a jump past the window report the whole window.
- **Cost (desktop timings, not a vessel computer):** global costmap 18-19.5 ms mean / 21-27 ms worst per cycle with Rolling's inflation and 26.5-27 / 32-36 ms with Jazzy's, against today's 8.7-26 / 21-31 ms typical and 307-359 ms worst on diagonal travel; local costmap 2.6-5.6 / 2.9-7.1 ms per cycle.
- **For a vessel on BEN's two costmaps:** the global costmap's leading-edge band, and the far side after a reversal, show charted land within 3 s (3 cycles at 1 Hz) instead of after up to a tile of travel or never; the local costmap within 0.6 s. Nothing that is charted stays unmarked longer than 3 cycles; a hazard today's tile-entry report showed at once may appear up to 2 cycles later. Tiles waiting for a chart grid are now painted unknown when the strip reaches them (as any reported tile always was).
- **Not established:** a vessel computer's timings; how often the tide estimate steps over 1 cm (each step reports the whole window, as today); Jazzy's `LayeredCostmap`/`Costmap2D` under this design until Jazzy CI runs; anything on a vessel or with a planner.
- **What the tests pin:** every cell after every cycle on axis-aligned travel, reversal, jump and back, reset, a late grid; each cell within 3 cycles on slow diagonals and every cell after 3 cycles stopped; the bounds never the whole window on steady diagonal travel; earlier layers' bounds not narrowed; the uncharted fixed window. **Not pinned:** inflation cost values (only lethal and unknown under inflation), timing, the reset line, the span clip and the half-cell margin (defensive), resolutions other than 1 m (the adversarial run covers 0.1 m), BEN's tile size.

### Left alone
rolker/s57_tools#59, #63 and the fixed-window `current_` stickiness: not touched. The tests keep them out by charts whose cell edges lie a quarter cell off the tile edges and that cover every tile a path reaches; no test depends on `isCurrent()`.

### Clean-up
No `build/`, `install/` or `log/` left in the package worktree; scratch files removed; no ROS daemon started.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-24 15:41 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: changes-requested

**Branch**: feature/issue-57 at `ae38308` (code head `3231db9`)
**Base**: origin/rolling
**Depth**: Deep (reason: hazard-marking code on a deployed vehicle, cross-distro backport; per handoff)
**Must-fix**: 2 | **Suggestions**: 3
**Round**: 1 | **Ship**: continue — round 1: 2 must-fix; first round always re-reviews after fixes

Reviewers: static (cppcheck: nothing on changed lines), governance, plan drift, Claude adversarial subagent (no must-fix; a 3,000-walk model of the strip logic at 0.05-1 m and coordinates to ±1e7 found no cell older than age 2), Gemini and Codex (both complete). All scratch work in the session scratchpad; nothing left in the tree.

### What I ran
- The committed harness (`measure.sh`, final beside `afa7403`): reproduces the implementation entry exactly (69 scenarios x 4 x 5; final oldest wrong age 2, streak 3; before/after 14 647 / 6 659 / 3 200 / 6 659 / 1 745).
- The harness extended with 46 scenarios (random walks of 0.9 and 4.5 m steps, three non-integer velocities to 200 cycles, stop-then-reverse and east-then-north at each phase, 0.5 m, 1e6 / -2e6 m from the origin at 1 m and 1e5 at 0.1 m, track_unknown with a late grid, allow_uncharted false on a diagonal, reset and chart-set change at each of 8 cycles of a slow diagonal): 460 runs per inflation set-up, final still oldest wrong age 2, streak 3, 0 origin mismatches.
- The test file compiled with the layer source (no `libs57_layer.so`, checked with `ldd`) for 15 variants: `origin/rolling`, A and B reverted, and 11 breaks.
- Timings, global 5000 x 5000 at 1 m, 1 Hz, desktop, with a layer after the inflation layer counting whole-window cycles: east and 45 deg at 2 m/s, a lawnmower (200 m legs, 20 m-radius turns, 2.75 m/s) and station keeping (position noise +-1 m per cycle).

### The later-marking case (for the owner)
- **When.** A cell is lethal today and not yet lethal with the fix only in this situation: a tile enters the window, today reports the whole tile in that cycle, and with the fix the tile is left to the strips (the tile rule) and its strip waits its turn (x, gap, y, gap). It needs travel on both axes in the cycles concerned, or a move larger than a step. Never on travel along one axis.
- **Worst delay: 3 cycles, not 2.** Such a cell is wrong at ages 0, 1 and 2 (max streak 3, 460 x 5 runs) and lethal at age 3; today it was lethal at age 0. On BEN: 3 s on the global costmap, 0.6 s on the local.
- **Where.** On continuous travel, within 3 cycles of travel of the leading edge: on BEN at least 2500 - 8 m from the boat (global) and 175 - 1.7 m (local). After a diagonal jump shorter than the window (a localisation jump), the uncovered region is up to the window's half width deep, and cells up to the boat can wait 3 cycles (in the 20-cell harness, a delayed cell 9 cells from the edge, beside the robot). On the local costmap (track_unknown off) a delayed cell reads free, not unknown, and `current_` is not lowered.
- **Same event as the spike.** Yes. Removing only the tile rule (the newly entered tile reported at once, as today) leaves 0 such cells and brings the whole-window cycles back at today's cadence: the cost test with the rule removed lists cycles "3 11 19 27 35" at (0.5, 0.5), the same list today's layer gives. Reporting the new tile is a full-height tile column (or full-width row); with the other axis's strip this cycle or last, Nav2's inflation merge makes it the whole window. Painting those cells sooner means either that spike or delaying the other axis instead. The trade the owner confirms: these cells up to 3 cycles later than today, in exchange for no whole-window repaint on diagonal travel; every other uncovered cell is painted far sooner than today (today: up to a tile of travel, or never).
- **Small change for the jump case only.** Treat a move of more than a tile (or some N cells) on both axes as a jump and report the whole window: whole-window cost only on such jumps, and nothing near the boat waits.

### Measurements (mean / worst ms per updateMap; implementer's in brackets)
| Global, 1 Hz | today Rolling infl. | final Rolling | today Jazzy infl. | final Jazzy |
|---|---|---|---|---|
| E 2 m/s | 8.7 / 21.6 (8.7 / 21.0) | 19.6 / 22.5 (19.3 / 22.3) | 10.6 / 34.6 (8.8 / 31.6) | 29.7 / 37.0 (26.6 / 32.8) |
| 45 deg 2 m/s | 14.5 / 309, 2 whole (14.5 / 307) | 18.8 / 23.1, 0 whole (18.3 / 21.5) | 14.9 / 333 (14.8 / 345) | 26.9 / 32.7 (26.6 / 32.3) |
| lawnmower 2.75 m/s | 8.8 / 21.5 | 20.4 / 23.8, 0 whole | 9.1 / 33.6 | 26.4 / 37.7, 0 whole |
| station keeping +-1 m | 8.3 / 11.1 | **65.2 / 314, 26 of 150 whole** | 8.8 / 11.1 | **76.7 / 355, 26 of 150 whole** |

### Findings
- [x] (must-fix) An axis reversal re-inflates the whole window: an x strip on one edge then an x strip on the opposite edge next cycle (InflationLayer merges this and last cycle's bounds; Codex), and a pending span that becomes the hull of both edges because `recordUncovered` clips after adding (a scrolled-out column joins a new one on the other side; Gemini). Measured: station keeping on the global costmap repaints the whole window 26 of 150 cycles, 65 / 314 ms (Rolling), 77 / 355 ms (Jazzy) against today's 8 / 11; the owner chose on "whole-window spikes gone". Prototyped in the harness: clip the spans before adding, and no strip in the cycle after a strip on the other side of the same axis (2 members, ~6 lines): 0 whole cycles, 15 / 23 ms (Rolling), 22 / 34 ms (Jazzy); over 460 runs x 5 set-ups the oldest wrong cell stays age 2, and adversarial whole-window cycles drop from 5050 to 3541 (Rolling r2; today 3068). Add a jitter case to `DiagonalTravelNeverRepaintsTheWholeWindow`. — `s57_layer/src/s57_layer.cpp:429-436, 454-469`
- [x] (must-fix) Statements to correct: README item 5 and the `chooseStrip` comment say the inflation layer "never" re-inflates the whole window (false on reversals, see above; and tide steps or chart-set changes still report the whole window); the implementation entry's PR text "may appear up to 2 cycles later" should be up to 3 cycles later than today, and should say where (within 3 cycles of travel of the leading edge, or anywhere in the uncovered region after a diagonal jump), that delayed cells on a costmap without track_unknown read free, and that nothing marked today stays unmarked beyond that. — `s57_layer/README.md:38`, `s57_layer/src/s57_layer.cpp:440-453`, `progress.md` Implementation
- [x] (suggestion) A diagonal jump shorter than the window leaves cells up to the boat unpainted for up to 3 cycles; report the whole window when a move exceeds a tile (or N cells) on both axes. — `s57_layer/src/s57_layer.cpp:419-422`
- [x] (suggestion) Tests: no break of these is caught by any test: `reset()` and `matchSize()` setting `whole_pending_` (the tile flags cover the window anyway), the span clip (cosmetic today; load-bearing after the must-fix, then guarded by a jitter cost case), the half-cell margin (Nav2's truncate-then-+1 covers cell edges), and `last_window_` updated only on strip cycles (a correctness break: a reversal across a gap cycle; the harness's zig-zags catch it, the unit tests do not). Keep the first, second and fourth lines as labelled defensive code; add a fast zig-zag case to the every-cell tests. — `s57_layer/test/test_rolling_window.cpp`
- [x] (suggestion) In `updateBounds`' `!enabled_` early return, set `whole_pending_ = true` so a disabled spell cannot leave `last_window_` stale. Unreachable today (`enabled_` is read only in `onInitialize`). — `s57_layer/src/s57_layer.cpp:187-188`

### Checked and found sound
- **Teeth.** On `origin/rolling` 11 tests fail as the entry says (East, North: "cycle 1 ... 20 of 400 ... first (30.5, 10.5) has cost 0 where the oracle has 254"); A reverted fails only `UnchartedFixedWindowMarkedUnknown`; `5b50b5e`'s source fails `BoundsFromEarlierLayersAreNotNarrowed` (given 15 / 25, not -35 / 75). Breaks caught: gap after every strip (6 tests), no gap (slow diagonals and cost test), x always first (slow diagonals), low-side y span missing (slow diagonal reversal), x strip a row short (6 tests), tile rule off (cost test only, same cycles as today).
- **Jazzy.** `getSizeInMetersX` is `(size - 1 + 0.5) * res` on `origin/jazzy`; the layer uses it only for tiles and the chart request now. `updateMap` differs only in returning early on empty bounds (both keep `bx0_..byn_`), `isCurrent` and default-value spelling; `InflationLayer::updateBounds` identical; `getBounds` identical, so the recording layer in the cost test is sound on both. Every `drive` path asserts equal origins per cycle; the cost test's three paths and both reversals re-derived: equal at every step.
- **Commits A and B.** Each correct with a test that fails without it; B changes nothing on BEN (chart_layer first) and stops the "Illegal bounds change" a later position would cause.
- **Threads.** The new members are touched only in `updateBounds`, `reset` and `matchSize`, all under the costmap mutex (`updateMap`, `clearEntirely`); the grid and dataset callbacks do not touch them.
- **Hygiene.** No `.agent/` file in the three code commits; each ends "Backport to jazzy.". Not taken up: Gemini's extra tile at a window edge on a tile boundary (pre-existing, cleared by `updateCosts`), Codex's stale trailing-edge inflation (Nav2's behaviour on any rolling costmap).

## Implementation
**Status**: complete
**When**: 2026-11-24 16:23 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**Branch**: feature/issue-57 at `bbc87c5`
**Addressed**: Local Review (Pre-Push) at `ae38308` (2026-11-24 15:41 -05:00)
**Commits**: `9b54db7` `5abf172` `db8c5c2` `f480aea` `251aa67` (code, each ends "Backport to jazzy.", none touches `.agent/`); `4cdda55` (harness, records); `bbc87c5` (boxes, records)

### Actions
- [x] (must-fix) Axis reversal re-inflates the whole window — `s57_layer/src/s57_layer.cpp` `recordUncovered`, `chooseStrip`; `s57_layer.h` (`9b54db7`)
- [x] (must-fix) Statements — README item 5 (`251aa67`), the `chooseStrip` comment (`9b54db7`, `5abf172`), the pull-request statements below
- [x] (suggestion) A jump: whole window on a move of more than a tile on either axis — `recordUncovered` (`5abf172`)
- [x] (suggestion) Tests and defensive lines: fast zig-zag every-cell case (`9b54db7`); `reset()`, `matchSize()` and the half-cell margin labelled defensive (`f480aea`); the span clip now caught (station keeping and zig-zags)
- [x] (suggestion) `whole_pending_ = true` on the `!enabled_` return, labelled unreachable (`db8c5c2`)

### Fix 1: what was built, and why not the prototype
- **State:** four spans, one per window edge (`pending_[x_low, x_high, y_low, y_high]`), each with an `age` (cycles its oldest cell has waited), and `last_reported_` (which strip, none, or whole). `last_axis_` is gone. Spans are clipped to the window and aged before a move's cells are added.
- **Rule (`chooseStrip`):** a strip follows last cycle's only if it is the same strip and nothing else is pending; otherwise a gap. After a gap (or a whole report) the oldest strip goes; if two strips have cells that have waited 2 cycles, the whole window is reported instead. Proof of the bound: the next chance for a different strip after one is served is two cycles later, so a strip passed over at age 2 or more would be painted at age 4; the escape prevents it, so every uncovered cell is painted at age 3 at most (wrong at ages 0, 1, 2).
- **One span per axis was not adequate.** I built the reviewer's prototype first (clip before add; no strip after a strip on the other side of the same axis; one flag for the side). A diagonal turned back on both axes then joins a still-pending north row with the new south row into a hull spanning the window, and zig-zags do the same. Measured on the extended harness (105 scenarios x 4 other-layer set-ups, before the jump rule): whole-window cycles, Rolling r2, today 2587, one span per axis 2575, one per side 1627; zig-zag x+y phase 3 88 against 17; fast zig-zag x+y 3 m 84 against 0. Oldest wrong cell age 2 in both. Station keeping on the global costmap: 0 whole-window cycles with one per side.
- **What is still whole:** a diagonal turned back on both axes in one cycle (all four sides pending; the escape fires once: the unit test pins one report, cycles " 11" with no inflation, " 11 12" with it). On the local costmap, station keeping with +-1 m of noise every 0.2 s (moves of up to 4 cells per cycle on both axes) gives 24 whole-window cycles of 300, costing 3.5 ms worst (Rolling) and 5.2 ms (Jazzy) on that 700 x 700 window. On the global costmap station keeping gives 0 of 150.
- **Worst delay:** 3 cycles, unchanged. On travel along one axis 0, or 1 in the cycle the travel turns back (the reversal test now allows 1 cycle).

### Fix 3: the jump rule
A move of more than a tile (`tile_size * resolution`, plus half a cell) on either axis in one cycle reports the whole window. Either axis, because a jump along one axis would otherwise wait a cycle behind the other axis's strip of the cycle before: the test's second case (travel north, then a 10 m jump east) fails with the rule on both axes only. BEN never triggers it in travel: at 5 m/s the global costmap (1 m, 1 Hz) moves 5 cells a cycle and the local one (0.5 m, 5 Hz) 2, against 100-cell tiles. The no-overlap test stays for a tile as wide as the window (removing it alone is caught by nothing now; removing both is caught).

### Tests (`test_rolling_window.cpp`) and their failures without their fix (scratch builds compiling the layer source into the test; `ldd` shows no `libs57_layer.so`)
- `WholeWindowTest.ReversalNeverRepaintsTheWholeWindow` (east then west, north then south, first step back 2 m so the window turns back at once; north-east, a stop, south-west): on the reviewed layer (`3231db9`) "these cycles repainted the whole window: 8" under r2 and r6, east-west and north-south. A 1 m step back from x.5 truncates to no move, a free gap, so the first version of this test passed on the reviewed layer; that is why the step is 2 m.
- `WholeWindowTest.StationKeepingNeverRepaintsTheWholeWindow` (+-1 m around (30.5, 30.5), a fixed pseudo-random sequence, 80 cycles): reviewed layer "7 15 17 19 37 53 55 59" (no inflation), 22 cycles under r2 and r6. Without the clip: fails. With a strip allowed after one on the other side of the same axis: "8".
- `WholeWindowTest.SharpDiagonalReversalRepaintsTheWholeWindowOnce`: reviewed layer " 9" / " 9 10" against the expected " 11" / " 11 12".
- `WholeWindowTest.DiagonalTravelNeverRepaintsTheWholeWindow`: moved into the new fixture; every cost case now asserts the window origin each cycle under both size formulas.
- `MovingWindowTest.FastZigZagsEveryCellWithinTheBound` (+-2 m reversing every 1 and 2 cycles, x only within 1 cycle, x+y within 3): with `last_window_` updated only on strip cycles (the reviewer's break) "cycle 16, robot at (22.5, 20.5): 20 of 400 cells differ ... allowance 1 ... first (31.5, 10.5) has cost 0 where the oracle has 254". Passes on the reviewed layer (a guard, not a regression test).
- `MovingWindowTest.JumpShorterThanTheWindowEveryCellEveryCycle` (10 m jumps, the robot's own column uncovered; every cell every cycle): without the rule "cycle 4, robot at (33.5, 30.5): 300 of 400 cells differ ... first (33.5, 20.5) has cost 0 where the oracle has 254" and "(30.5, 23.5): 200 of 400 ... first (30.5, 13.5)"; with the rule on both axes only, the second.
- `ReversalPastTheStartEveryCellEveryCycle` became `...WithinOneCycle` (allowance 1).
- Every path in `drive` and in the cost helper asserts, each cycle, that Rolling's `size * resolution` and Jazzy's `(size - 1 + 0.5) * resolution` give the same origin and that the window has it; all pass, so each path (the jitter at x.5 +- whole metres included) was re-derived step by step by the test itself.

### Deliberate breaks of the final source (16), and what catches each
gap after every strip: East, North, FastZigZags, JumpPastTheWindow, JumpShorter, Reset, LateGrid · no gap: Diagonal, Reversal, SharpDiagonal, StationKeeping (every cycle whole) · other side of the same axis allowed next: Reversal, StationKeeping · tile rule removed: Diagonal ("3 11 19 27 35"), Reversal, SharpDiagonal, StationKeeping · low-side spans not recorded: ReversalPastTheStart, FastZigZags, SlowDiagonal, SharpDiagonal · no clip: FastZigZags, StationKeeping · `last_window_` only on strip cycles: FastZigZags · ages not kept: FastZigZags, both SlowDiagonals, SharpDiagonal · escape removed: FastZigZags ("cycle 18 ... allowance 3 cycles, the oldest wrong cell has been in the window 3 cycles"), SharpDiagonal · jump rule removed: JumpShorter · jump rule on both axes: JumpShorter · jump rule and no-overlap test removed: JumpPastTheWindow, JumpShorter.
**Caught by nothing:** `reset()` and `matchSize()` setting `whole_pending_`, the half-cell margin (labelled defensive, `f480aea`; a resize while running reaches the `matchSize` line but no test drives one), the no-overlap test alone (covered by the jump rule while a tile is narrower than the window), and the `!enabled_` line (unreachable).

### Runs on the final code
- Package suite on Rolling, package worktree (`build.sh --packages-up-to s57_layer --parallel-workers 4`; `test.sh --packages-select s57_layer --parallel-workers 4 --ctest-args -j4`, `ROS_DOMAIN_ID` 177): 9 of 9 ctest entries passed; colcon test-result 78 tests, 0 errors, 0 failures, 8 skipped (cppcheck's). `ldd` showed the worktree's own installed `libs57_layer.so`. `build/`, `install/`, `log/` removed afterwards.
- The 7 new or changed tests, 30 runs each as separate processes: 30/30 each (68 s).
- Harness adversarial set (`4cdda55`, final beside `afa7403`): 105 scenarios x 4 other-layer set-ups x 5 inflation set-ups = 2100 runs, 89 060 cycles, 149 s on 5 processes.

| | none | Rolling r2 | Rolling r6 | Jazzy r2 | Jazzy r9 |
|---|---|---|---|---|---|
| today: oldest wrong age / streak | 134 / 135 | 127 / 128 | 102 / 125 | 127 / 128 | 76 / 125 |
| final: oldest wrong age / streak | 2 / 3 | 2 / 3 | 2 / 3 | 2 / 3 | 2 / 3 |
| whole-window cycles today / final | 1120 / 1075 | 2587 / 1718 | 2910 / 2524 | 2587 / 1718 | 10101 / 11170 |

Per scenario group, Rolling r2, oldest wrong age / longest streak / whole-window cycles summed over the 4 other-layer set-ups (with no other layer alone), today -> final: station keeping (4) 22/23/0(0) -> 2/3/91(0); lawnmower 0° 34/35/4(1) -> 2/3/46(0); lawnmower 30° 36/39/30(6) -> 2/3/32(0); fast zig-zag x 1-3 m 24/29/0 -> 0/1/0(0); fast zig-zag x+y 1-3 m 24/29/4(1) -> 1/2/0(0); stop then reverse (4 phases) 8/9/72(15) -> 2/3/27(0); reversal (4 phases) 8/9/129(26) -> 2/3/56(4, the sharp diagonal escape); sub-window jumps on one axis (6.2, 0), (0, -7.3), (4.6, 0.4) and both (6.2, 5.1), (-9.1, 8.7) at 4 phases each: final oldest age 2 or less, 2 whole cycles per run with no other layer for a jump of more than a tile (the jump and the inflation merge's next cycle), 1 per run for (4.6, 0.4); random walks of 3 m steps 102/124/76(12) -> 2/3/373(54); tide step, chart-set change, reset at 5 cycles each -> 40(10), 40(10), 20(5). Whole-window cycles with another layer come mostly from set-up 3 (a layer reporting a strip at the east edge every cycle, whose union with this layer's west strip is the window). Jazzy r9 on a 20-cell window repaints nearly everything whatever the layer does. The full per-scenario lines are in the run's output (`PER_SCENARIO=1`), not kept.
- Before/after: cell-cycles lethal today and not lethal in the final code: 19 738 (none), 7 932 (r2, both distros), 4 217 (r6), 2 176 (J r9); every one lethal in the fresh window, none in a cell that had been in the window 3 cycles or more. 0 origin mismatches.

### Measurements, desktop (Intel Core Ultra 7 265, Release, Rolling's source-built Nav2), mean / worst ms per `updateMap`, whole-window cycles in brackets
Global 5000 x 5000 at 1 m, 1 Hz, 150 cycles:

| | E 2 m/s | E 5 | 45° 2 | 45° 5 | lawnmower 2 | lawnmower 5 | station keeping +-1 m |
|---|---|---|---|---|---|---|---|
| today, Rolling inflation | 8.6 / 21.1 | 9.3 / 23.6 | 16.2 / **310** (4) | 28.1 / **309** (10) | 8.5 / 20.9 | 8.6 / 20.9 | 8.2 / 10.6 |
| final, Rolling inflation | 19.3 / 22.1 | 19.5 / 22.5 | 18.0 / 22.4 | 18.8 / 23.9 | 19.0 / 21.7 | 19.2 / 24.1 | 15.3 / 19.9 (0) |
| today, Jazzy inflation | 8.9 / 31.7 | 10.0 / 34.0 | 16.9 / **349** (4) | 31.6 / **378** (10) | 8.7 / 31.4 | 9.1 / 31.7 | 8.2 / 9.6 |
| final, Jazzy inflation | 26.3 / 34.8 | 27.6 / 33.7 | 25.4 / 32.2 | 27.1 / 36.7 | 25.8 / 32.2 | 26.2 / 33.9 | 22.4 / 33.7 (0) |

All final global runs: 0 whole-window cycles. The reviewed code (`3231db9`), reviewer's measurement: station keeping 65.2 / 314 (Rolling), 76.7 / 355 (Jazzy), 26 of 150 whole. No inflation, global, 45° 2 m/s: today 8.1 / 9.0, final 8.2 / 8.9.

Local 700 x 700 at 0.5 m, 5 Hz, 300 cycles (lawnmower 40 m legs, 10 m turns): today 0.06-0.17 / 3.0-4.1 (Rolling), 0.06-0.26 / 4.6-7.2 (Jazzy), 4 and 8 whole cycles on the 45° runs; final 2.58-2.66 / 2.9-3.5 (Rolling), 4.6-5.6 / 5.2-7.2 (Jazzy), 0 whole except station keeping, 24 of 300 (worst 3.5 Rolling, 5.2 Jazzy).

### Statements for the pull request (replacing the earlier entry's)
- **Design:** on a rolling costmap the layer reports the cells each move of the window uncovers as thin strips at the window's edges (west, east, south, north), one per cycle, with a gap cycle between different strips, because Nav2's inflation layer merges each cycle's bounds with the last and two different strips together span the window. After a gap the strip waiting longest goes next. The whole window is still reported on the first cycle, after a reset, on a jump of more than a tile on either axis, on a tide step over `tide_invalidate_threshold`, on a chart-set change, and once when diagonal travel turns back on both axes in one cycle (and on the local costmap during violent station keeping, see the cost).
- **Delay bound:** every uncovered cell is painted within 3 cycles (wrong at most in the cycle it is uncovered and the 2 after). On travel along one axis in the same cycle, or the next when that travel turns back.
- **Later than today:** a charted hazard can appear **up to 3 cycles later** than with today's code, only for cells of a tile newly entering the window when today reports that whole tile at once and the strips make it wait: travel with movement on both axes, or a move larger than one step (up to a tile). Never on travel along one axis (cells uncovered by a reversal are in tiles already generated, which today never repaints). Where: within 3 cycles of travel of the leading edge, on BEN at least 2485 m from the boat on the global costmap and 172 m on the local at 5 m/s; after a jump of up to a tile, within a tile of the edge (at least 2400 m and 125 m); a larger jump is now reported whole at once. On a costmap without `track_unknown_space` (BEN's local) a delayed cell reads free, not unknown, and `current_` is not lowered. Nothing marked today stays unmarked longer (harness: no such cell older than age 2).
- **Cost (desktop, not a vessel computer):** global costmap 18-19.5 ms mean / 22-24 ms worst per cycle with Rolling's inflation and 25-28 / 32-37 ms with Jazzy's, on east, 45°, lawnmower at 2 and 5 m/s; station keeping 15.3 / 19.9 and 22.4 / 33.7, with no whole-window cycle in any of these; against today's 8.5-31.6 mean and 309-378 ms worst on diagonal travel (8.2 / 10.6 station keeping). Local costmap 2.6-5.6 / 2.9-7.2 ms.
- **What the tests pin:** every cell after every cycle on axis-aligned travel, a jump past the window and back, a jump of 10 m, a reset and a late grid; within one cycle after a reversal and on east-west zig-zags; within 3 cycles on slow diagonals, a diagonal reversal and diagonal zig-zags; no whole-window bounds on steady diagonal travel, axis reversals, a stop-then-reverse diagonal and station keeping; exactly one whole-window report on a sharp diagonal reversal; window origins equal under both distros' size formulas each cycle; earlier layers' bounds not narrowed; the uncharted fixed window. **Not pinned:** inflation cost values (only lethal and unknown under inflation), timing, the `reset()` and `matchSize()` lines and the half-cell margin (defensive), the `!enabled_` line (unreachable), a resize while running, resolutions other than 1 m (the harness covers 0.1 m), BEN's tile size.
- **Not established:** a vessel computer's timings; how often BEN's real position noise or a tide estimate step triggers a whole-window report; Jazzy's `LayeredCostmap`/`Costmap2D` under this design until Jazzy CI runs (Jazzy's `InflationLayer` is the copied one); anything on a vessel or with a planner.
- **For the owner (unchanged, his to confirm):** the tile rule, a tile wholly outside last cycle's window left to the strips, is what trades those up-to-3-cycles-later cells for no whole-window repaint on diagonal travel. Not changed here.

### Checks
No hook suite: this is a project pass with no `conventions=` hook rule; every commit ran the repository's pre-commit hooks. Scratch work in the session scratchpad (`fix57/`), removed at the end; no ROS daemon started; `DISPLAY` unset; domains 171-178 and 187-189.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-24 17:13 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: changes-requested

**Branch**: feature/issue-57 at `b467bfe` (code head `251aa67`)
**Base**: origin/rolling
**Depth**: Deep (reason: round 2, the strip scheduling replaced in hazard-marking code on a deployed vehicle; per handoff)
**Must-fix**: 1 | **Suggestions**: 4
**Round**: 2 | **Ship**: recommended — round 2: 1 mechanical must-fix (prev 2), not rising; fix the statements and ship

Scope: `git diff 3231db9..251aa67 -- . ':!.agent'` and the scheduling as it now stands. Reviewers: static (cppcheck: nothing on changed lines), governance and plan drift (no deviation beyond the recorded fix pass), a Claude adversarial subagent (no must-fix; exhaustive state search on three window/tile sizes, floating-point walks at 0.05-1 m and origins to 4.7e6, every rational velocity to 12 cells/cycle: no cell wrong at age 3, no whole window on steady travel), Gemini and Codex (both complete). Scratch work in the session scratchpad; nothing left in the tree.

### The 3-cycle bound
- **Invariant:** every valid span's `age` is at least the age of every cell in it (`add` resets it only when the span was empty, a clip only removes cells, a report or a whole window clears the span); a cell is painted in the cycle its span is reported. After a strip the next cycle is that strip again only when nothing else is pending, otherwise a gap; after a gap (or whole) the oldest span goes. So a span passed over after a gap waits a gap and is next oldest-or-whole two cycles later: a span of age a not chosen would be served at a+2. `chooseStrip` reports the whole window when a second span has age >= 2 (the chosen one being the oldest), so no span is left pending at age >= 3 after a report: every cell is painted at age <= 3, wrong at most at ages 0, 1, 2. Moves over a tile or past the window report the whole window (`recordUncovered`).
- **Proved by exhaustion** for the logic (integer model, copied from `recordUncovered`/`chooseStrip`, every move up to a tile per axis, BFS to closure): N=20 T=4 (14 982 states), N=23 T=7 (99 258), N=12 T=20 (tile wider than the window, 175 950), N=30 T=3: no span pending at age 3 after any report.
- **Runs:** the harness's 105 scenarios (reproduced exactly: final oldest wrong age 2, streak 3, whole 1075 / 1718 / 2524 / 1718 / 11 170) plus 255 of mine x 6 other-layer set-ups (the 4 plus a full-height strip at the west edge before this layer and strips alternating west/east edges after it) x 5 inflation set-ups = 7650 runs, 307 560 cycles: station keeping at +-0.2..3 m (1 m and 0.5 m cells, at a tile edge, x only), random walks to +-5 m (across the jump threshold), diagonal reversals every 1-4 cycles in and out of phase, spirals, stop k=0..5 then reverse at 4 phases, moves of 4/4.4/4.6/5/10 m on each axis and both (and at -60 m), tiles of 20 and 25 with moves of 9-21 m, tiles 3 and 7 on windows 20 and 23, 0.5 m and 0.1 m across the origin, tide/chart set/reset at cycles 2-12 of a walk and of a reversal, late grids on a walk, 0.1 cell/cycle drift. **Oldest wrong lethal/unknown cell age 2 in every set-up; 0 cell-cycles lethal today and not in the final code among cells in the window 3 cycles or more; 0 origin mismatches.** Longest wrong streak 4 under Rolling r6 and Jazzy r9: inflation values (not lethal/unknown) beside a cell painted at age 3, all explained by a young cell within the radius (value_diffs_unexplained 0; 1 under Jazzy r9 on a 20-cell window, as in the implementer's run).

### Complexity: is each rule needed (scratch copies, harness base set, whole-window cycles no inflation / Rolling r2, final 1075 / 1718)
- Oldest first replaced by index order: bound broken (oldest wrong age 28; model violation in 4 moves). Needed.
- Escape removed: oldest wrong age 4. Needed. Escape at age >= 3: tests fail (FastZigZags, SharpDiagonal). The threshold is tight.
- Same strip again (only when alone) removed, always a gap: bound holds, 1052 / 1711 whole, but axis travel paints one cycle late (East, North and 5 more tests fail). It is the design's "same update on travel along one axis"; needed for that term only.
- Jump rule removed: bound holds, 1026 / 1627 whole (fewer); loses the jump-near-the-robot guarantee (JumpShorter fails). Needed for that.
- Clip after add instead of before: identical results and every test passes. With one span per side the order no longer matters (a side's new cells are added only when the window moves toward it); the clip itself is needed (the implementer's no-clip break).
- **The host's simpler formulation** (one strip a cycle, a gap between different strips, oldest first, whole when anything would exceed the bound) **is exactly what the code does**: given oldest first, "a second span has waited 2" is "something would exceed 3". The variant without oldest first (index order, whole when any other span has waited 2) keeps the bound (model and harness) but 3428 / 6167 whole cycles, 3.6x. I found no simpler form with the same guarantees; the escapes below are inherent to a 3-cycle bound with full-length strips and a merge of two cycles' bounds (steady diagonal travel already serves each edge at age 3, so any third edge pending forces a whole window).

### Whole-window cycles by class, no other layer, today -> final (BEN sizes; global 600 cycles at 1 m, local 1500 at 0.5 m, no inflation, so each count is a report; with inflation each costs 2 cycles)
- Steady travel at 0, 10, 30, 45, 60 deg at 2 m/s, 45 deg at 5 m/s, drift 0.1 and 0.5 m/s, lawnmowers at 0, 30, 45 deg, circles r 20 and 50 m, turn in place, and all of these with +-0.2 or +-0.5 m noise: global today 0-21 (45 deg: 8 at 2 m/s, 21 at 5 m/s; noisy 45 deg 5-8), final 0; local likewise final 0 except one 10 deg 2 m/s +-0.5 m run, 3.
- Station keeping (uniform per cycle): +-0.2, 0.5, 1 m: 0 everywhere, today and final. **+-2 m on the global: 17, 11, 12 of 600 (final) vs 0 today. Local +-1 m: 43, 36, 44 of 1500; +-2 m: 117, 104, 103** (today 0).
- Why the local gets them and the global did not: displacement per cycle in cells, not the update rate. The logic sees only window moves; +-1 m is +-2 cells at 0.5 m, so the window takes 3-5 origins per axis and all four edges pend; at 1 m cells +-1 m gives mostly 2 origins. +-2 m on the global (2 cells) reproduces it. The adversarial subagent: 2 origins per axis never report the whole window (exhaustive); 3 do.
- **Timings (Rolling inflation / Jazzy's copied, mean / worst ms, 150 global or 300 local cycles; implementer's in brackets):** global station keeping +-1 m today 8.1 / 10.2, final 15.5 / 20.1 (15.3 / 19.9), Jazzy 22.2 / 31.2 (22.4 / 33.7), 0 whole; **+-2 m final 45.2 / 317, 14 of 150 whole; Jazzy 57.1 / 348** (today 8.5 / 21.0); steady 0 / 30 / 45 deg at 2 m/s final 19.3 / 22.2, 18.4 / 22.2, 18.3 / 22.7 (today 45 deg 16.2 / 312, 4 whole); lawnmower 45 deg 18.4 / 34.0 (today 12.2 / 310, 2 whole). Local station keeping +-1 m final 2.6 / 3.5, Jazzy 4.7 / 8.9 (3.5 / 5.2), 24 of 300; +-2 m 2.7 / 3.5, 46 of 300; +-0.5 m 0. On the local window a whole repaint costs about as much as a strip, so it does not matter there. On the global it is the 0.3 s spike, at about 9% of cycles, if BEN's position really jitters 2 m per second (not established; an INS-blended fix normally does not).
- Turns: the model shows tight turns from oblique travel trigger it (90 deg NE to NW at 0.7-3.5 cells/cycle in 2-6 of 12 phases when sharp; a 180 deg turn over 8 cycles at 1.4 cells/cycle, 7 of 12). The 20 m-radius lawnmower and the circles above: 0.

### Findings
- [ ] (must-fix) Statements claim more than the code: README item 5 says station keeping does not re-inflate the whole window and lists the sharp diagonal reversal as the only other case; `StationKeepingNeverRepaintsTheWholeWindow` passes because its jitter (whole metres around x.5) keeps 2 origins per axis; the PR statements say "on the local costmap during violent station keeping" only. Measured: global +-2 m station keeping 14 of 150 whole (317 / 348 ms worst), local +-1 m 24 of 300; tight turns from oblique travel and zig-zags too (Gemini's NE-to-NW turn reproduced in the model). Say the whole window is reported whenever a second edge's cells have waited 2 updates, which happens on jitter of 2 cells or more per update, sharp turns or reversals of travel on both axes; rename or recomment the station-keeping test (jitter within one cell); qualify "the next when that travel turns back" as a single reversal (2-cell zig-zags can wait 2; subagent). PR text: the "after a jump of up to a tile at least 2400 / 125 m" figure is for a single jump. — `s57_layer/README.md:38`, `s57_layer/test/test_rolling_window.cpp:1099-1116`, `progress.md` Implementation statements
- [ ] (suggestion) `ReversalPastTheStartEveryCellWithinOneCycle` passes with allowance 0 (checked): its first step back is 1 m from x.5, no move, so it never exercises the one-cycle reversal. Restore 0, or turn back with a 2 m step and keep 1 (Gemini). — `s57_layer/test/test_rolling_window.cpp:846-855`
- [ ] (suggestion) The jump threshold is unpinned at its edge: doubling it (`2*tile_size_`) passes all 26 tests (the test jumps 10 m over 4 m tiles). Add a jump of a tile plus a cell (5 m, whole) and of exactly a tile (4 m, strips). — `s57_layer/src/s57_layer.cpp:448-451`
- [ ] (suggestion) `age++` runs on empty spans too; signed overflow after 2^31 cycles (13.6 years at 5 Hz, UB). Increment only valid spans; all 26 tests pass with that change (Gemini, Codex). — `s57_layer/src/s57_layer.cpp:460`
- [ ] (suggestion) Comments: the `whole_pending_` comment omits the disabled case; the `!enabled_` line is not unreachable (it runs every cycle on a layer configured disabled), its effect is unobservable because nothing re-enables it; the jump rule never fires before the no-overlap test on a window narrower than a tile plus a cell (a 60-cell local costmap with default 100-cell tiles), which the comment could say. — `s57_layer/src/s57_layer.h:219-221`, `s57_layer/src/s57_layer.cpp:198-206, 439-451`

### Checked and found sound
- **Tests, failing without their fix** (test file compiled with the layer source, `ldd`: no `libs57_layer.so`): on `3231db9` Reversal, StationKeeping, SharpDiagonal (" 9" / " 9 10") and JumpShorter fail, FastZigZags passes (a guard, as recorded); jump rule removed: JumpShorter ("300 of 400 ... (33.5, 20.5)"); `last_window_` only on strip cycles: FastZigZags ("cycle 18 ... allowance 3 ... 3 cycles"); always a gap: 7 tests; escape removed: FastZigZags, SharpDiagonal; tile rule removed and no gap: the 4 whole-window tests; mine: index order (FastZigZags, both SlowDiagonals), escape at age 3 (FastZigZags, SharpDiagonal). Caught by nothing: the jump threshold doubled (above), clip order (equivalent). Final: 26 of 26.
- **Origins:** every `drive` and `wholeWindowCycles` path asserts per cycle that Rolling's `size` and Jazzy's `size - 0.5` give the same origin and that the window has it, failing with both origins in the message. Re-derived: every path is at x.5 (the +-1 m jitter, the 2 m steps back, the zig-zags, the 10 m jumps), where `int(k + 0.5)` and `int(k + 0.75)` agree for every integer k of either sign.
- **SharpDiagonal pin:** right to pin the one known cost (a change detector, said so in its comment); Jazzy gives the same cycles (origins asserted equal; `InflationLayer::updateBounds` and `getBounds` identical on `origin/jazzy`).
- **Jump rule:** `abs` of an origin difference, exact multiples of the resolution, threshold tile + half a cell: independent of sign and of window size; a jump reports the whole window and paints every generated tile that cycle. BEN travel (5 and 2 cells/cycle against 100) never triggers it; a few-metre glitch is strips; a 60 m glitch on the local (50 m tiles) is a whole window, cheap there. Fine.
- **`db8c5c2`:** harmless and correct (see the comment suggestion).
- **Jazzy:** nothing new called from Nav2. The eight code commits cherry-pick onto `origin/jazzy` in order without conflict, and the result's `s57_layer` is identical to the branch's. No `.agent/` file in any code commit; each ends "Backport to jazzy.".
- **Round-1 items:** all five closed (reversal spikes gone on the global, 0 of 150 at +-1 m; statements corrected except as above; jump rule; defensive lines labelled; disabled spell).
- **PR numbers:** 2485 m / 172 m (half window minus 3 cycles at 5 m/s) and 2400 / 125 m (minus a tile) check; delayed cells read free without `track_unknown_space` (default value FREE_SPACE).
- Not taken up: Codex's stale inflation at the trailing edge (Nav2's rolling behaviour, as in round 1), Codex's `current_` while a strip waits (the owner's term, in the PR statements), Gemini's tide/chart-set flags not clearing the spans (the bound holds: those cycles report every tile except the ones wholly outside last cycle's window, whose cells stay pending in the strips), Gemini's extra end tile (pre-existing).
