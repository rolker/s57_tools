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
