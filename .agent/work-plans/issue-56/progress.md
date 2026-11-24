---
issue: 56
---

# Issue #56 — s57_layer leaves the west and south edge of a rolling costmap unmarked (worldToTile rounds toward zero)

## Issue Review
**Status**: complete
**When**: 2026-11-24 11:05 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)

**Issue**: #56

### Scope Assessment

**Well-scoped?** Yes, with one addition. The defect and fix are confirmed against the source (below). The one-function fix plus test is the right core. The same truncation, with a different trigger, sits in `updateCosts` lines 469 and 474; the planner should decide whether to fold it in (see Recommendations).
**Right repo?** Yes. `s57_layer` is in `rolker/s57_tools`. Fix on `rolling` first, hand backport to `jazzy` with `git cherry-pick -x`.
**Serves**: none (project issue; the project states no goals). It fixes charted hazards going missing from the costmap, which is the point of `s57_layer`.
**Design picture?** No change. The project has no design document and the fix changes no interface.
**Dependencies**: none to merge. Coordinates with #52 part 2 (the Rolling port): both branches add a test to `s57_layer/CMakeLists.txt` (trivial textual conflict); the fix itself is in `worldToTile`, which the port does not touch.

**Statements in the issue, checked against the source**

- `worldToTile` (`s57_layer.cpp:157-162`): `int ix = (x - x_origin_)/resolution_/tile_size_;` and the same for y. Confirmed. The conversion truncates toward zero, so for negative offsets it returns the ceiling, one too high unless the offset is an exact multiple of a tile.
- Callers: `updateBounds` (start/end tile, ~lines 302-303) and `updateCosts` (lines 463-464). Both loop `start_tile..end_tile`, so a wrong start tile skips the tile holding the lower-left corner. Confirmed. `updateBounds` also skips it, so `generateTile` is never called for that tile and its bounds never reach the update region; the strip is missed in both functions.
- Tile bounds and copy offsets (`updateBounds` 314-317, `generateTile` 341-344, `updateCosts` 469/474) are computed from the tile id as `origin + id*resolution*tile_size` and are correct for negative ids. Only the id lookup is wrong, so "data that is copied lands in the right place" holds.
- `x_origin_`/`y_origin_` are assigned only in `matchSize` (lines 145-153), which `initialize` calls once (line 131) and Nav2 calls on `resizeMap`. A rolling window's moves do not update them. Confirmed.
- Which windows are affected: any window, rolling or static, whose lower-left corner is below the layer origin on an axis. A static window is affected too if its origin is configured negative and differs from the layer's origin, but static costmaps set the layer origin from their own origin, so in practice they are not. The issue's statement stands for deployed configurations.
- Within a tile the cells at and above the tile's lower edge are fine. Only a window corner with a negative offset that is not a multiple of the tile size loses its partial tile: the strip is between 0 and one tile wide (up to `tile_size` cells, default 100, `README.md:77`).
- "Always unmarked where it should be marked": confirmed. Skipped tiles are neither copied nor overwritten, so nothing is wrongly marked. With `allow_uncharted: false` the uncharted branch of `updateCosts` is skipped for the same tile, so the strip is not marked unknown either, as the issue says.
- BEN numbers (`ben_project11/config/nav2_params.yaml`): local costmap 350 x 350 m at 0.5 m, `rolling_window: true`, `track_unknown_space` not set (default false); global costmap 5000 x 5000 m at 1.0 m, `rolling_window: true`, `track_unknown_space: true`. Layer `tile_size` is not set in either, so 100 cells: 50 m wide strip for local (starts 175 m out), 100 m wide for global (starts 2500 m out). Matches the issue. Correction: the costmap `global_frame` is `<tf_prefix>/map_tide`, not `map`; the issue's "`map` is anchored at the first position fix" should name `map_tide`. Whether that frame's origin is the first fix was not verified here and the fix does not depend on it. The legacy `costmap_chart.yaml` (20000 m at 3 m) is not loaded by `nav2_params.yaml`.
- `bathymetry_layer` (`src/bathymetry_layer.cpp:349-355`): `static_cast<int>(std::floor((x - x_origin_) / tile_meters))`, and for the master-origin offset in `updateCosts` (1190-1215) `static_cast<int>(std::lround((master_grid.getOriginX() - x_origin_) * inv_res))`, with a comment that lround absorbs sub-cell TF jitter. So it does both conversions correctly.

**Integer conversions of the same kind**

| Location | What | Affected? |
|---|---|---|
| `s57_layer.cpp:159-160` `worldToTile` | world to tile index, truncates | Yes, the defect. Called from `updateBounds` 302-303 and `updateCosts` 463-464 |
| `s57_layer.cpp:469` `tile_offset_x` | `-ti*tile_size_ + (master_grid.getOriginX()-x_origin_)/resolution_` converts a double to int by truncation | Latent, different trigger. The quotient is an integer number of cells by construction, but floating-point error can leave it at `k-1e-12`, which truncates to `k-1` for positive and `k+1` for negative values (the whole tile content shifts one cell). Exact for BEN (0.5 and 1.0 are binary-exact) and not guaranteed for other resolutions such as 0.1 or 0.3. `bathymetry_layer` uses `lround` here |
| `s57_layer.cpp:474` `tile_offset_y` | same, y axis | Same as above |
| `s57_layer.cpp:314-317`, `341-344` tile id to world | `origin + id*resolution*tile_size`, double arithmetic | Not affected, correct for negative ids |
| `s57_layer.cpp:423-431` `generateTile` `worldToMapEnforceBounds` | `Costmap2D` clamps negative offsets to cell 0; coordinates are relative to the tile origin so never meaningfully negative | Not affected (a half-resolution overshoot clamps) |
| `s57_layer.cpp:459-460` `updateCosts` `mapToWorld` | cell centres, no int conversion | Not affected |
| `s57_layer.cpp:278-279` `worldToLatLon` request to `s57_grids` | double arithmetic, no integer index | Not affected. The request region is `world_min - 2*buffer` .. `world_max + 2*buffer`, unaffected by the tile bug |
| `s57_grids` sources | `grep` found no integer conversion of a coordinate to an index (it uses grid_map's own indexing); grid_publisher.cpp:436 is only a log line | Not affected |

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Only what's needed | OK | One function and a test is the minimum. The `lround` for the offsets is optional and is a planner decision, see Recommendations |
| A change includes its consequences | Watch | The test must be wired into `s57_layer/CMakeLists.txt`, which the port (#52 part 2) also edits; plan for a trivial merge conflict |
| Test what you change | Action needed | The test needs a rolling window whose lower-left corner is below the layer origin; the existing fixtures cannot show the defect (every `updateCosts` test uses a non-rolling 10 x 10 costmap at origin 0, `test_allow_uncharted.cpp:50-52, 86-87`) |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| None | No | A bug fix in a project package; no workspace ADR is triggered |

### Consequences

- Add a gtest to `s57_layer/CMakeLists.txt` next to the existing ones; the port's branch adds another there.
- Backport: one `git cherry-pick -x` of the fix commit onto `jazzy`; no records file inside the code commit.
- The README of `s57_layer` needs no change unless the `lround` addition changes documented behaviour (it does not).

### Verification

- `s57_layer` does not build on Rolling until #52 part 2 merges (CMake, tf2 headers, Nav2 API changes); no Rolling CI job builds Nav2 source (the CI `rolling` job covers four of six packages). So on this branch the test cannot run locally on this machine.
- Jazzy CI builds and tests `s57_layer` against released Nav2 on pull requests into `rolling` and into `jazzy`. That is the real run of the new test: it should fail without the fix and pass with it (the issue's scratch test shows 25/100 cells lethal without it, 100/100 with it).
- Local evidence on Rolling: take a scratch copy carrying the port's changes plus the fix and test, as the port's plan reviewer did, and run the test there. A single-line fix is cheap to carry.
- Combining: the fix lands on `rolling` through this branch, the port through its own. The two touch different lines of `s57_layer.cpp`; the only expected conflict is the test registration in `CMakeLists.txt`. Whichever merges second rebases; the first merge does not need the second.

### Recommendations

- Test: a rolling-style costmap whose lower-left corner is below the layer origin on x only, on y only, and on both (plus one control with the corner at an exact tile multiple, which already passes), over an all-land chart grid, asserting every cell of the master costmap is lethal. Build the costmap with `resizeMap(w, h, res, ox, oy)` at the layer's origin and then move it with `updateOrigin` (or `resizeMap` with a lower origin after layer `matchSize`), and give the layer a chart grid by the same injection the existing tests use or by its own helper.
- Fix: `std::floor` in `worldToTile`, with the cast as `static_cast<int>(std::floor(...))`, as `bathymetry_layer` has it.
- Planner to decide, no owner needed: also change `updateCosts` lines 469 and 474 to `static_cast<int>(std::lround(...))`, as `bathymetry_layer` does. The planner should say which, and keep it a separate commit if it is added, so each cherry-pick stays one fix. If it is added, pair it with a test at a binary-inexact resolution (for example 0.1 m); if no such test can be made to fail, leave it out.
- Correct the issue's BEN note: the global frame is `map_tide`.
- Record in the plan that verification on Rolling depends on #52 part 2 and Jazzy CI is the first real run.

### Actions
- [ ] The test needs a rolling window whose lower-left corner is below the layer origin; the existing fixtures cannot show the defect (every `updateCosts` test uses a non-rolling 10 x 10 costmap at origin 0, `test_allow_uncharted.cpp:50-52, 86-87`)
- [ ] Test: a rolling-style costmap whose lower-left corner is below the layer origin on x only, on y only, and on both (plus one control with the corner at an exact tile multiple, which already passes), over an all-land chart grid, asserting every cell of the master costmap is lethal. Build the costmap with `resizeMap(w, h, res, ox, oy)` at the layer's origin and then move it with `updateOrigin` (or `resizeMap` with a lower origin after layer `matchSize`), and give the layer a chart grid by the same injection the existing tests use or by its own helper.
- [ ] Fix: `std::floor` in `worldToTile`, with the cast as `static_cast<int>(std::floor(...))`, as `bathymetry_layer` has it.
- [ ] Planner to decide, no owner needed: also change `updateCosts` lines 469 and 474 to `static_cast<int>(std::lround(...))`, as `bathymetry_layer` does. If it is added, pair it with a test at a binary-inexact resolution (for example 0.1 m) and keep it a separate commit; if no such test can be made to fail, leave it out.
- [ ] Correct the issue's BEN note: the global frame is `map_tide`.
- [ ] Record in the plan that verification on Rolling depends on #52 part 2 and Jazzy CI is the first real run.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 11:01 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the issue review confirmed the defect and the scope; its points are for the planner and none changes what the work is for)
**After**: issue-actions
**Decision**: proceed

Read: the Issue Review, status complete. Scope stands: `std::floor` in `S57Layer::worldToTile` (`s57_layer/src/s57_layer.cpp:159-160`) and a test that fails without it: a rolling costmap whose lower-left corner is below the origin on x only, on y only and on both, with a control at an exact tile multiple, over an all-land grid, asserting every cell is lethal. For the planner: (1) the sibling conversion at `s57_layer.cpp:469` and `:474` (the master-to-layer origin offset truncated to int, where floating-point error at a resolution such as 0.1 m can shift a tile's content by one cell; `bathymetry_layer` uses `lround` there): include it as a **separate** commit only if a test can be made to fail without it; otherwise leave it and list it for rolker/s57_tools#54. (2) `s57_layer` does not build on Rolling until rolker/s57_tools#52 part 2 merges, so local verification is on a scratch copy that carries the port's changes plus this fix and test; the Jazzy CI job on the pull request into `rolling` is the first real run. (3) Both this branch and the port's add a test to `s57_layer/CMakeLists.txt`; whichever merges second rebases. (4) The issue text says `map`; the costmaps' global frame on BEN is `map_tide`, and whether that frame is anchored at the first fix was not verified; the fix does not depend on it, and the pull request says what is and is not known. The owner asked for this fix now, in parallel with the port, with its own pull request into `rolling` and a backport to `jazzy` as soon as it merges.

## Plan Authored
**Status**: complete
**When**: 2026-11-24 11:12 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Plan**: `.agent/work-plans/issue-56/plan.md` at `dde57e4`

Two commits: `std::floor` in `worldToTile` with a rolling-window test (six cases, every cell asserted; an exact tile multiple is not a control because `updateCosts` looks up cell centres), then `lround` for the master-to-layer offset at 469/474 with a 0.1 m test that shows land shown free without it. Each case was shown failing without its fix and passing with it on a Rolling scratch copy carrying the port's changes; Jazzy is CI only. Two further defects found (exposed cells not repainted after a move; empty chart list sets no needs_update) are left to the host.

## Plan Review
**Status**: complete
**When**: 2026-11-24 11:24 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: ready

**Issue**: #56 — s57_layer leaves the west and south edge of a rolling costmap unmarked (worldToTile rounds toward zero)
**Plan**: `.agent/work-plans/issue-56/plan.md` at `dde57e4`
**Branch**: `feature/issue-56`

Independent scratch run: a package of my own in the session scratchpad compiling the port's `s57_layer.cpp` at `02d7617` (feature/issue-52, the port's code commits) three ways (as is, `floor` only, `floor` + `lround`) into a test executable with Rolling Nav2's `LayeredCostmap` and `InflationLayer` (`ldd`: underlay `libnav2_costmap_2d_core.so` and `liblayers.so`; the layer is compiled into the binary, so no installed `libs57_layer.so` can shadow it).

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Two conversions, one test file, three header members, one CMake target; two commits, each one fix. Defects A and B reported, not fixed |
| Issue alignment | Good | Covers the issue and every issue-review action (floor, optional `lround` with a failing test, `map_tide` correction, Rolling verification via scratch, Jazzy CI as first real run). Serves: "None (the project states no goals). It fixes charted hazards missing from the costmap, the purpose of `s57_layer`." Correct for a project without goals |
| File targeting | Good | `s57_layer.cpp`, `s57_layer.h`, new test, `CMakeLists.txt`. No README change needed |
| Consequences | Needs work | Defect A's description understates the serious case (finding 2); defect B's overstates it for BEN (finding 3) |
| Principle alignment | Good | Each fix has a test that fails without it (reproduced independently, below). Prior art (`bathymetry_layer` `floor`/`lround`) taken. Design picture: no change, stated |
| ADR compliance | Good | No workspace ADR triggered (project bug fix) |
| ROS conventions | Good | Test drives Nav2's own `LayeredCostmap::updateMap`; `__has_include` switch as the port's tests have it |

### Independent checks

- **#56 cases, through `updateMap`** (10 x 10 at 1 m, tile 4, all land): as is, robot (0,0) 19 wrong, (0,20) 10, (20,0) 10, (-3,-3) origin (-8,-8) 64, (20,20) 0; also (-3.5,-3.5) 64 and (-1000.25,-2000.75) 19. With `floor`, 0 in every case. The planner's correction stands: an exact tile multiple is not a control.
- **Class of defect.** `worldToTile` has two callers (`updateBounds` with the window edges, `updateCosts` with cell centres of `min_i` and the exclusive `max_i`); every other tile-id use is `origin + id*res*tile`, correct for negative ids. The copy offsets with `start_i`/`i_count` handle a negative tile correctly. Edge rounding: by reading, if floating-point error puts the window's right or top edge just below a tile boundary, `updateCosts`' end cell centre lands one tile above `updateBounds`' end tile; `tiles_[tile]` then default-constructs that tile (`complete = false`), which holds `current_` false. This is the same with or without `floor`, needs an edge within an ulp of a boundary, and was not run; outside #56.
- **`lround`.** Reproduced: 10 x 10 at 0.1 m, one `updateMap(0.85, 0.5)` gives origin `0.30000000000000004`; against an unmoved 20 x 20 oracle, `floor` only gives 10 of 100 wrong, first (1,0) cost 0 where 254 is expected; with `lround`, 0. The offset `(master_origin - x_origin_)/res` is an integer number of cells by construction (`updateOrigin` keeps the grid aligned and `matchSize` re-takes the origin on every resize), so it is never legitimately half-way; `lround` is right for both signs, and equals truncation whenever truncation was right. Same as `bathymetry_layer.cpp:1190-1215` (which multiplies by `1/res`; immaterial). BEN's 0.5 and 1.0 m keep every origin an exact multiple, so nothing changes on BEN. The test's double is IEEE-deterministic (`0.85-0.5` is exact, `/0.1` gives `3.4999999999999996`, `0 + 3*0.1` is the same with or without FMA contraction), and the origin assertion stops it passing vacuously.
- **Nav2 Jazzy API.** `origin/jazzy` `LayeredCostmap(std::string, bool, bool)`, `addPlugin(std::shared_ptr<Layer>)`, `updateMap(x, y, yaw)` are the same as `main`; `Layer::initialize` takes `nav2_util::LifecycleNode::WeakPtr` there, which the switch provides. `updateOrigin` arithmetic is identical.
- **`map_tide`.** `mru_transform.cpp:165-181` anchors `map` at the first fix unless an origin is configured; `sea_surface_estimator.hpp:367-372` publishes `map_tide` as a z-only translation of the odometry header frame. As the plan states; the plan's list of what stays unestablished is right.

### Findings

1. **[Principle alignment — Test what breaks] Low.** The uncharted case's premise is too narrow. "A real `updateMap` cycle marks nothing" holds only for a costmap without an inflation layer. `Costmap2DROS::on_configure` calls `setRobotFootprint` after the plugins initialise, `InflationLayer::onFootprintChanged` sets `need_reinflation_`, and the first `updateMap` then passes the whole window to every layer's `updateCosts`. Scratch: allow_uncharted false, no charts, `LayeredCostmap` + `InflationLayer` + `setFootprint`, one `updateMap`: 100 of 100 `NO_INFORMATION` (0 of 100 without the inflation layer). So the direct `updateBounds`/`updateCosts` call asserts what production does on its first cycle on both BEN costmaps; it is not pinning a behaviour production never shows. Resolution: drive that case through `updateMap` with an `InflationLayer` and `setFootprint` (Nav2's production path), or say in the test's comment which production cycle the direct call stands for.
2. **[Consequences — defect A] High, for the owner; not blocking this plan.** Defect A is real, and its serious form is not the one the plan describes. Mechanism: `updateOrigin` fills newly exposed cells with the default; `S57Layer::updateBounds` reports only tiles with `needs_update`, and a tile already in `tiles_` never gets it again except through tide (> 1 cm), chart-set change or `reset()`. Stock rolling layers report the whole window every cycle (`StaticLayer::updateBounds` rolling branch; `bathymetry_layer.cpp:896-901` likewise). The plan says the inflation layer "only re-expands bounds the layer already reported"; it also merges the previous cycle's bounds and adds its radius. Scratch, all-land chart, tile 4 m:
   - No inflation: moving east 1 m per step, unpainted columns accumulate (60 of 100 after 8 m).
   - Inflation radius 2 m (< tile): a column stays unpainted permanently.
   - Inflation radius 6 m (≥ tile, like BEN: 150 m against 50 m local and 100 m global tiles): the band at the leading edge grows up to one tile less one cell, then is repainted when the window enters a new tile column.
   - **Reversal with inflation 6 m and a footprint set:** 10 m east, then back west: the retraced window reaches 100 of 100 cells free on an all-land chart, and stays so until the window enters a tile column never seen before.
   On BEN this means: travelling straight, an unpainted band up to 50 m deep at 125-175 m ahead in the local costmap (and up to 100 m at 2400-2500 m in the global), repainted every tile width of travel. On any return into ground already seen (survey lines longer than half the local window, a reversal), the local costmap shows `FREE_SPACE` on charted land over the re-entered area. The global costmap is affected only where an area left its 5000 m window and came back; it then shows `NO_INFORMATION`, which `allow_unknown: true` plans through. Not established: how often the sea-surface average moves more than 1 cm on BEN (each such move repaints the window, so frequent tide jitter would mask A), and whether BEN's controller consults the local costmap for collision. Resolution: the host reports A to the owner as its own issue, and states that it may be more serious than #56. It does not change this plan's tests (each starts from empty tiles in one cycle).
3. **[Consequences — defect B] Low.** Real as described in a bare `LayeredCostmap`, narrower in production. On a `Costmap2DROS` with an inflation layer, the first cycle paints the whole window (finding 1), and a later empty chart response invalidates every tile (`getDatasetsCallback:721-729` sets `needs_update`), so those tiles are painted unknown. What remains is tiles that first enter the window after that, with `allow_uncharted: false`: they are never painted (the tile mechanism of defect A). With `track_unknown_space: true` their default is already `NO_INFORMATION`, so B shows only with `track_unknown_space: false`. BEN sets no `allow_uncharted` (default true), so B does not apply to BEN at all. Resolution: correct the plan's B description in the record the host passes to the owner.
4. **[Principle alignment — Test what breaks] Low.** The `lround` test fails without the fix only because it paints in the cycle that moves the window. A two-cycle version (paint at origin 0, then move to 0.3) passes without `lround` (0 of 400 wrong in scratch), because defect A means the moved cells are never repainted. Resolution: keep the single-cycle structure and say in the test comment why it must stay single-cycle.
5. **[Consequences — vessel] Info.** The plan's "they become unknown where `allow_uncharted: false` and there is no chart" does not apply on BEN (default true). The rest of the vessel statement stands: newly lethal cells in the west/south strip and their inflation (at cost_scaling 0.05 the inflation cost is about 21 at 50 m and under 1 at 150 m); a mission whose path or goal is on charted land in that strip now fails to plan where it used to plan through, which is the correct outcome; nothing previously marked becomes unmarked.
6. **[File targeting — CMake] Info.** The port inserts `test_initialize` between the `test_depth_costs` and `test_plugin_loading` blocks. The conflict is in that one place, as the plan says; registering the new test after `test_plugin_loading` may avoid even that. The port's `.cpp` hunks (`@@ -7`, `-32`, `-121`, `-652`) do not touch lines 159-160, 469 or 474, and it does not touch `s57_layer.h`.

### Summary

The plan is correct and ready: the `floor` fix is complete for the defect class, the `lround` fix is right for every sign and does nothing on BEN, and both tests fail without their fix (reproduced independently on Rolling). The findings are refinements to one test's framing and, more importantly, to the record of defect A, which is real and has a re-entry form that leaves the local costmap free on charted land.

### Recommended Actions

- [ ] Drive the uncharted case through `updateMap` with an `InflationLayer` and `setFootprint`, or comment which production cycle the direct call stands for (finding 1)
- [ ] Comment in the `lround` test why it must paint in the cycle that moves the window (finding 4)
- [ ] Host: report defect A to the owner as its own issue, with the re-entry case and the unestablished tide-repaint frequency (finding 2)
- [ ] Host: correct defect B's description for the owner: not applicable on BEN; only for tiles entering later with `allow_uncharted: false` and `track_unknown_space: false` (finding 3)
- [ ] Optionally register the new test after `test_plugin_loading` (finding 6)

## Checkpoint
**Status**: complete
**When**: 2026-11-24 11:24 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the plan review's verdict is ready; its findings on this plan are low or informational and are applied as below; the two further defects it confirmed are outside this issue and go to the owner separately)
**After**: plan
**Decision**: proceed

Read: the Plan Review at `dde57e4`, verdict ready. Both commits go in as planned: `std::floor` in `worldToTile` with the rolling-window test, and `lround` on the master-to-layer cell offset with its 0.1 m test. One push, not test-first. Findings applied: (1) the uncharted case is driven through Nav2's own cycle, a `LayeredCostmap` with an `InflationLayer` and `setFootprint` and one `updateMap`, as a costmap node does on its first cycle; if that proves impractical the direct call stays and its comment says which production cycle it stands for. (4) The `lround` test stays single-cycle and its comment says why: painted in a later cycle, the moved cells are not repainted at all (the separate repaint defect), so a two-cycle version passes without the fix. (5) The pull request's vessel statement drops "become unknown where `allow_uncharted` is false" for BEN, which leaves that setting at its default of true. (6) The new test is registered after `test_plugin_loading` in `s57_layer/CMakeLists.txt`, away from where the port inserts its own. **Not in this issue:** the review confirmed that cells exposed by a rolling window's move are not repainted until a tile is invalidated (the planner's defect A; on re-entered ground the local costmap can show free space on charted land), and that with no charts and `allow_uncharted: false` newly entered tiles are not marked (defect B, which does not apply to BEN). Neither is touched here; the tests each start from empty tiles in one cycle and are not affected. The host takes A to the owner as its own issue.

## Implementation
**Status**: complete
**When**: 2026-11-24 11:40 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Branch**: feature/issue-56 at `a5b2294`

Two code commits, neither touching `.agent/`, each ending "Backport to jazzy." (`git merge-tree` shows each cherry-picks cleanly onto `origin/jazzy` in order; `s57_layer` is identical on `rolling` and `jazzy`):

- `ce8fd8d` s57_layer: find a rolling window's tiles below the layer origin (#56). `std::floor` in `worldToTile`; `current_charts_`, `chart_bounds_`, `grids_` moved to protected in `s57_layer.h` with a one-line comment; new `test/test_rolling_window.cpp` (7 cases) and its target, registered after `test_plugin_loading` (links `nav2_costmap_2d::layers` for `InflationLayer`).
- `a5b2294` s57_layer: round the master-to-layer cell offset to the nearest cell (#56). `static_cast<int>(std::lround(...))` at `:469` and `:474`, tile term added as an integer (as `bathymetry_layer.cpp:1190-1215`); the 0.1 m test case.

Final diff of `s57_layer.cpp` against `origin/rolling` (no other line changed):

```diff
@@ -156,8 +156,8 @@ void S57Layer::matchSize()
-  int ix = (x - x_origin_)/resolution_/tile_size_;
-  int iy = (y - y_origin_)/resolution_/tile_size_;
+  int ix = static_cast<int>(std::floor((x - x_origin_)/resolution_/tile_size_));
+  int iy = static_cast<int>(std::floor((y - y_origin_)/resolution_/tile_size_));
@@ -466,12 +466,12 @@ void S57Layer::updateCosts(...)
-    int tile_offset_x = -ti*tile_size_ + (master_grid.getOriginX()-x_origin_)/resolution_;
+    int tile_offset_x = -ti*tile_size_ + static_cast<int>(std::lround((master_grid.getOriginX()-x_origin_)/resolution_));
-      int tile_offset_y = -tj*tile_size_ + (master_grid.getOriginY()-y_origin_)/resolution_;
+      int tile_offset_y = -tj*tile_size_ + static_cast<int>(std::lround((master_grid.getOriginY()-y_origin_)/resolution_));
```

### Tests

Rolling `LayeredCostmap("map", true, false)`, 10 x 10 at 1 m, `tile_size` 4, in a costmap node's order (`resizeMap`, `addPlugin`, `initialize`), one `updateMap(robot)`; all-land chart given through the protected members; every cell asserted, each case also asserts the window origin it relies on. The uncharted case (`allow_uncharted: false`, no chart) runs through Nav2's cycle with an `InflationLayer` after the layer and `setFootprint`, as a costmap node's first cycle; its comment says the inflation layer is needed because with no chart the layer reports no bounds of its own (defect B). The `lround` case is single-cycle against an unmoved 20 x 20 oracle, asserts origin `0.30000000000000004`, and its comment says why it must stay single-cycle (defect A). No sleeps; no DDS traffic beyond the node's own participant.

### Verification (Rolling, scratch copy)

Scratch copy in the session scratchpad: `s57_layer` and `s57_grids` from the port's branch `feature/issue-52` at `8ab87bf` (by `git archive`, read-only) with this branch's `s57_layer` diff applied; built with colcon against `/opt/ros/rolling`, the underlay install (Nav2 from source) and the core install. Final run on the exact `git merge-tree` result of this branch and `8ab87bf` (tree `4a53c3e`, which merges with no conflict).

| Case | Without `floor` | With `floor` |
|---|---|---|
| both axes, robot (0,0) | 19 of 100 not 254; origin (-5, -5); first (0, 0) at (-4.5, -4.5) cost 0 | OK |
| x only, (0,20) | 10 of 100; origin (-5, 15); first (0, 0) at (-4.5, 15.5) cost 0 | OK |
| y only, (20,0) | 10 of 100; origin (15, -5); first (0, 0) at (15.5, -4.5) cost 0 | OK |
| exact tile multiple, (-3,-3) | 64 of 100; origin (-8, -8); first (0, 0) at (-7.5, -7.5) cost 0 | OK |
| far negative, (-1000.25,-2000.75) | 19 of 100; origin (-1005, -2005); first (0, 0) at (-1004.5, -2004.5) cost 0 | OK |
| positive space control, (20,20) | OK | OK |
| uncharted, (0,0) | 19 of 100 not 255; origin (-5, -5); first (0, 0) at (-4.5, -4.5) cost 0 | OK |

`lround` case, `floor` only: "10 of 100 cells differ from the unmoved window; window origin (0.30000000000000004, 0); first wrong cell (1, 0) centred at (0.45, 0.05) has cost 0 where the unmoved window has 254" (land shown free). With `lround`: OK.

- Each of the 8 new tests 30 times as separate processes (`--gtest_filter`): 30 pass, 0 fail each.
- `colcon test --packages-select s57_layer` (`--ctest-args -j1`): "Summary: 60 tests, 0 errors, 0 failures, 8 skipped" (the 8 skipped are cppcheck's per-file skips); gtest: allow_uncharted 3, tide_offset 7, depth_costs 12, initialize 9, plugin_loading 1, rolling_window 8, 0 failures.
- `ldd` under the same environment: every `s57_layer` test binary (`test_allow_uncharted`, `test_tide_offset`, `test_depth_costs`, `test_initialize`, `test_rolling_window`) resolves `libs57_layer.so => <scratch>/ws/install/s57_layer/lib/libs57_layer.so`; `test_rolling_window` loads `liblayers.so` and `libnav2_costmap_2d_core.so` from `rolling/underlay/install/nav2_costmap_2d/lib`. No other `libs57_layer.so` exists under `/opt/ros` or `rolling/`.

### Jazzy compile, by reading (`navigation2` `origin/jazzy`, c7046550)

`LayeredCostmap(std::string, bool, bool)`, `resizeMap(uint, uint, double, double, double, bool = false)`, `addPlugin(std::shared_ptr<Layer>)`, `updateMap(double, double, double)`, `setFootprint(const std::vector<Point> &)`, `getCostmap()`; `Layer::initialize(LayeredCostmap *, std::string, tf2_ros::Buffer *, const nav2_util::LifecycleNode::WeakPtr &, CallbackGroup::SharedPtr)` (the `__has_include` switch gives `nav2_util::LifecycleNode`); `nav2_costmap_2d/inflation_layer.hpp` with `InflationLayer()`; `Costmap2D::getCost(uint, uint)`, `mapToWorld(uint, uint, double &, double &)`, `worldToMap(double, double, uint &, uint &)`, `getOriginX/Y`, `getSizeInCellsX/Y`; CMake target `nav2_costmap_2d::layers` (alias at `CMakeLists.txt:94`). All present with these signatures. `Costmap2D::updateOrigin` and `LayeredCostmap::updateMap`'s rolling and bounds logic match `main`; `InflationLayer::onFootprintChanged` sets `need_reinflation_` and `updateBounds` then reports the whole map, as on `main`.

### Statements for the pull request

- **Shown on Rolling:** on a scratch copy carrying the port's changes (`feature/issue-52` at `8ab87bf`), built against the underlay's Nav2 install: each case above failing without its fix and passing with it, all `s57_layer` tests passing, 30/30 repeats.
- **Jazzy:** Jazzy CI on this pull request is the first Jazzy compile and run; nothing was run on Jazzy locally. The Nav2 calls were checked against `origin/jazzy` by reading.
- **For a vessel:** cells in the west and south edge strip of a rolling window that were free on the local costmap, or unknown on the global one, become lethal where the chart says so, with their inflation. Nothing that was marked becomes unmarked. A path or goal on charted land in that strip now fails to plan, which is correct. BEN leaves `allow_uncharted` at its default (true), so the uncharted case does not apply there.
- **Not established:** that the odometry frame on the boat is `map`; whether the map is re-anchored in operation; anything on a vessel or with a planner.
- **`lround`:** changes nothing at BEN's 0.5 m and 1.0 m resolutions (binary-exact).
- **Combining with the port:** `git merge-tree` of this branch and `8ab87bf` merges with no conflict (the new test is registered after `test_plugin_loading`; the port inserts `test_initialize` before it; the `.cpp` hunks do not overlap; the port does not touch `s57_layer.h`). The merged tree builds and passes all `s57_layer` tests on Rolling (above). The new test already uses the port's `__has_include` switch. Whichever merges second rebases; the port's plan's "Found defect 2" becomes "fixed by #56".

### Not done here

- Defect A (cells exposed by a window move not repainted until a tile is invalidated) and defect B (no charts, `allow_uncharted: false`: newly entered tiles not marked) are not fixed or worked round. The uncharted test depends on B only in that it needs the inflation layer's whole-window bounds (said in its comment); the `lround` test would pass vacuously in a two-cycle form because of A (said in its comment). The other tests start from empty tiles in one cycle and are unaffected.
- No push, no GitHub post. Scratch copy removed.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-24 12:02 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: changes-requested

**Branch**: feature/issue-56 at `23aa7d5` (code head `a5b2294`)
**Base**: origin/rolling
**Depth**: Standard (reason: about 350 changed lines of C++ in a costmap layer deployed on vehicles; a new test whose only automated run is Jazzy CI)
**Must-fix**: 1 | **Suggestions**: 2
**Round**: 1 | **Ship**: continue (round 1: 1 must-fix; first round always re-reviews after fixes)

Reviewers: Claude adversarial (fresh subagent) completed; Gemini and Codex (cross_model_review.sh, branch origin/rolling) completed; their prompt and findings files were removed from the tree.

### Findings
- [x] (must-fix) Five of the eight new tests fail on Jazzy. They assert window origins that only Rolling's Nav2 produces. Jazzy's `Costmap2D::getSizeInMetersX` returns `(size_x_ - 1 + 0.5) * resolution_` (navigation2 `origin/jazzy` `costmap_2d.cpp:526`). Rolling's returns `size_x_ * resolution_`, after Nav2 #6028 (`86416664`, first released in 1.5.0, not on `origin/jazzy`). So on Jazzy, `updateMap` puts the 10 x 10 window at robot - 4.75, and `updateOrigin` truncates that toward zero. The origins Jazzy gives are: robot (0,0) gives (-4,-4), not (-5,-5); (0,20) gives (-4,15); (20,0) gives (15,-4); and (-3,-3) gives (-7,-7), which is no longer on a tile multiple. That makes BelowOriginOnBothAxes, BelowOriginOnXOnly, BelowOriginOnYOnly, BelowOriginOnAnExactTileMultiple and UnchartedBelowOriginMarkedUnknown fail. FarBelowOrigin, the control and the 0.1 m case still give the asserted origins. Fix: choose robot positions that give the same origin under both formulas (for example a robot at -0.5 gives -5 on both, -3.5 gives -8, and 19.5 gives 14), or compute the expected origin. Keep the origin assertions. The implementation entry's "Jazzy compile, by reading" missed this function. — `s57_layer/test/test_rolling_window.cpp:189,194,199,207,249-250`
- [x] (suggestion) The 0.1 m case varies the chart only along x, with the window's y origin at 0. So the `lround` on `tile_offset_y` and a negative non-binary origin are never exercised by the committed test. The reviewer's scratch sweep showed both are right, but no committed test does. Mirror the coast in y, or add a negative-origin variant. — `s57_layer/test/test_rolling_window.cpp:262-318`
- [x] (suggestion) The pull request statements need two changes:
  - "Jazzy: the Nav2 calls were checked against `origin/jazzy` by reading" should not imply the window arithmetic matches. It does not (see the must-fix).
  - The vessel statement should add that the global costmap's strip also changes from unknown to its charted cost where the chart shows water, not only where it shows land. Under the 0.1 m resolution case, the `lround` commit also moves cells that were wrongly lethal, one cell off a coast, back to their charted cost (none at 0.5 or 1.0 m).
  — `.agent/work-plans/issue-56/progress.md` (Implementation, "Statements for the pull request")

### Verified (Rolling scratch copy)

The scratch copy is this branch merged with the port's head `28924ab` (`git merge-tree`, tree `043df53`, no conflict), built against `/opt/ros/rolling` and the underlay and core installs. `ldd` showed `libs57_layer.so` resolving to the scratch install.

**Source changes.**
- Only lines 159-160 and 469/474 of `s57_layer.cpp` differ. With both fixes reverted, the file is byte-identical to the port's.
- The tests fail without their fix. Reverting `floor` fails the 6 floor cases with the messages in the Implementation entry. Reverting `lround` fails only the 0.1 m case ("10 of 100 cells differ…").

**Reviewer's sweep.** One `updateMap` per case over 48 windows with tile size 100: 300 x 300 cells at 0.5, 1.0 and 0.1 m, and 700 x 700 at 0.5 m.
- **Windows.** One in each quadrant, straddling each axis and both, at fractional positions, on an exact tile multiple, and at up to 1e6 m.
- **Charts.**
  - An all-land chart, with every cell checked.
  - A 13 m by 17 m checkerboard. Each cell is compared with the chart's own value at the chart cell that the layer's first-writer rule assigns it. That reference is independent of the tile code.
  - An unmoved non-rolling oracle.
- **Results.**

  | Fixes | Not lethal on all-land | Wrong against the reference | Notes |
  |---|---|---|---|
  | Both | 0 of 5,520,000 | 0 | |
  | `floor` only | 300 | 430 | All in one 0.1 m case |
  | `lround` only | 1,298,735 | 647,048 | |
  | Neither | 1,298,735 | 647,308 | Only the 3 positive-quadrant windows were fully painted |

- **Nothing unmarked by `floor`.** Comparing outputs per cell, `floor` unmarks no cell that was lethal before. `lround` unmarks 130 cells, all in one 0.1 m case. Those were lethal one cell off the coast, and the reference shows the new value is correct.
- An ASan build of the whole sweep with both fixes reported no memory error.

### Not established / for the owner (not findings on this PR)
- **Pre-existing, in `generateTile`.** A chart contributes nothing to a tile whose centre lies outside the chart's extent. `grid_map::SubmapGeometry` fails when the requested centre is off the map. The tile's cells covered by that chart stay unpainted unless another chart covers them. This was seen in scratch with a chart ending inside a tile. It is not affected by this change, and how often it matters with real chart sets is not established.
- **Scratch harness crashes.** The release-build harness crashed occasionally (1 of 40 single-case runs, and in long multi-layer runs), with every variant including unchanged code. The ASan build was clean. The committed `test_rolling_window` passed 100 of 100 runs in release. This is not attributed to the change.

## Implementation
**Status**: complete
**When**: 2026-11-24 12:24 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**Branch**: feature/issue-56 at `66b0ab8`
**Addressed**: Local Review (Pre-Push) at `23aa7d5` (2026-11-24 12:02 -05:00)
**Commits**: `285a116`, `229c4c4` (code, no `.agent/` change, each ending "Backport to jazzy."); `66b0ab8` (boxes ticked)

### Actions
- [x] (must-fix) Five tests asserted Rolling-only window origins: robot positions moved so both distros give the same origin, origin assertions kept, comment in the test names both size formulas and Nav2 #6028 — `s57_layer/test/test_rolling_window.cpp` (`285a116`)
- [x] (suggestion) y and negative-origin coverage for the rounded offset: new case `NegativeNonBinaryOffsetRoundsToNearestCellOnBothAxes`, single-cycle, origin asserted — `s57_layer/test/test_rolling_window.cpp` (`229c4c4`)
- [x] (suggestion) Pull request statements corrected — below

No line of `s57_layer.cpp` or `s57_layer.h` changed. The four code commits cherry-pick onto `origin/jazzy` in order with no conflict (`git merge-tree` chain), and the result's `s57_layer` is identical to this branch's.

### Window origins under both formulas (arithmetic, Python replay)

`LayeredCostmap::updateMap` (rolling): `new = robot - getSizeInMetersX()/2`; `Costmap2D::updateOrigin`: `cell = static_cast<int>((new - origin)/res)` (truncation toward zero, so -5.25 and -5.5 both give -5), `origin = origin + cell*res`. Starting origin 0, 10 x 10 at 1 m: Rolling asks robot - 5 (`size*res`), Jazzy robot - 4.75 (`(size-1+0.5)*res`).

| Case | Robot | Jazzy asks / origin | Rolling asks / origin |
|---|---|---|---|
| both axes | (-0.5, -0.5) | -5.25 / (-5, -5) | -5.5 / (-5, -5) |
| x only | (-0.5, 19.5) | (-5.25, 14.75) / (-5, 14) | (-5.5, 14.5) / (-5, 14) |
| y only | (19.5, -0.5) | (14.75, -5.25) / (14, -5) | (14.5, -5.5) / (14, -5) |
| exact tile multiple | (-3.5, -3.5) | -8.25 / (-8, -8) | -8.5 / (-8, -8) |
| far negative, fractional | (-1000.25, -2000.75) | (-1005.0, -2005.5) / (-1005, -2005) | (-1005.25, -2005.75) / (-1005, -2005) |
| positive control | (20, 20) | 15.25 / (15, 15) | 15.0 / (15, 15) |
| uncharted | (-0.5, -0.5) | -5.25 / (-5, -5) | -5.5 / (-5, -5) |
| 0.1 m, x (10 x 10) | (0.85, 0.5) | see below / (0.30000000000000004, 0) | see below / (0.30000000000000004, 0) |
| 0.1 m, negative x and y (new) | (0.14, 0.14) | -0.335 (q -3.35) / -0.30000000000000004 | -0.36 (q -3.5999999999999996) / -0.30000000000000004 |
| its oracle, 20 x 20 at 0.1 m | (-0.06, -0.06) | -1.0350000000000001 (q -10.35) / -1.0 | -1.06 (q -10.6) / -1.0 |

0.1 m case, exact doubles. Jazzy: `getSizeInMetersX` = 9.5*0.1 = 0.9500000000000001, asks x 0.85 - 0.47500000000000003 = 0.37499999999999994, /0.1 = 3.749999999999999, cell 3; y asks 0.024999999999999967, /0.1 = 0.24999999999999967, cell 0. Rolling: size 1.0, asks x 0.35 (0.85 - 0.5, exact), /0.1 = 3.4999999999999996, cell 3; y 0.0, cell 0. Both: origin 0 + 3*0.1 = 0.30000000000000004, offset /0.1 = 3.0000000000000004, tile 1's `-4 + 3.0000000000000004` = -0.9999999999999996 truncates to 0 (`lround` gives -1). New case: origin 0 + -3*0.1 = -0.30000000000000004, offset -3.0000000000000004, tile -1's `4 - 3.0000000000000004` = 0.9999999999999996 truncates to 0 (`lround` 1); tiles 0 and 1 give -3 and -7 either way. Oracle: -1.0 / 0.1 = -10.0 exactly.

**Jazzy emulated, two ways.** (1) Arithmetic replay in Python of `updateMap`'s origin and `updateOrigin`'s truncation for both formulas (the table). (2) A scratch-only `jazzy_size.cpp` defining `Costmap2D::getSizeInMetersX/Y` with Jazzy's bodies, linked into scratch copies of the test executable (not into any install); `nm -D` shows the executable exports both, so the dynamic linker binds the calls from `libnav2_costmap_2d_core`'s `updateMap` and from `libs57_layer`'s `updateBounds` to them. Proof it takes effect: the previous test file (`a5b2294`) under it fails exactly as the review said: "window origin is (-4, -4), not (-5, -5)", "(-4, 15), not (-5, 15)", "(15, -4), not (15, -5)", "(-7, -7), not (-8, -8)", and the uncharted case's `ASSERT_EQ` on -5; 5 failed of 8. The same file without the shim passes 8 of 8. The new file passes 9 of 9 with and without the shim, at `285a116` (8 of 8) and at `229c4c4`.

### Nav2 behaviour the test depends on, `origin/jazzy` (c7046550) against the underlay's Rolling (9113449e)

- `Costmap2D::getSizeInMetersX/Y`: **differs** (`(size-1+0.5)*res` against `size*res`); handled by the positions above. Also used by `S57Layer::updateBounds` for the window's far edge (origin + 9.5 m on Jazzy, + 10 m on Rolling); either way the end tile is the one holding cell 9 and the clamped bounds map to `xn = 10` in `updateMap`, so the whole window is updated (the shim run confirms).
- `Costmap2D::updateOrigin`: same (body identical, truncating `static_cast<int>`, so toward zero for negatives).
- `Costmap2D::resizeMap`, `mapToWorld`, `worldToMap`, `worldToMapEnforceBounds`, `resetMap`: same.
- `LayeredCostmap::resizeMap`: same (`matchSize` on each plugin). `addPlugin`, `setFootprint`: same in behaviour (Rolling uses `atomic<shared_ptr>::store` where available).
- `LayeredCostmap::updateMap`: same order (`updateOrigin` on both maps, out-of-bounds warning, early return with no plugins, `updateBounds` per plugin, bounds to cells with `+1` and clamps, `resetMap(x0, y0, xn, yn)` of the combined map, `updateCosts` per plugin). Differences: Jazzy returns early when `xn < x0 || yn < y0` where Rolling skips the reset and paint but still marks disabled plugins current; Jazzy has no disabled-plugin `setCurrent`. Neither applies (non-empty bounds, every layer enabled).
- `InflationLayer::onFootprintChanged`: same in effect (`need_reinflation_ = true`; Jazzy takes the inscribed radius from the costmap, Rolling recomputes it; Jazzy logs an error where Rolling warns when the radius is below it). `updateBounds`: identical (whole map when `need_reinflation_`). `updateCosts`: on Jazzy, with no lethal cell (and `inflate_around_unknown` false) nothing is enqueued, so an all-unknown window stays unknown; as on Rolling. `onInitialize`: Jazzy declares parameters through `declareParameter` and registers a parameter callback; same defaults.
- `Layer::initialize`: same behaviour (stores parent, name, tf, node, then `onInitialize`); takes `tf2_ros::Buffer *` and `nav2_util::LifecycleNode::WeakPtr`, which the test's `__has_include` switch provides.
- `CostmapLayer::clearArea` differs (Jazzy's exclusive bounds) but neither the test nor `S57Layer` calls it.

### y and negative-origin coverage

`229c4c4` adds `NegativeNonBinaryOffsetRoundsToNearestCellOnBothAxes`: 10 x 10 at 0.1 m, robot (0.14, 0.14), origin -0.30000000000000004 on both axes (asserted), coast `x < -0.1 || y < -0.1`, one cycle. The comparison moved to a helper, `matchesOracle`, shared with the 0.1 m x case. The oracle is now a 20 x 20 rolling window with the **same layer origin** (so the same tiles), moved once to an origin a whole number of cells from it in floating point (0 for the x case, as before; -1.0 here); the helper asserts that origin and that the oracle shows both land and water under the window. Why the same tiles: a first attempt with a non-rolling oracle at layer origin -1.0 disagreed with the layer even with both fixes, because `generateTile`'s first-writer rule lets a chart cell whose edge lies on a layer cell edge write the neighbouring cell too, and at 0.1 m whether it does depends on floating-point rounding of the tile's own origin (pre-existing, not changed here; at 0.5 and 1.0 m the edges are exact). With a coast at -0.2 that also left no land in the window at all, which the new land-and-water check now catches.

Teeth (scratch, Rolling build and the Jazzy shim build give identical results):

| Variant of `s57_layer.cpp` | new negative case | 0.1 m x case | 1 m cases |
|---|---|---|---|
| both fixes | pass | pass | 7 pass |
| `lround` only (no `floor`) | 19 of 100 differ, first (0, 0) at (-0.25, -0.25) cost 0 where the oracle has 254 | pass | 6 fail, control passes |
| `floor` only (no `lround`) | 17 of 100, first (1, 1) at (-0.15, -0.15) cost 254 where the oracle has 0 | 10 of 100 | 7 pass |
| only the y `lround` reverted | 9 of 100, first (1, 1) cost 254 where 0 | pass | 7 pass |
| only the x `lround` reverted | 9 of 100 | 10 of 100 | 7 pass |
| neither | 19 of 100 | 10 of 100 | 6 fail |

### Verification

Port head read from the shared object store, no fetch: `feature/issue-52` = `origin/feature/issue-52` = `454ae59` (records only since `28924ab`). `git merge-tree --write-tree 229c4c4 454ae59` (the code head; the records commits after it do not touch `s57_layer`) gives tree `faebdd5`, no conflict; archived to a scratch workspace and built (`s57_grids`, `s57_layer`, `--parallel-workers 4`, default build type) against `/opt/ros/rolling`, the underlay install and the core install. A second scratch workspace (Release, with the scratch-only shim targets) ran the variants.

- `colcon test --packages-select s57_layer --parallel-workers 1 --ctest-args -j1`: "Summary: 61 tests, 0 errors, 0 failures, 8 skipped" (cppcheck per-file skips); gtest: allow_uncharted 3, depth_costs 12, initialize 9, plugin_loading 1, rolling_window 9, tide_offset 7, 0 failures.
- Each of the 9 `test_rolling_window` cases 30 times as separate processes (`--gtest_filter`), in three binaries: merged tree default build, Release build, Release build with the Jazzy shim: 810 runs, 810 pass, no crash, no non-zero exit. The reviewer's occasional release-build harness crash was not seen in the committed test binary.
- `ldd`: `test_allow_uncharted`, `test_tide_offset`, `test_depth_costs`, `test_initialize`, `test_rolling_window` each resolve `libs57_layer.so => <scratch>/ws2/install/s57_layer/lib/libs57_layer.so`; `test_rolling_window` loads `liblayers.so` and `libnav2_costmap_2d_core.so` from `rolling/underlay/install/nav2_costmap_2d/lib`. No `libs57_layer.so` exists under `/opt/ros` or `~/project11/rolling`.

### Statements for the pull request (corrected)

- **Jazzy:** nothing was run on Jazzy; Jazzy CI on this pull request is the first Jazzy compile and run. The Nav2 calls the test makes match `origin/jazzy` by signature, but the window arithmetic differs: Jazzy's `Costmap2D::getSizeInMetersX/Y` is `(size - 1 + 0.5) * resolution`, Rolling's `size * resolution` (Nav2 #6028), so a rolling window centres a quarter cell differently. The test places each robot so both formulas give the same window origin, asserts that origin, and was run on Rolling with Jazzy's two functions substituted in the test executable (all pass; the previous positions fail there exactly as predicted). The other Nav2 behaviour it depends on was compared by reading (list above).
- **For a vessel:** cells in the west and south edge strip of a rolling window, previously left at the costmap default (free on BEN's local costmap, unknown on its global one), now take their charted cost: lethal with inflation where the chart shows land or hazards, and the charted water cost where it shows water, so on the global costmap the strip changes from unknown to charted everywhere the chart covers it. Nothing that was marked becomes unmarked by the `floor` change. A path or goal on charted land in that strip now fails to plan, which is correct. BEN leaves `allow_uncharted` at its default (true), so the uncharted case does not apply there.
- **`lround`:** changes nothing at BEN's 0.5 m and 1.0 m resolutions (binary-exact offsets). At resolutions that are not exact in binary it moves a tile's content back by the one cell it was shifted, which both marks cells that were wrongly free and returns cells that were wrongly lethal one cell off a coast to their charted cost (none at 0.5 or 1.0 m).
- **BEN's numbers under Jazzy:** BEN runs Jazzy, so its windows centre at robot - 174.875 m (local, 700 cells at 0.5 m) and robot - 2499.75 m (global, 5000 cells at 1 m) instead of 175 and 2500, then snap to whole cells. The strip is still between 0 and one tile less a cell wide (50 m local, 100 m global) and starts about 175 m / 2500 m from the vessel; its width at a given position shifts by at most one cell. Not material to the earlier statement.
- **Not established:** that the odometry frame on the boat is `map`; whether the map is re-anchored in operation; anything on a vessel or with a planner; Jazzy compile and run (CI).
- **Combining with the port:** merges with `454ae59` with no conflict; the merged tree builds and passes all `s57_layer` tests on Rolling.

### Not done here

- `generateTile`'s first-writer rule at non-binary resolutions (a chart cell can claim a neighbouring layer cell depending on floating-point rounding of the tile origin), seen while building the oracle: pre-existing, not changed, for the owner alongside the review's other `generateTile` note.
- Checks: the `s57_layer` suite above; no workspace script changed, so no script suite applies. No push, no GitHub post. Scratch workspaces removed.
