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
