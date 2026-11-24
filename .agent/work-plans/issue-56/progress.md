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
