# Plan: s57_layer leaves the west and south edge of a rolling costmap unmarked (worldToTile rounds toward zero)

## Issue

https://github.com/rolker/s57_tools/issues/56

## Context

`S57Layer::worldToTile` (`s57_layer/src/s57_layer.cpp:159-160`) converts to `int` by truncation, so every negative tile index comes out one too high. The layer's origin (`x_origin_`, `y_origin_`) is set only in `matchSize`, so a rolling window whose lower-left corner is west or south of it loses its west or south partial tile. One correction to the issue and the issue review: **an exact tile multiple is not a control.** `updateCosts` looks up its start tile at the centre of cell 0 (`mapToWorld`), and that centre is never on a tile edge. So any corner below the origin loses a strip 1 to `tile_size` cells wide, and a corner on an exact multiple loses a whole tile. Measured below: 64 of 100 cells are wrong at origin (-8, -8) with tiles of 4.

All of this was shown on a scratch copy: `git archive` of `930e040`, with the port's changes a to g from `issue-52/plan.md` applied by hand. Those are CMake linking, `tf2/utils.hpp`, the `declareParameter` lambda, the free `create_client` and `create_subscription`, and the tests' `__has_include` switch. The port's branch `feature/issue-52` has no code commits yet. The copy was built on Rolling against the underlay and core installs, and `ldd` showed the test loading the scratch `libs57_layer.so`. `s57_layer` 47 tests, 0 failures with both fixes; the new binary 20/20 passes in repeated runs.

## Approach

1. **Fix: `std::floor` in `worldToTile`.** `static_cast<int>(std::floor((x - x_origin_)/resolution_/tile_size_))`, and the same for y, as `bathymetry_layer.cpp:349-355` does. Every consumer was checked, by reading and by the test:
   - **Callers.** `updateBounds:300-301` takes the window edges, `updateCosts:462-463` the cell centres. A window edge exactly on a tile boundary can, through float error, round one tile low. That only generates an extra tile, which the clamp at `:330-333` keeps out of the window.
   - **Tile id to world.** `:314-317`, `:341-344` and `generateTile`'s tile `Costmap2D` and submap all compute `origin + id*res*tile`, which is correct for negative ids.
   - **Copy offsets.** `:469-474`, with `start_i = max(min_i, -off)` and `i_count = min(max_i, tile-off) - start_i`. For tile -2 of origin (-5, -5) with tiles of 4, this copies 1 column from source column 3, which is right.
   - **The request to `s57_grids`.** `:270-279` uses the window edges plus the buffer, with no tile id, so it is unaffected.
   - **`tiles_` map keys.** Negative pairs are fine.
2. **Test: new `s57_layer/test/test_rolling_window.cpp`, registered in `CMakeLists.txt` next to the other gtests.** (Plain `ament_add_gtest` with the existing tests' link lines.)
   - **Fixture.** `S57LayerForTest` adds one chart: a `DatasetInfo` label, its world bounds, and a `grid_map` 100 m by 100 m with `elevation = 1` (land). This needs `current_charts_`, `chart_bounds_` and `grids_` moved from private to protected in `s57_layer.h`, with a comment, following the `tide_offset_` precedent. Charts then need no service, topic, TF or spinning.
   - **Construction.** The node is constructed through the `__has_include("nav2_ros_common/lifecycle_node.hpp")` switch (`TestLifecycleNode`) in the new file itself, so the file compiles on Jazzy now and on Rolling after the port, in either merge order.
   - **Driving the window.** Window 10 x 10 cells at 1 m, `tile_size` 4 (declared on the node before `initialize`, as `test_allow_uncharted.cpp` does): a partial tile at every edge. `LayeredCostmap("map", rolling_window=true, track_unknown=false)` is set up with `resizeMap(10, 10, 1, 0, 0)`, so `matchSize` captures origin (0, 0). Then `addPlugin(layer)` and one `updateMap(robot_x, robot_y, 0)`. That is Nav2's own rolling cycle: `updateOrigin` to `robot - size/2`, the layer's `updateBounds`, `resetMap`, then `updateCosts` over the reported bounds. No costmap node, no sleeps, no DDS traffic beyond the node's own participant: the client finds no server and nothing spins.
   - **Assertion.** Every cell equals the expected cost. On failure it reports the count of wrong cells, the window origin, and the first wrong cell's index, world centre and cost.
   - **Cases, scratch results without the fix → with it:**

     | Case | Robot | Window origin | Expect | Without fix | With fix |
     |---|---|---|---|---|---|
     | both axes | (0, 0) | (-5, -5) | lethal | 19 of 100 wrong, first (0,0) at (-4.5,-4.5) cost 0 | pass |
     | x only | (0, 20) | (-5, 15) | lethal | 10 wrong | pass |
     | y only | (20, 0) | (15, -5) | lethal | 10 wrong | pass |
     | exact tile multiple | (-3, -3) | (-8, -8) | lethal | 64 wrong (whole tile row and column) | pass |
     | positive space (control) | (20, 20) | (15, 15) | lethal | pass | pass |
     | uncharted, `allow_uncharted: false`, no chart | (0, 0) | (-5, -5) | `NO_INFORMATION` | 19 wrong (left `FREE_SPACE`) | pass |

   - **Uncharted case.** This case drives `updateOrigin`, `updateBounds`, then `updateCosts` over the full window, as the existing uncharted tests do. Its correct expectation after the fix: every cell `NO_INFORMATION`. Through `updateMap` it stays 100 of 100 free **with or without** the fix. `generateTile:347-354` returns early for an empty chart list without setting `needs_update`, so the layer reports no bounds and is never asked for those cells. That is defect B below, not #56.
3. **Sibling conversion (`:469`, `:474`): in, as its own commit.** It is `static_cast<int>(std::lround((master_grid.getOriginX()-x_origin_)/resolution_))`, with the tile term added as an integer, as `bathymetry_layer.cpp:1190-1215` does.
   - **What was tried.** First a Python replay of `Costmap2D::updateOrigin`'s arithmetic over random walks at 0.05, 0.1, 0.2 and 0.3 m with origins 0, 0.3, -0.7 and 12.34: thousands of tile offsets truncate wrong.
   - **The failing case.** At 0.1 m, the window moves from 0 to `updateOrigin(0.35)`, so its origin is 0 + 3*0.1 = `0.30000000000000004`. The tile above it then has offset `-0.9999999999999996`, which truncates to 0. That tile's content lands one cell west.
   - **Test** (same file): a coast grid at 0.1 m (land west of 0.6 m and in a band from 0.9 to 1.0 m, 10 m water elsewhere). The oracle is a 20 x 20 window that never moved, compared cell by cell at the same world point. The test asserts the moved origin is that double, so it cannot pass vacuously.
   - **Scratch result.** With `floor` only, 10 of 100 wrong; the first is (1, 0) at (0.45, 0.05), cost 0 where 254 is expected: **land shown free**. With `lround`, pass. This happens at any resolution that is not binary-exact, and BEN's 0.5 and 1.0 are exact.
4. **What changes for a vessel.**
   - **Cells.** Cells in the west and south edge strip that were `FREE_SPACE` (local) or `NO_INFORMATION` (global) become lethal, or get their charted cost, where the chart says so. They become unknown where `allow_uncharted: false` and there is no chart. Nothing that was marked becomes unmarked.
   - **Inflation.** BEN inflates with a 150 m radius, so costs also rise up to 150 m inward of the strip.
   - **Planning.** The global planner has `allow_unknown: true`, so it planned through the unknown strip and now will not.
   - **The `lround` commit.** It changes nothing for BEN (exact resolutions). At 0.1 m and similar, it moves chart content back by one cell.
   - **Strips on BEN** (`nav2_params.yaml`: no `tile_size`, so 100 cells; no `origin_x`/`origin_y`, so the layer origin is `map_tide` (0, 0)):

     | Costmap | Size, resolution | Rolling | `track_unknown_space` | Strip |
     |---|---|---|---|---|
     | local | 350 m, 0.5 m | yes | no | 1 to 100 cells (0.5 to 50 m); 25 m with the boat at the origin; 175 m out |
     | global | 5000 m, 1.0 m | yes | yes | 1 to 100 m; 2500 m out |

   - **The `map_tide` frame.**
     - **Established by reading.** `ben_core_launch.py` runs `mru_transform` with only `base_frame`, `map_frame` and `odom_frame`. No `map_origin_latitude` or `map_origin_longitude` is set anywhere under `rolling/platforms`. `mru_transform.cpp:167-181` therefore anchors `map` "at the first position fix". `sea_surface_estimator.hpp:367-370` publishes `map_tide` as a z-only translation of the odometry header frame (map), with identity rotation. So `map_tide`'s horizontal origin is the first fix.
     - **What follows.** At start the window is centred on the boat, so its lower-left is at (-175, -175) and (-2500, -2500). The strip exists from the first cycle and stays while the boat is less than half a window east or north of the start point.
     - **Not established.** Whether the odometry header frame is in fact `map` on the boat. Whether `reset_map_frame` is used in operation; it re-anchors at a later fix. Whether `resizeMap` runs again in operation; a dynamic size change would re-run `matchSize`. Nothing was run on a vessel.
5. **Verification statement for the pull request.**
   - **Shown on Rolling.** The scratch copy with the port's changes: each case's before and after as in the table, the `lround` case before and after, 47/47 `s57_layer` tests, and 20/20 repeats.
   - **Shown by Jazzy CI.** The new test passes against released Nav2. Nav2's `origin/jazzy` `layered_costmap.cpp` and `costmap_2d.cpp` differ from `main` only in the `updateMap` early-return restructuring, not in the `updateOrigin` arithmetic. The tests assert the window origins they rely on.
   - **Not shown.** Any vessel, planner or costmap-node run.
   - **Two-step push: not recommended**, but it is the host's call. The arithmetic Jazzy runs is the same by reading, and the origin assertions guard against vacuity. If the host wants it, push the test-only commit (step 6 alternative) first: one extra Jazzy CI run, expected to fail 5 cases, then the fix.
6. **Commits.** Each builds and passes on Jazzy on its own; nothing under `.agent/` is in either.
   1. `s57_layer: find a rolling window's tiles below the layer origin (#56)`. Contents: `worldToTile` `std::floor`, `s57_layer.h` protected chart members, `test/test_rolling_window.cpp` (the six cases) and the CMake target.
   2. `s57_layer: round the master-to-layer cell offset to the nearest cell (#56)`. Contents: `lround` at `:469` and `:474`, and the 0.1 m test case.

   **Alternative for a two-step push:** split commit 1 into the test commit (header, test, CMake) followed by the floor commit, and likewise split commit 2. The backport is `git cherry-pick -x` of the code commits in order. The records travel in the pull request only.

   **Merge order with #52 part 2:**
   - **This branch first:** the port rebases. The likely conflict is the test registration in `s57_layer/CMakeLists.txt` (the port adds `test_initialize` with `ament_add_ros_isolated_gtest`). The new test already uses the `__has_include` switch, so the port's commit 4 need not touch it. `.cpp` edits do not overlap: the port edits lines 10, 35-124 and 655; this branch edits 159-160, 469 and 474. The port's plan section "Found defect 2" becomes "fixed by #56".
   - **Port first:** this branch rebases, with the same CMake conflict. The new test is then re-run on the real worktree build, not on a scratch copy.
   - **`jazzy`:** the same pair of conflicts, in whichever order the backports land.

## Files to Change

| File | Change |
|---|---|
| `s57_layer/src/s57_layer.cpp` | `worldToTile` floor (commit 1); `lround` at 469 and 474 (commit 2) |
| `s57_layer/src/s57_layer.h` | `current_charts_`, `chart_bounds_`, `grids_` moved to protected, with a comment (commit 1) |
| `s57_layer/test/test_rolling_window.cpp` | new (commit 1); offset case (commit 2) |
| `s57_layer/CMakeLists.txt` | `ament_add_gtest(test_rolling_window ...)` (commit 1) |

## Prior Art

`bathymetry_layer` has both conversions right (`floor` at 349-355 and `lround` at 1190-1215), and both are taken. Its tests use only non-rolling `LayeredCostmap`s, so its test design was not reusable for the window. The existing `s57_layer` tests' direct `updateBounds`/`updateCosts` calls and their parameter pre-declaration were taken. The port's DDS-based grid delivery (its test 2) was rejected here in favour of protected members, because it needs no service, topic or TF.

## Serves

None (the project states no goals). It fixes charted hazards missing from the costmap, the purpose of `s57_layer`.

## Design Picture

No change to the design picture. The project has no design document, and no interface changes. The header change is protected test access only.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Two conversions and their tests. Defects A and B below are reported, not fixed |
| Test what breaks | Every case was shown failing without its fix and passing with it, on Rolling |
| Verify before claiming | Jazzy is CI only. Nothing about a vessel or planner was run. What is known and not known about `map_tide` is stated |
| A change includes its consequences | The CMake conflict with the port is planned for. The README needs no change |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| None | No | Bug fix in a project package |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `s57_layer/CMakeLists.txt` tests | the port's branch, at rebase | Yes (merge order above) |
| `s57_layer.h` access | the port's plan, if it injects grids the same way | No: the port uses DDS |
| fix on `rolling` | `jazzy` backport by cherry-pick | Yes, host |

## Open Questions

- **Two-step push?** Recommendation no; the host decides.
- **Defects found on the way, both distros, not #56 (the host decides: new issue or #54):**
  - **A.** After a window moves within tiles already copied (`needs_update` false), the newly exposed cells are not in the layer's bounds and keep `updateOrigin`'s default. Scratch: positive space, a 1-cell move east leaves 10 of 100 cells free on all-land. On BEN the inflation layer only re-expands bounds the layer already reported. Tide changes over 1 cm and chart-set changes invalidate every tile, which repaints everything. How long cells stay unpainted on a vessel is not established.
  - **B.** `generateTile:347-354` (empty chart list) does not set `needs_update`, so with no charts at all and `allow_uncharted: false` a real `updateMap` cycle marks nothing unknown (100 of 100 free, before and after #56).

## Estimated Scope

Single pull request into `rolling`, two commits; then the backport.
