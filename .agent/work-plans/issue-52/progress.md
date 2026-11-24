---
issue: 52
---

# Issue #52 — Port s57_tools to ROS 2 Rolling

## Issue Review
**Status**: complete
**When**: 2026-11-24 08:55 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)

**Issue**: #52

### Scope Assessment

**Well-scoped?** Yes, with one Done-when item that cannot be met as written (below). Six packages, one repository, a known recipe from #415 (causes 1 to 16). It is larger than the earlier single-package ports, and it splits cleanly in two (see Recommendations).
**Right repo?** Yes. The fixes belong in s57_tools; the `core.repos` pin is a separate small change in unh_marine_autonomy, as the issue says.
**Serves**: none written down (s57_tools has no goals or design document); in the workspace's terms, keeping the project on the owner's Rolling track (#415).
**Design picture?** No change. No design document exists for the project.
**Dependencies**: rolker/unh_marine_autonomy#415 (tracking). Built-in dependencies from other repositories: `marine_autonomy`, `marine_vertical_datum`, `marine_bathymetry_store` (ported, in the hosted core install); `geodesy`, `grid_map_core`, `grid_map_ros`, `nav2_*` from the development underlay on Rolling.

**What each package is and depends on** (from `package.xml`):

| Package | Type | Depends on within the six | Needs Nav2 or `grid_map_ros` |
|---|---|---|---|
| `s57_msgs` | ament_cmake, messages | none | no |
| `marine_charts` | C++ library, bundled ISO 8211 reader | none | no (`grid_map_core` only) |
| `s57_to_geotiff` | C++ CLI | `marine_charts` | no (also `marine_autonomy`, `marine_vertical_datum`) |
| `enc_updater` | ament_python | `s57_to_geotiff` (exec) | no (also `marine_bathymetry_store`) |
| `s57_grids` | C++ node, lifecycle | `marine_charts`, `s57_msgs` | yes: `grid_map_ros` (pulls `nav2_msgs`) |
| `s57_layer` | Nav2 costmap plugin | `s57_grids`, `marine_charts`, `s57_msgs` | yes: `nav2_costmap_2d`, directly and through `s57_grids` |

Graph: `s57_msgs` and `marine_charts` are leaves; `s57_to_geotiff` -> `marine_charts`; `enc_updater` -> `s57_to_geotiff`; `s57_grids` -> `marine_charts`, `s57_msgs`; `s57_layer` -> `s57_grids`, `marine_charts`, `s57_msgs`. The Rolling-CI-able set is `s57_msgs`, `marine_charts`, `s57_to_geotiff`, `enc_updater`. `s57_grids` and `s57_layer` cannot be (the owner's decision of 2026-11-24: no job builds Nav2 source, not even `nav2_msgs`).

What a Rolling job needs for the four: `grid_map_core` is not released for Rolling, so the job clones the grid_map repository (`rolling` branch; the underlay has it at `0ece1b5`) and builds `--packages-up-to marine_charts`. `grid_map_core` depends only on `eigen` and `grid_map_cmake_helpers` (which depends on `ament_cmake_core` only), so building the closure does not pull `nav2_msgs` in; `grid_map_ros` and the rest of that repository stay unbuilt. For `s57_to_geotiff` and `enc_updater` the job also needs `marine_autonomy`, `marine_vertical_datum` and `marine_bathymetry_store` (and their closure inside unh_marine_autonomy, none of it Nav2) from a clone of unh_marine_autonomy at `rolling`, the way mru_transform's job does it, with `rosdep install` without `-r` over the build closure (cause 11).

### Findings against the issue

**1. The Jazzy half of Done-when cannot be met as the workflow stands.** The existing `ci.yml` builds and tests `s57_msgs marine_charts s57_grids s57_layer` only (`--packages-up-to` / `--packages-select`). `s57_to_geotiff` and `enc_updater` are in neither step, and neither could be: their dependencies (`marine_autonomy`, `marine_vertical_datum`, `marine_bathymetry_store`) are not in that job, and `rosdep install -r` would hide the missing keys. So "all six build and test on Jazzy (CI)" needs the workflow to grow (a clone of unh_marine_autonomy on both distros), or the Done-when to say four on Jazzy by CI and two by another route. Also the job triggers on `jazzy` only and its `geodesy` clone is `-b ros2` unpinned (cause 8: needs `f70b81a` or later for `ecef.h`).

**2. The table of known causes is incomplete.** First Rolling build and test in the package worktree (`./build.sh --parallel-workers 4 --continue-on-error`) stops at the first package: `marine_charts` fails and the other four are not processed. To see past it, I made a throwaway copy in the session scratchpad, applied only the minimum to get over each error, and rebuilt (nothing was changed in the worktree). Result by package:

| Package | Result on Rolling | Error class, file and line | #415 cause |
|---|---|---|---|
| `s57_msgs` | builds; 4 lint tests pass | none | none |
| `marine_charts` | FAILS to build, in the worktree | `CPL_CVSID("$Id$")`: "expected constructor, destructor, or type conversion" in `src/iso8211/ddffield.cpp:39`, `ddfutils.cpp:37`, `ddfmodule.cpp:43`, `ddfsubfielddefn.cpp:43`, `ddfrecord.cpp:44`, `ddffielddefn.cpp:42`. The `nLeaderSize`/`szLeader`/`achLeader` errors in `ddfrecord.cpp:192-269` are a cascade of the first and vanish with it (the constant is defined at line 46). After deleting the six lines it builds. | 3 (the list gave six files; confirmed) |
| `s57_to_geotiff` | builds after that fix; 6 gtests + lint pass | none | none |
| `enc_updater` | builds; `colcon test` FAILS: "Ran 0 tests ... NO TESTS RAN", exit 5 | `setup.py:20` `tests_require=['pytest']` | 4 (confirmed) |
| `s57_grids` | builds after the `marine_charts` fix, against underlay `grid_map_ros`; launch test (2) + lint pass | none. The one `TransformListener` is `make_shared<TransformListener>(*tf_buffer_)` (`grid_publisher.cpp:115`), the form that is unchanged on both distros. `s57_catalog.cpp:67` `create_subscription` is on an `rclcpp_lifecycle` node, not Nav2's. | 14 does not apply; 7 does not apply here |
| `s57_layer` | FAILS at configure, then at compile (each step below found by patching the scratch copy) | (a) `CMakeLists.txt:28` `ament_target_dependencies`. (b) `src/s57_layer.cpp:10` `tf2/utils.h`. (c) `s57_layer.cpp:35` and 14 more `declareParameter` (15 calls, lines 35 to 121). (d) `s57_layer.cpp:124` `node->create_client<GetDatasets>()` returns `nav2::ServiceClient` on Rolling and no longer assigns to the `rclcpp::Client::SharedPtr` member `get_datasets_client_` (`s57_layer.h:77`). (e) `s57_layer.cpp:655` `node_.lock()->create_subscription<GridMap>(...)` hidden by `nav2::LifecycleNode`. Tests not reached; `test_depth_costs.cpp`, `test_allow_uncharted.cpp`, `test_tide_offset.cpp` still use `nav2_util::LifecycleNode`. | (a) 1 and 8 (`geodesy`, and `geodesy/ecef.h` at `s57_layer.cpp:5` needs `version_gte="1.0.7"`); (b) 2; (c) 6; (d) NEW, see below; (e) 7; tests 13 |

The issue's table matches the source for causes 1, 2, 4, 7, 13, 14 (14: no change needed). It misses: cause 8 (not mentioned: the `geodesy` target and the 1.0.7 floor, and `ecef.h` is used), cause 3's concrete list (six files, `marine_charts`), and a **new cause**: Nav2's `nav2::LifecycleNode::create_client` returns a `nav2::ServiceClient` wrapper, as `create_subscription` is hidden (cause 7); the free `rclcpp::create_client(node, name)` returns what Jazzy's member did. `s57_layer` uses `prune_requests_older_than` and `async_send_request` on it, so the free function is the one that keeps the code unchanged. This is a candidate cause 17 for #415. Cause 9 (`package.xml` order) and cause 11 (undeclared dependencies: `s57_layer` uses `grid_map_ros`/`grid_map_msgs` and `tf2_ros` without declaring them, relying on `s57_grids`) are not shown by a build here and are for the plan to check with `ament_xmllint` and `rosdep` without `-r`. Cause 12 (GDAL 3.12 `GDALAllRegister()` race): `s57_catalog.cpp:15` calls it in a constructor and `s57_to_geotiff/test/test_exporter.cpp:25` per test; whether anything runs it concurrently is for the plan to check. Causes 5, 10, 15, 16 do not appear (no `.warn(`, no `gz4d_geo.h`, no module shadowing, no `Waitable`).

**3. What the tests cover, and what they do not.** `s57_layer` (on the navigation path: it marks charted hazards, land, restricted areas and depth cost in the costmap) has 23 gtests: `test_depth_costs` (12: the cost ramp, hazard, land and restricted stay lethal, caution, suppressed mode), `test_tide_offset` (7), `test_allow_uncharted` (3), `test_plugin_loading` (1). They cover what the layer marks, in the layer's own functions. None checks that an operator override of a parameter is read under the layer's full name, which is exactly what replacing `declareParameter` (cause 6) can break silently, and no test builds a subscription from `getDatasetsCallback` (cause 7) or checks the client. So compiling is not evidence. #415's `bathymetry_layer` and `ais_layer` ports added an overrides test and an `initialize()` test for the same reason; `s57_layer` needs both, and a mutation check of each. `s57_grids`, the publisher of the grids `s57_layer` consumes, has only a launch test (2 tests) and lint. `marine_charts` has no unit tests, only lint, so the ISO 8211 reader and the S57-to-grid conversion are not exercised under GDAL 3.12 by any test; the `s57_grids` launch test and the chart data are the only runs. `s57_to_geotiff` has 6 gtests (the exporter, the CATZOC sigma table). `ament_cppcheck` skips on Rolling (22 tests skipped across the four packages that built).

**4. Consumers elsewhere under `/home/roland/project11/rolling`.** `ben_project11` launches `s57_grids_launch.py` under the namespace `s57` (`launch/ben_core_launch.py:162-168`) and names `s57_layer::S57Layer` in `config/costmap_chart.yaml:2` (also `nav2_params.yaml`, `navigator.yaml`, `ben.yaml`). `s57_layer` and `bathymetry_layer` in unh_marine_autonomy are designed to compose (the README says so); `bathymetry_layer` mirrors `s57_layer::get_cost_from_grid`. `mru_transform`'s `chart_datum_launch.py` and `marine_web_view` mention the chart layers by name only. Nothing else builds against these packages' headers. `unh_marine_autonomy/config/repos/core.repos:22` pins s57_tools.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Only what's needed | OK | The port stays to the causes and the CI/branch work; leftovers go to a follow-up issue, as the owner settled. |
| Test what breaks | Action needed | The issue says `s57_layer` must not change what it marks and that "the plan says how that is shown". The existing tests do not read overrides or run `initialize()`; the plan must add an overrides test (parameters declared under the full name, each override differing from the member's starting value) and an `initialize()`/subscription/client test, mutation-checked, as `ais_layer` and `bathymetry_layer` have. |
| Verify before claiming | Action needed | Done-when says all six on Jazzy by CI, but the workflow builds four and could not build `s57_to_geotiff` or `enc_updater` as it is. Either the workflow grows or the sentence changes; the owner decides, since it changes what "done" is. |
| Put each thing at the level it applies to | OK | Branch policy, pin and follow-up issue are placed correctly. |
| Keep one current design | OK | No design document in the project; nothing to update. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| Progress.md entry types (ADR-0013) | Yes | This entry; the plan and the later entries go in `progress.md`; the records go with the pull request, not in a code commit. |
| Other ADRs | No | None bears on a port inside a project repository; the fixes stay on forms that also build on Jazzy. |

### Consequences

- `core.repos` pin to `rolling` in unh_marine_autonomy (named in the issue).
- `ben_project11` loads `s57_layer` on Rolling: no simulation or costmap run of the layer is part of this work; say so in the records, as #418 did for `bathymetry_layer`.
- Cause 17 (candidate) for the #415 table, and the `s57_layer` row in cause 6's "where else" list is now done.

### Recommendations

- Split the work into two parts under this issue, as the other ports were: part 1, the four Nav2-free packages (`s57_msgs`, `marine_charts`, `s57_to_geotiff`, `enc_updater`) with the `rolling` branch and the CI changes (Rolling job cloning grid_map for `grid_map_core` and unh_marine_autonomy for the three store/datum packages; Jazzy job unchanged for those it already builds, extended if the owner chooses); part 2, `s57_grids` and `s57_layer` (checked by hand on Rolling against the underlay's navigation2 `9113449e`, `grid_map` `0ece1b5`, geodesy at or above 1.0.7, runs recorded, and Jazzy by CI). Part 1 can merge alone, and it unblocks `s57_to_geotiff` and `enc_updater` for the hosted core install; part 2 is where the risk sits.
- Ask the owner whether Done-when should read "four packages on Rolling in CI, two by hand; on Jazzy the four the workflow builds today, the other two by the extended workflow or by hand", rather than "all six (CI)" for Jazzy.
- In the plan, record the new cause (`create_client` on `nav2::LifecycleNode`, fix: `rclcpp::create_client(node, name)`), alongside causes 6 and 7, and a comment on #415 for it after the pull request.
- In the plan's `s57_layer` section, use the local `declareParameter` helper and the free `rclcpp::create_subscription` and `rclcpp::create_client`, state `geodesy` as `<depend version_gte="1.0.7">` with `find_package(geodesy 1.0.7 REQUIRED)`, and pin the Jazzy job's `geodesy` clone at `f70b81a` (the present `-b ros2` clone is unpinned).
- In the plan, check by hand what a build cannot show: `ament_xmllint` on every `package.xml` (cause 9), `rosdep install` without `-r` (cause 11: `s57_layer`'s undeclared `grid_map_ros`/`tf2_ros`), and the `GDALAllRegister()` call sites for concurrency (cause 12).
- Mention in the plan that `marine_charts` has no behavioural test: either add a small one over a bundled test cell for the ISO 8211 reader (since GDAL 3.12 changed the macros that file set uses) or record in the follow-up issue that the Rolling check is the `s57_grids` launch test and a real chart run.

### Actions
- [ ] The issue says `s57_layer` must not change what it marks and that "the plan says how that is shown". The existing tests do not read overrides or run `initialize()`; the plan must add an overrides test (parameters declared under the full name, each override differing from the member's starting value) and an `initialize()`/subscription/client test, mutation-checked, as `ais_layer` and `bathymetry_layer` have.
- [ ] Done-when says all six on Jazzy by CI, but the workflow builds four and could not build `s57_to_geotiff` or `enc_updater` as it is. Either the workflow grows or the sentence changes; the owner decides, since it changes what "done" is.
- [ ] Split the work into two parts under this issue, as the other ports were: part 1, the four Nav2-free packages (`s57_msgs`, `marine_charts`, `s57_to_geotiff`, `enc_updater`) with the `rolling` branch and the CI changes (Rolling job cloning grid_map for `grid_map_core` and unh_marine_autonomy for the three store/datum packages; Jazzy job unchanged for those it already builds, extended if the owner chooses); part 2, `s57_grids` and `s57_layer` (checked by hand on Rolling against the underlay's navigation2 `9113449e`, `grid_map` `0ece1b5`, geodesy at or above 1.0.7, runs recorded, and Jazzy by CI). Part 1 can merge alone, and it unblocks `s57_to_geotiff` and `enc_updater` for the hosted core install; part 2 is where the risk sits.
- [ ] Ask the owner whether Done-when should read "four packages on Rolling in CI, two by hand; on Jazzy the four the workflow builds today, the other two by the extended workflow or by hand", rather than "all six (CI)" for Jazzy.
- [ ] In the plan, record the new cause (`create_client` on `nav2::LifecycleNode`, fix: `rclcpp::create_client(node, name)`), alongside causes 6 and 7, and a comment on #415 for it after the pull request.
- [ ] In the plan's `s57_layer` section, use the local `declareParameter` helper and the free `rclcpp::create_subscription` and `rclcpp::create_client`, state `geodesy` as `<depend version_gte="1.0.7">` with `find_package(geodesy 1.0.7 REQUIRED)`, and pin the Jazzy job's `geodesy` clone at `f70b81a` (the present `-b ros2` clone is unpinned).
- [ ] In the plan, check by hand what a build cannot show: `ament_xmllint` on every `package.xml` (cause 9), `rosdep install` without `-r` (cause 11: `s57_layer`'s undeclared `grid_map_ros`/`tf2_ros`), and the `GDALAllRegister()` call sites for concurrency (cause 12).
- [ ] Mention in the plan that `marine_charts` has no behavioural test: either add a small one over a bundled test cell for the ISO 8211 reader (since GDAL 3.12 changed the macros that file set uses) or record in the follow-up issue that the Rolling check is the `s57_grids` launch test and a real chart run.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 08:43 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: owner
**After**: issue-actions
**Decision**: proceed

The issue review recommended two parts; the owner, shown the cut on 2026-11-24, answered "Two parts, Nav2-free first (Recommended)". **Part 1**: `s57_msgs`, `marine_charts`, `s57_to_geotiff`, `enc_updater`, the `rolling` branch and CI on both distros; the Jazzy job grows to build and test all six packages, as the issue's "Done when" says (today it covers four; the other two need a clone of rolker/unh_marine_autonomy, and `rosdep install` loses its `-r`). **Part 2**: `s57_grids` and `s57_layer`, checked on Rolling by hand (no CI job builds Nav2 source), where the navigation-path risk sits. Each part is its own pull request into `rolling` saying `Part of #52`, with its own hand backport. The plan covers both parts and details part 1; part 2 is planned again at the plan checkpoint after part 1 merges. For the planner, from the review: causes the issue's table missed (cause 3, `CPL_CVSID` in six `marine_charts` files; cause 8, `geodesy`; causes 9, 11 and 12 to check by hand; a new cause in `s57_layer`, `nav2::LifecycleNode::create_client` returning a wrapper, candidate 17 on #415); `marine_charts` has no behavioural test, so the plan says what shows that its ISO 8211 reader still reads charts under GDAL 3.12 after the cause 3 fix (the six files are a vendored copy of GDAL's reader: say what the smallest safe fix is and how it is checked); part 2 needs the parameter-override and `initialize()` tests the other two costmap layers got.
