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

## Plan Authored
**Status**: complete
**When**: 2026-11-24 08:55 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Plan**: `.agent/work-plans/issue-52/plan.md` at `e81764c`

Part 1 (to implement now): delete the six `CPL_CVSID` lines (cause 3), add a mutation-checked `marine_charts` test reading NOAA's real `CATALOG.031` through the vendored ISO 8211 reader (the existing tests never touch it), `extras_require` in `enc_updater` (cause 4, 0 to 117 tests), declare `ros2node` (cause 11), and one `ci.yml` for both branches: Jazzy builds and tests all six with strict rosdep and geodesy pinned at `f70b81a`, Rolling the four Nav2-free packages against grid_map `0ece1b5` and unh_marine_autonomy `rolling` (rehearsed: 159 results, 0 failures). Part 2 (`s57_grids`, `s57_layer`) is outlined with its causes, tests and hand runs, to be planned again after part 1 merges. Draft PR step skipped: the host owns every push.

## Plan Review
**Status**: complete
**When**: 2026-11-24 09:04 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: ready

**Issue**: #52 — Port s57_tools to ROS 2 Rolling
**Plan**: `.agent/work-plans/issue-52/plan.md` at `e81764c`
**Branch**: `feature/issue-52`

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Part 1 is five small commits (six deleted lines, one test with a 1347-byte fixture, two manifest/setup lines, one workflow), as the owner's two-part cut asks. Part 2 is an outline only. |
| Issue alignment | Good | Every issue-review action that belongs to part 1 is covered: cause 3 with a behavioural test of the reader, causes 4, 9, 11, 12 checked, the Jazzy job grown to all six with strict rosdep and geodesy pinned. Part 2's actions (overrides and `initialize()` tests, cause 17) are in the outline. Serves: "None written down (s57_tools has no goals or design document); for the workspace, the Project11 Rolling port, rolker/unh_marine_autonomy#415." Accepted: the project has no goals written down. |
| File targeting | Good | The six `iso8211/*.cpp`, the new test and fixture, `CMakeLists.txt`/`package.xml`, `enc_updater/setup.py`/`package.xml`, `ci.yml`. Nothing else needs changing: the README says nothing about CI or test layout. |
| Consequences | Needs work | The fixture's README leaves out what NOAA's user agreement asks of redistributed copies (finding 1). The `core.repos` pin and the cause 17 comment are correctly left to their own changes. |
| Principle alignment | Good | Test what breaks: the new test fails under three mutations (repeated here). Verify before claiming: Jazzy stated as CI only. The one claim stated as not checkable here can be checked (finding 2). |
| ADR compliance | Good | ADR-0013 only; records go in their own commit, as the owner settled. |
| ROS conventions | Good | `ros2node` as `exec_depend` is the right kind; `extras_require={'test': [...]}` is the form other ports used; `ament_add_gtest` with `ament_cmake_gtest` as test_depend. |

### Checks run for this review

- **Cause 3.** `S57Catalog`'s constructor (`marine_charts/src/s57_catalog.cpp:26-47`) is the only user of the vendored `DDF*` classes; cells are opened with `GDALOpenEx` (`s57_dataset.cpp:63,469`); `S57Catalog` is built by `s57_grids/src/grid_publisher.cpp:37` and `s57_to_geotiff/src/exporter.cpp:543`. On a scratch copy with only the six lines deleted, `marine_charts` builds under GCC 15 and GDAL 3.12.2 with `-Wall -Wextra -Wpedantic` and no warning from `src/iso8211` (only `tf2_ros/buffer.h` and `s57_catalog.cpp:55`). Symbol clash: `libgdal.so` 3.12.2 here and `libgdal.so.34` from noble's `libgdal34t64_3.8.4+dfsg-3ubuntu3` (downloaded and read with `nm -D`) export no `DDFModule`, `DDFRecord`, `DDFField`, `DDFFieldDefn`, `DDFSubfieldDefn` or `DDFScan*`/`DDFFetch*` symbol (both builds hide them; only `DDFRecordIndex` and `S57Reader` methods that take `DDF*` pointers are exported), so `libmarine_charts`'s copies cannot interpose on GDAL's own S-57 driver, nor the reverse, on either distro. The vendoring is needed for that same reason: GDAL does not export the reader. The copy uses `nullptr` and still has `CPL_CVSID`, so it dates from GDAL 2.3 to about 3.5 (2018 to 2022); upstream's reader has had changes since. It reads only NOAA's `CATALOG.031` from an exchange set the updater downloads, so the age matters little here; it belongs in the follow-up issue.
- **The fixture and test.** Downloaded `https://charts.noaa.gov/ENCs/US5PSMCC.zip` today: `ENC_ROOT/CATALOG.031` is 1347 bytes, sha256 `10a94bcfcff9a9aaa4f913304e6a100546579a6c2bf4654911f93b389440da8f` (as the plan says). Its bytes hold 8 `CATD` records; the only `.000` is `US5PSMCC\US5PSMCC.000` with `43.050000^-70.800000^43.125000^-70.725000` (SLAT, WLON, NLAT, ELON); `.001` and `.002` carry the same bounds and are not datasets by the reader's rule. Wrote an equivalent two-test gtest on a scratch copy (all-world query gives one dataset `US5PSMCC.000` with those bounds; a box inside selects 1, boxes east and north select 0): passes; each of three mutations fails both tests: `0.5*CPLAtof(...)` in `ExtractFloatData` (`ddfsubfielddefn.cpp`, the `A/I/R/S/C` case), `"SLAT"` read as `"NLAT"` at `s57_catalog.cpp:38`, catalogue path misnamed. The box queries are what `grid_publisher.cpp:171,427` and `exporter.cpp:544` call, so the test covers the downstream selection.
- **Cause 11.** `nav_liveness.py` runs `ros2 node list` only when `nav_liveness.nodes` is configured; any probe error raises `InterlockRefusal` (fail closed), so without `ros2node` the updater never swaps the chart layer: the vehicle keeps its old charts and the operator gets a refusal, a failure to update, not a silent one. `ros2node` resolves on jazzy/noble (`ros-jazzy-ros2node`) and rolling/resolute (`ros-rolling-ros2node`). `ros_core` already pulls it in (through `ros2cli_common_extensions`), so the gap bites only on a minimal install, and the `ros-core` CI images cannot show it. Every other subprocess (`s57_to_geotiff`, `import_geotiff` via the ament index) and every import (`osgeo`, `yaml`, `ament_index_python`, stdlib) is declared.
- **Cause 4.** Scratch copy with the `extras_require` change: `colcon test` gives 117 tests, 0 errors, 0 failures, 0 skipped (`test_sanity`'s `importorskip('osgeo.gdal')` ran, not skipped); `build/enc_updater/pytest.xml` exists where the guard step reads it.
- **CI.** Over `git archive` copies of unh_marine_autonomy `origin/jazzy` (`ff6bd708`) and `origin/rolling` (`0ca4a587`) with s57_tools `HEAD` and grid_map `0ece1b53`: Rolling closure is the four plus `marine_autonomy`, `marine_interfaces`, `marine_tiled_raster_store`, `marine_vertical_datum`, `marine_bathymetry_store`, `grid_map_core`, `grid_map_cmake_helpers` (11, no Nav2, no `bathymetry_layer`); Jazzy closure of all six takes the same five from unh_marine_autonomy (plus `geodesy` once the clone is present). Every external key of the Jazzy six-package closure resolves on noble (`rosdep resolve --os=ubuntu:noble`, including `nav2_costmap_2d`, `nav2_util`, `grid_map_ros`, `tf_transformations`, `marine_acoustic_msgs`, `python3-gdal`, `ros2node`), and every key of the Rolling closure on resolute. The two-job shape, the `if:` on the Rolling job and the per-job `-b jazzy`/`-b rolling` clone match mru_transform's `ci.yml`: one file serves both branches because each job names its own branch and the Rolling job skips on `jazzy`. Time estimates agree with today's Jazzy runs (3.3 to 3.6 min) and the models' latest runs.
- **Part 2 against part 1.** Part 1 changes no header or symbol `s57_grids` or `s57_layer` use. Cause 17 is real: Rolling's `nav2::LifecycleNode::create_client` (`nav2_ros_common/lifecycle_node.hpp:211`) returns `nav2::interfaces::create_client`'s wrapper.

### Findings

1. **[Consequences]** (Medium) The fixture README leaves out what NOAA asks of redistributed copies. NOAA's user agreement (`USERAGREEMENT.TXT` in the same zip, and the copy unh_marine_simulation keeps at `asv_sim/enc/NOAA_USERAGREEMENT.TXT`) says ENCs "may be redistributed without restriction", and also that copying "for further distribution is discouraged unless ... this User Agreement is displayed ... and a reference to this Web site is included". It also says redistributed copies are not official NOAA ENCs. The plan's README gives the source URL and the redistribution sentence but leaves the agreement out on purpose, because of its CRLF line endings. Committing the file is safe: the data is a US Government product with no copyright claim (the agreement names only trademarks), and the precedent is in the project already. To follow the agreement, add the agreement file. Either convert it to LF line endings (the text does not change) or add the path to the `exclude:` of the whitespace hooks in `.pre-commit-config.yaml`. Also say in the README that the file is NOAA data, not covered by `marine_charts`' Apache-2.0 `LICENSE`, and that it is not for navigation, as `asv_sim/enc/README.md` says. A synthetic catalogue, built byte by byte in the test, would remove third-party data. It would cost about 50 lines of ISO 8211 leader and directory code, and the format controls (`(A(2),I(10),3A,A(3),4R,2A)`) would be ones the test author wrote, not ones a real producer wrote. The owner decides which; the real file with the agreement is the smaller change.
2. **[Principle alignment]** (Low) The plan keeps `source install/setup.bash` in the Jazzy test step because "the `s57_grids` launch test runs under it today; dropping it cannot be checked here". It can be checked here. On a scratch Rolling workspace (`s57_msgs`, `marine_charts`, `s57_grids`, with `s57_grids` absent from the underlay and core installs), `colcon test --packages-select s57_grids` passed without sourcing the overlay: 3 results, 0 failures, `ROS_DOMAIN_ID=187` checked empty first. colcon gives that test the same environment on Jazzy. Sourcing the overlay hides a missing workspace `test_depend`, which is the reason mru_transform's header gives for not doing it. Drop the line to match the models. If it stays, give a reason the run above does not answer. unh_marine_navigation's broad Jazzy phase does keep it, so either is defensible. Stating the reason correctly is not optional.
3. **[Scope: part 2 outline]** (Low) The outline's cause 12 concern for `grid_publisher.cpp` is unlikely. The concern is that `GDALAllRegister()` in `on_configure` runs while `std::async` grid jobs from line 226 are still running. But `on_cleanup` (lines 146-163) sets `abort_flag_` and waits on every pending future before it returns, and a lifecycle node can configure again only after cleanup. The same code has a pre-existing defect: `abort_flag_` is set at line 148 and never reset (its only other use is `grid_publisher.h:122`, the initialiser). After a cleanup and a new configure, every `S57Dataset::getGrid` returns a null grid at `s57_dataset.cpp:177`. The result is then dereferenced in `GridMapRosConverter::toMessage(*grid)` (around `grid_publisher.cpp:258`): a crash, or charts silently missing from the costmap. This is on both distros and is not a port cause. Record it in part 2's plan or the follow-up issue. Nothing in it changes part 1.
4. **[File targeting]** (Info) The vendored reader dates from about 2018 to 2022 (it has `nullptr` and still has `CPL_CVSID`), older than upstream's reader. Add a line to the "Left alone" list: compare it with upstream's `frmts/iso8211` for hardening fixes. Part 1 needs nothing more: it compiles cleanly on GDAL 3.12, and neither distro's `libgdal` exports its classes, so no symbol clash.

### Summary

Part 1 is sound and checked. The cause 3 fix is the whole of what GDAL 3.12 needs from the vendored reader. The new test reproduces and catches the three mutations. `enc_updater` runs 117 real tests. The CI closures and rosdep keys resolve on both distros as the plan says. Before implementation, fold in the fixture's licence text (an owner decision between the real catalogue with NOAA's agreement and a synthetic one) and the test-step sourcing.

### Recommended Actions

- [ ] Add NOAA's user agreement next to `CATALOG.031`, LF-normalised or excluded from the whitespace hooks. In `test/data/README.md`, state that the file is NOAA data, outside the package licence and not for navigation. Or, if the owner prefers, use a synthetic catalogue.
- [ ] Drop `source install/setup.bash` from the Jazzy test step, or keep it with a reason that holds: it is checkable on Rolling and passes without the line.
- [ ] Part 2 / follow-up issue: record `abort_flag_` never being reset in `s57_grids` (a null grid dereferenced after cleanup and configure). Restate cause 12 for `grid_publisher.cpp` as unlikely, because `on_cleanup` waits on every pending future. Add the age of the vendored reader to "Left alone".

## Checkpoint
**Status**: complete
**When**: 2026-11-24 09:04 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the plan review's verdict is ready; its findings have concrete corrections the implementer applies; the third-party fixture follows a precedent already in the project and is listed for the owner's veto)
**After**: plan
**Decision**: proceed

Read: the Plan Review at `e81764c`, verdict ready, four findings. Part 1 is implemented now; part 2 is planned again after part 1 merges. (1) **The NOAA fixture is committed as planned, with what NOAA's user agreement asks**: the agreement file beside it (LF line endings, text unchanged, or excluded from the whitespace hooks if conversion would alter it: say which), and a README that gives the source URL and download date, says the file is NOAA data not covered by the package's Apache-2.0 licence, and that it is not for navigation, as rolker/unh_marine_simulation's `asv_sim/enc` already does for the same data. The alternative was a synthetic catalogue written by the test (about 50 lines, and no real producer's format checked). The owner may veto before the merge. (2) The Jazzy test step drops `source install/setup.bash`, matching the other repositories' workflows: the reviewer showed the `s57_grids` launch test passes without it, and sourcing the overlay hides a missing workspace test dependency. (3) and (4) go to part 2's plan and the follow-up list, not into part 1: `abort_flag_` in `s57_grids`' `grid_publisher` is set at cleanup and never reset, so after a cleanup and a new configure every grid request returns null and is then dereferenced (a crash, or charts missing from the costmap, on both distros); the vendored ISO 8211 reader is older than upstream's and should be compared with it for hardening fixes. Host decisions on the planner's open questions: `ros2node` is declared now, in its own commit; grid_map is pinned by commit in the Rolling job.

## Implementation
**Status**: complete
**When**: 2026-11-24 09:20 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Branch**: feature/issue-52 at `e954dbc3a9bff142f34e95e16c27448879c4024d`

Part 1 of 2 (`s57_msgs`, `marine_charts`, `s57_to_geotiff`, `enc_updater`, CI on both distros). `s57_grids` and `s57_layer` not touched. Five commits, none touching `.agent/`, each ending "Backport to jazzy.":

1. `79d64d6` marine_charts: drop CPL_CVSID from the vendored ISO 8211 reader (#415 cause 3): the six lines only.
2. `2d5efe0` marine_charts: test reading an S-57 exchange-set catalogue: `test/test_s57_catalog.cpp` (2 gtests), `test/data/enc_root/CATALOG.031` (1347 bytes, sha256 `10a94bcf…40da8f`), `test/data/enc_root/USERAGREEMENT.TXT`, `test/data/README.md`, `ament_cmake_gtest` test_depend, and a `.pre-commit-config.yaml` exclusion.
3. `ae81fb9` enc_updater: tests_require to a test extra (#415 cause 4).
4. `961e1a7` enc_updater: declare ros2node (#415 cause 11).
5. `e954dbc` ci: Jazzy job builds all six packages; Rolling job for the four.

### Verification (Rolling, this machine; `ROS_DOMAIN_ID=173`, checked empty)

- `./build.sh --parallel-workers 4 --packages-up-to s57_msgs marine_charts s57_to_geotiff enc_updater`: `Summary: 4 packages finished [1.22s]` (incremental; first full build `Summary: 4 packages finished [8.96s]`).
- `./test.sh --parallel-workers 4 --packages-select` the four, linters included: `Summary: 4 packages finished [3.03s]`; `colcon test-result`: `Summary: 159 tests, 0 errors, 0 failures, 18 skipped` (the 18 skipped are cppcheck, which skips on Rolling).
- `test_s57_catalog` binary run 25 times: 25/25 passed.
- Mutations on a scratch copy, each failing **both** tests: (a) `0.5*CPLAtof(...)` at `ddfsubfielddefn.cpp:473` → bounds read as 21.525/-35.4/21.5625/-35.3625, so the bounds and the inside-box test fail; (b) `"SLAT"` read as `"NLAT"` at `s57_catalog.cpp:38` → zero-height bounds, no chart selected (`all.size()` 0, `inside.size()` 0); (c) catalogue path misnamed at `s57_catalog.cpp:26` → "found 0 charts" (the fallback scan finds nothing), both fail. Caution for later mutation checks in a package worktree: the first try passed all three because `env.sh` puts the worktree's `install/marine_charts/lib` on `LD_LIBRARY_PATH`, which outranks the build tree's RUNPATH, so the test loaded the unmutated library; rerun with the scratch build directory first on `LD_LIBRARY_PATH` (shown with `ldd`).
- `enc_updater`: before, `Ran 0 tests ... NO TESTS RAN`, exit 5; after, `117 passed`, 0 failures, 0 skipped (no failures to explain).
- `ament_xmllint` on `enc_updater/package.xml` and `marine_charts/package.xml`: no problems. `rosdep resolve ros2node` → `ros-rolling-ros2node`.
- Part 2's packages as they stand: `./build.sh --parallel-workers 4 --packages-up-to s57_grids` → `Summary: 3 packages finished [8.43s]`, against the underlay's `grid_map_ros`; its tests (launch test and lint) `100% tests passed, 0 tests failed out of 6`. `s57_layer` still fails on Rolling for its own unchanged reason, first at `s57_layer/CMakeLists.txt:28` (`ament_target_dependencies`, cause 1), then the rest the issue review listed.
- pre-commit (v4.7.0 in a scratch venv) over every changed file: all hooks pass; `CATALOG.031` is classed binary (`identify`: `binary`), so it is left byte-identical, sha256 unchanged after the run.

### The fixture

`CATALOG.031` and NOAA's `USERAGREEMENT.TXT` are both committed byte for byte from `US5PSMCC.zip` (downloaded 2026-11-24; the agreement is byte-identical to `asv_sim/enc/NOAA_USERAGREEMENT.TXT` in unh_marine_simulation, sha256 `27b503af…599f848f`). The agreement has mixed line endings (one CRLF line) **and** trailing spaces on 42 lines; normalising it would mean `trailing-whitespace` stripping those spaces as well as `mixed-line-ending` rewriting the line endings (both checked: each changes the file's bytes; `end-of-file-fixer` leaves it alone). The words would not change, but the copy would no longer match NOAA's checksum. So it is **excluded** from `trailing-whitespace` and `mixed-line-ending` only, with a comment in the config file, and the README says why. The README gives source URL, download date, sizes and sha256 of both files, says they are NOAA data, not covered by the package's Apache-2.0 licence, redistributed under NOAA's agreement (which permits redistribution and asks that the agreement be shown and NOAA's site referenced), not an official NOAA ENC, not updated, and not for navigation. The repository has no third-party notices file, so no notices entry.

### CI

- Jazzy test step does **not** source `install/setup.bash`. Checked on Rolling in a scratch workspace (`git archive HEAD`, `env -i`, only `/opt/ros/rolling` and the underlay sourced, `AMENT_PREFIX_PATH` without the workspace install): `colcon test --packages-select s57_grids` passed, launch test included (18 results, 0 failures). Also: unh_marine_autonomy's Jazzy job (run 45402341426, 2026-11-24) runs `bathymetry_layer`'s `test_plugin_loading`, the same pluginlib test `s57_layer` has, without sourcing the overlay, and it passed.
- Rolling job rehearsed with its own `run:` blocks taken from the YAML (a script runs each with the step's shell, working directory and job `env`, under `env -i` plus `HOME`, `ROS_DISTRO=rolling` as the `ros:rolling-ros-core` image sets it, a colcon defaults file for 4 workers and `MAKEFLAGS=-j4`), real clones of grid_map at `0ece1b5` and unh_marine_autonomy `rolling`: closure 11 packages (the four, five from unh_marine_autonomy, `grid_map_core`, `grid_map_cmake_helpers`), guard passes; rosdep `--simulate` plus `rosdep check` with no `-r`: "All system dependencies have been satisfied"; `ament_xmllint`: no problems; build 49 s; test 3 s; `Summary: 159 tests, 0 errors, 0 failures, 18 skipped`; "Python tests ran": `117 tests, 0 skipped, 0 errors`. The guard fails (exit 1) when `grid_map_ros` is added to the selection. Not rehearsed: apt update and install, checkout (a `git archive` stood in), `rosdep init`/`update`, and the real `rosdep install` (replaced by `--simulate` and `rosdep check`; the packages are already on this machine). The first rehearsal without `ROS_DISTRO` failed at rosdep, as expected outside the image.
- Jazzy job, what could be checked here: with geographic_info at `f70b81a` and unh_marine_autonomy `jazzy` (`ff6bd70`), the closure is 12 packages (the six, the five, `geodesy`), and all 49 external rosdep keys resolve with `--os=ubuntu:noble --rosdistro=jazzy`.

### Unverified on Jazzy (no Jazzy here; Ubuntu 26.04, GCC 15, GDAL 3.12, no Docker)

The pull request's first CI run is the first Jazzy evidence for all of Part 1. It is also the first CI build anywhere of `s57_to_geotiff` and `enc_updater`, their first cppcheck run (Jazzy runs it, Rolling skips it), the first strict `rosdep install` in this repository, and the first run of the catalogue test on GDAL 3.8.

### Statements for the pull request

- Part 1 of 2 for #52 (`Part of #52`). Part 2 (`s57_grids`, `s57_layer`) is planned again after this merges.
- The vendored ISO 8211 reader (`marine_charts/src/iso8211`) is used for one thing: `S57Catalog` reads the exchange set's `CATALOG.031` to list the chart cells and their bounds. Chart cells are read by GDAL's own OGR S-57 driver, not by this reader.
- The new test reads NOAA's real `CATALOG.031` through that reader and checks the one dataset, its bounds, and that a box inside selects it and boxes clear of each edge do not; three mutations each fail both tests. It does not read a chart cell, does not test GDAL's S-57 driver, and does not cover the catalogue's fallback directory scan.
- The fixture is third-party data: NOAA ENC data, not covered by the package's Apache-2.0 licence, redistributed under NOAA's user agreement (committed beside it), not an official NOAA ENC and not for navigation.
- The Rolling CI job builds and tests four of the six packages (`s57_msgs`, `marine_charts`, `s57_to_geotiff`, `enc_updater`): `s57_grids` needs `grid_map_ros` (which pulls in `nav2_msgs`) and `s57_layer` needs `nav2_costmap_2d`; Nav2 is not released for Rolling and no CI job builds it from source. The Jazzy job builds and tests all six.

### Left alone (for the follow-up issue)

- `marine_charts` public headers include packages the library links `PRIVATE` or not at all and does not export: `include/marine_charts/s57_catalog.h:8` and `s57_dataset.h:7-8` (`geographic_msgs`, `geometry_msgs`), `grid_creation_context.h:4,6,18,32` (`geometry_msgs`, `tf2_ros`, `rclcpp::Logger`); `CMakeLists.txt:49-56` links them `PRIVATE`; `CMakeLists.txt:72` exports neither `tf2_ros` nor `rclcpp`. Consumers name them (the new test does, `CMakeLists.txt:94-101`).
- Deprecated `tf2_ros/buffer.h`: `marine_charts/include/marine_charts/grid_creation_context.h:6`.
- `cmake_minimum_required(VERSION 3.8)` below 3.10: `s57_msgs/CMakeLists.txt:1`, `marine_charts/CMakeLists.txt:1`, `s57_to_geotiff/CMakeLists.txt:1`.
- `-Wparentheses` at `marine_charts/src/s57_catalog.cpp:55` (`while(entry = readdir(dir))`).
- `marine_charts/package.xml:6`: description "TODO: Package description".
- No test reads a real chart cell through GDAL's S-57 driver (`S57Dataset::open`, `marine_charts/src/s57_dataset.cpp:61`; `s57_to_geotiff` `exporter.cpp:488`, `runExport`), or covers the catalogue's fallback scan (`s57_catalog.cpp:49-70`).
- cppcheck skips on Rolling (18 results skipped in the four packages).
- The vendored reader (`marine_charts/src/iso8211/`) is older than upstream GDAL's `frmts/iso8211` (it dates from about GDAL 2.3 to 3.5); compare for hardening fixes.
- For Part 2's plan: `abort_flag_` in `s57_grids` is set at `src/grid_publisher.cpp:148` (cleanup) and never reset (initialised at `src/grid_publisher.h:122`); after a cleanup and a new configure every `S57Dataset::getGrid` returns null at `marine_charts/src/s57_dataset.cpp:177`, and the result is dereferenced in `GridMapRosConverter::toMessage(*grid)` at `grid_publisher.cpp:259` and `:487`. Both distros; not a port cause.

### Differences from the plan and checkpoint

- The user agreement is committed byte for byte and excluded from two hooks rather than LF-normalised, because normalising would also strip trailing spaces and break the match with NOAA's checksum (the checkpoint allowed either; this is the reason).
- The test has the plan's two gtests; the outside-box test checks all four edges (east, west, north, south), not only east and north.
- The workflow names each job's packages once, in a job-level `PACKAGES` variable, so no line exceeds yamllint's 120 characters; the rosdep and guard steps do not source `/opt/ros/<distro>/setup.bash` (they run under `set -u`, and need only colcon and rosdep from the image).
- Commit 2 also changes `.pre-commit-config.yaml` (the exclusion), which the plan's file table did not list.
