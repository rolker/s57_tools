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

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-24 09:25 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: approved

**Branch**: feature/issue-52 at `5f8d9c4`
**Base**: origin/rolling
**Depth**: Deep (reason: CI workflow rewrite plus vendored C++ reader, third-party fixtures and packaging across 4 packages; 18 files, +992 -37)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — no must-fix findings; remaining suggestions can be applied or tracked

Part 1 scope: s57_msgs, marine_charts, s57_to_geotiff, enc_updater, CI on both distros. Checked: `79d64d6` touches only the six `CPL_CVSID` lines; the fixtures are byte-identical to `ENC_ROOT/CATALOG.031` and `ENC_ROOT/USERAGREEMENT.TXT` in NOAA's current `US5PSMCC.zip` (fetched for this review; sha256 `10a94bcf…40da8f`, `27b503af…f848f`) and to unh_marine_simulation's copy of the agreement; the test's expected bounds equal the `.000` CATD bytes, and each of the four clear boxes fails exactly one comparison in `S57Dataset::intersects`; nothing under `test/` is installed. Under `colcon test`, a package's own install prefix is in its test environment (colcon_core `verb/test.py`, "for testing a package include itself"), so `test_s57_catalog` loads `install/marine_charts/lib`, which LD_LIBRARY_PATH puts ahead of the build-tree RUNPATH; in CI and `./build.sh` that copy is installed by the same build, so the fresh library is tested; only a build-tree rebuild without install (the mutation method) is fooled. The same rule lets the `s57_layer` plugin test and the `s57_grids` launch test find their own packages without sourcing the overlay. All five code commits cherry-pick cleanly onto `origin/jazzy` (scratch worktree, removed); none touches `.agent/`. Not verifiable here: anything Jazzy (no Jazzy install). Note: ament_cppcheck skips every cppcheck 2.x unless `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS` is set, Jazzy's 2.13 included, so neither job runs cppcheck (the plan expects Jazzy to).

Gemini findings rejected: cpplint `runtime/string` and missing copyright header in `test_s57_catalog.cpp` (both linters are disabled in `marine_charts/CMakeLists.txt`); unknown names in `$PACKAGES` passing silently (`--packages-up-to` errors on an unknown name).

### Findings
- [ ] (suggestion) README paraphrases NOAA's redistribution condition as "shown with redistributed copies"; the agreement (section 3) says "displayed and accepted by anyone accessing" plus a website reference, and adds full liability for products incorporating ENCs in altered form: quote the conditions verbatim — `marine_charts/test/data/README.md:27`
- [ ] (suggestion) Add a `.gitattributes` `-text` rule for `marine_charts/test/data/enc_root/*`: git treats both fixtures as text (no NUL bytes), so a `core.autocrlf=true` checkout can rewrite the agreement's mixed line endings and break the documented sha256 (Codex and Gemini) — `marine_charts/test/data/README.md:32`
- [ ] (suggestion) The Rolling closure guard sees only packages on disk in `src/`, so its `nav2_.*` branch cannot match a Nav2 dependency (none is cloned); such a dependency would fail later at strict `rosdep install` while Nav2 is unreleased for Rolling, but would pass if Nav2 ever releases. Reword the header claim ("fails the job if the closure ever takes in a Nav2 package") to "Nav2 source", or also check the closure's declared keys (Codex and Gemini) — `.github/workflows/ci.yml:243`

## Checkpoint
**Status**: complete
**When**: 2026-11-24 09:31 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the pre-push review approved round 1 with no must-fix; Jazzy cannot be built on the development machine, so the pushed branch's first run is the first Jazzy evidence, and the first CI build anywhere of two of the six packages)
**After**: publish
**Decision**: publish

Read: the Local Review (Pre-Push) at `5f8d9c4`, verdict approved, ship recommended, three suggestions open: the fixture README paraphrases NOAA's redistribution conditions where it should quote them (`marine_charts/test/data/README.md:27`); a `.gitattributes` rule should keep git from rewriting the two fixtures' line endings on checkout, which would break the recorded checksums (`marine_charts/test/data/README.md:32`); the Rolling job's closure guard can only see packages on disk, so the workflow header should say it guards against Nav2 source entering the build, not against any Nav2 dependency (`.github/workflows/ci.yml:243`). They stay open and go through triage after publish together with what the first CI run shows, so one fix pass serves both. Also from the review, for the pull request text: neither CI job will run cppcheck (it skips on cppcheck 2.x on both distros), and CI tests the freshly built library. The pull request says `Part of #52` (part 1 of 2).

## Integrated Review
**Status**: complete
**When**: 2026-11-24 09:39 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Reviews-read**: none

**PR**: #53 at `9a251c0`
**Sources**: 2 (Local Review (Pre-Push) @ `5f8d9c4`, covers `9a251c0` via bookkeeping; CI run 45415089806)
**Cross-source confirmations**: 0
**CI**: all-pass

No pull-request reviews or comments; no Copilot review or check-run on the head (the only check-runs are the two CI jobs; no reviewer requested). Gemini and Codex ran inside the pre-push review, not on GitHub.

CI run 45415089806 at `9a251c0`, both jobs success, nothing re-run:
- **build-and-test (jazzy)**, 3m45s (last `jazzy` run 32596986676, 2026-08-22, four packages: 3m21s). Strict `rosdep install` (no `-r`): "#All required rosdeps installed successfully" (GDAL `3.8.4+dfsg-3ubuntu3`). Build `Summary: 12 packages finished [2min 9s]` (the six, five from unh_marine_autonomy, `geodesy`). Test, overlay not sourced: `Summary: 6 packages finished [4.33s]`; `colcon test-result`: `Summary: 216 tests, 0 errors, 0 failures, 29 skipped`. `test_s57_catalog` `[  PASSED  ] 2 tests` on GDAL 3.8; `s57_to_geotiff` `test_exporter` 6 passed (its first CI run); `enc_updater` `117 passed in 0.90s`, "Python tests ran": `117 tests, 0 skipped, 0 errors`; `s57_grids` 6/6 (launch test passed); `s57_layer` 7/7 (`test_allow_uncharted` 3, `test_tide_offset` 7, `test_depth_costs` 12, `test_plugin_loading` 1).
- **build-and-test (rolling)**, 3m29s (first run). Guard listed 11 packages (the four, `grid_map_cmake_helpers`, `grid_map_core`, five from unh_marine_autonomy), no match, passed. Strict rosdep: "#All required rosdeps installed successfully" (GDAL 3.12.2). `ament_xmllint` passed. Build `Summary: 11 packages finished [1min 48s]`; test `Summary: 4 packages finished [3.54s]`; `Summary: 159 tests, 0 errors, 0 failures, 18 skipped`; `test_s57_catalog` 2 passed; `117 passed in 0.84s`; "Python tests ran": `117 tests, 0 skipped, 0 errors`. Same numbers as the local rehearsal.
- cppcheck ran on neither: "cppcheck 2.13.0 has known performance issues and therefore will not be used" (Jazzy), 2.19.0 (Rolling), as the pre-push review said. The Implementation entry's "first cppcheck run (Jazzy runs it)" is wrong; the follow-up item "cppcheck skips on Rolling" should say both distros. Only known warnings in this repository's code (`-Wparentheses` at `marine_charts/src/s57_catalog.cpp:55`, the `tf2_ros/buffer.h` deprecation), both already in the follow-up list.

### Findings
- [x] (suggestion, Local Review (Pre-Push) @ `5f8d9c4`) The README's "permits redistribution and asks that the agreement be shown with redistributed copies and that NOAA's web site be referenced" understates the agreement. Section 2: "You may download, use, and redistribute them without restriction ... except as noted herein." Section 3: redistributed ENCs "are NOT considered official NOAA ENCs"; "Copying of the NOAA ENCs to any other server or location for further distribution is discouraged unless the following guidelines are followed: 1) this User Agreement is displayed and accepted by anyone accessing the NOAA ENCs, and 2) a reference to this Web site is included so that anyone accessing the NOAA ENCs is advised of their origin."; and "If these NOAA ENC's are incorporated into any other product in a form other than as provided by NOAA, the producer of that product assumes full liability and must adhere to conditions described in the paragraph above concerning copying." What the committed files meet: the agreement sits byte-identical beside the data and the README points to it (displayed, as in NOAA's own zip); the origin URL is given (reference); the README says not official and not for navigation. "Accepted by anyone accessing" cannot be enforced by a repository, nor by NOAA's own zip; the agreement's preamble makes use itself the acceptance ("Your use ... shall constitute your agreement"), and that travels with the file. The two files are unaltered, but the exchange set is partial (the cell files are omitted), so the liability paragraph may apply; it adds liability, not a prohibition. This is ordinary for redistributed public NOAA data: unh_marine_simulation's `asv_sim/enc` commits three NOAA cells with a byte-identical `NOAA_USERAGREEMENT.TXT` and a README that paraphrases more loosely. Not, in itself, a reason to prefer a synthetic catalogue (that remains the owner's veto). Fix: replace the paraphrase with the three quotations above, note that copying is "discouraged unless", not forbidden, and that the files are unaltered but the exchange set is partial — `marine_charts/test/data/README.md:27`
- [x] (nit, Local Review (Pre-Push) @ `5f8d9c4`) The Rolling guard sees only packages on disk: its `nav2_.*` alternative can match only Nav2 source, never a Nav2 dependency declared by the closure (`grid_map_ros` would still be caught, since the grid_map clone has it on disk). Today a declared Nav2 key would fail at strict rosdep while Nav2 is unreleased on Rolling; once released, it would pass silently. Smallest fix: reword the header (`.github/workflows/ci.yml:13-15`) to "takes in Nav2 source, grid_map_ros or bathymetry_layer" (two lines). Fuller fix, about four lines in the same step: `rosdep keys --ignore-src --from-paths $paths | grep -E '^nav2_'` and fail on a match. No false positives: the step is Rolling-only, so Jazzy's legitimate Nav2 keys never reach it, and none of the 11 closure packages declares a `nav2_` key (checked: the four `package.xml` files, the five from unh_marine_autonomy `rolling`, and `grid_map_core`/`grid_map_cmake_helpers` depend on `eigen` and `ament_cmake_core` only) — `.github/workflows/ci.yml:243`

### False positives
- (Local Review (Pre-Push) @ `5f8d9c4`, Codex and Gemini) "`core.autocrlf=true` checkout can rewrite the agreement's mixed line endings and break the sha256" — checked in a scratch clone at `9a251c0`: with `core.autocrlf=true`, and with `* text=auto eol=crlf`, both fixtures check out with the recorded sha256 (`git ls-files --eol`: `CATALOG.031` is `-text`, the agreement `mixed`; git's auto conversion leaves a blob holding any CRLF untouched). Only an explicit `text eol=crlf` attribute changed the agreement (sha256 `2ad2620c…`). A repository `.gitattributes` `marine_charts/test/data/enc_root/* -text` would guard against that one case (it outranks a global attributes file); optional, not needed for the stated failure.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 09:39 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (two valid findings are open and each has a small, named fix; merging past open findings is the owner's answer only)
**After**: findings
**Decision**: address

Read: the Integrated Review of pull request 53 at `9a251c0`: two valid findings, one false positive, CI all-pass (run 45415089806: Jazzy builds 12 packages and tests all six of this repository, 216 tests, 0 errors, 0 failures, with a strict rosdep install and without sourcing the overlay; the catalogue test passes on GDAL 3.8.4; `enc_updater` 117 passed; Rolling builds 11 packages, 159 tests, 0 failures; cppcheck runs on neither). Addressed: (1) the fixture README quotes NOAA's agreement instead of paraphrasing it: redistribution "without restriction ... except as noted herein"; redistributed ENCs are not official; copying for further distribution "is discouraged unless" the agreement is displayed and accepted by anyone accessing the ENCs and a reference to NOAA's site is included; the liability sentence for products incorporating ENCs in a form other than as provided; and it says that the two files are unaltered but the exchange set is partial (the catalogue without its cells), and that a repository can display the agreement but cannot enforce acceptance. One docs commit for backport. (2) the Rolling job's guard also checks the closure's declared rosdep keys for `nav2_`, and the header says what the guard does; one CI commit for backport. The `.gitattributes` suggestion is a false positive as stated (the triage tested a checkout with `core.autocrlf=true`: both fixtures keep their checksums) and is not acted on. The implementation entry's "first cppcheck run" is wrong; the pull request text already says cppcheck runs on neither job.

## Implementation
**Status**: complete
**When**: 2026-11-24 09:43 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**PR**: #53 at `04c9f6f`
**Addressed**: Integrated Review at `9a251c0` (2026-11-24 09:39 -05:00)
**Commits**: `504b756` (docs), `04c9f6f` (CI); records `5298c4c` (boxes) and this entry. Neither docs nor CI commit touches `.agent/`; both end "Part of #52. Backport to jazzy."

### Actions
- [x] Fixture README quotes NOAA's agreement instead of paraphrasing it: section 2's redistribution sentence, section 3's "NOT considered official NOAA ENCs" sentence, the "discouraged unless" guidelines sentence, the "in a form other than as provided by NOAA" liability sentence, and the preamble's "Your use ... shall constitute your agreement" sentence (each checked verbatim against `enc_root/USERAGREEMENT.TXT`, whitespace-normalised; the preamble's "by any this Agreement" kept as in the file and marked so). Then in the README's own words: both files unaltered byte for byte; the exchange set partial (catalogue without `US5PSMCC.000`-`.002` and the four `.TXT` files it lists); agreement displayed beside the data and NOAA's site referenced; a repository cannot make a reader accept it, and by the preamble use constitutes agreement. Source URL, date, checksums, not-Apache-2.0, not official, not for navigation kept. Fixtures unchanged: `CATALOG.031` `10a94bcf…40da8f`, `USERAGREEMENT.TXT` `27b503af…599f848f` — `marine_charts/test/data/README.md:27`
- [x] Rolling guard also checks the closure's declared rosdep keys: `rosdep keys --ignore-src --from-paths <closure paths>`, fails on any `^nav2_` key and names the declaring package (from `colcon list`); on-disk check kept; header says the guard catches Nav2 source, `grid_map_ros` or `bathymetry_layer` entering the build, and any `nav2_*` key declared in the closure. `rosdep keys` needs no `setup.bash`; without `ROS_DISTRO` it only warns "ROS_PYTHON_VERSION is unset" and gives the same keys; the image sets `ROS_DISTRO`, as the rosdep install step relies on — `.github/workflows/ci.yml:243`

### Verification
- Guard step's own `run:` block (scratch workspace, `env -i` plus `HOME`, `ROS_DISTRO=rolling`, grid_map `0ece1b5`, unh_marine_autonomy `rolling`): as is, 11 packages, exit 0 (closure keys contain no `nav2_`); `<depend>nav2_msgs</depend>` added to `s57_to_geotiff/package.xml`, exit 1, "nav2_msgs, declared by s57_to_geotiff (src/s57_tools/s57_to_geotiff)"; `grid_map_ros` added to `PACKAGES`, exit 1, "grid_map_ros" / "the closure takes in the packages above, which this job must not build".
- Rehearsal of the Rolling job's `run:` blocks at HEAD (4 workers, `MAKEFLAGS=-j4`; rosdep install as `--simulate` plus `rosdep check`): guard exit 0; "All system dependencies have been satisfied"; `ament_xmllint` "No problems found"; build `Summary: 11 packages finished [48.5s]`; test `Summary: 4 packages finished [2.75s]`, `117 passed`; `Summary: 159 tests, 0 errors, 0 failures, 18 skipped`; Python tests ran `117 tests, 0 skipped, 0 errors`. Not rehearsed: apt, checkout, rosdep init/update, the real install.
- pre-commit 4.7.0 (scratch venv) on both changed files: all hooks pass; shellcheck on the guard block: clean.
- Cherry-pick trial onto `origin/jazzy` in a scratch worktree (removed): `79d64d6 2d5efe0 ae81fb9 961e1a7 e954dbc 504b756 04c9f6f`, all clean.
- `.gitattributes` (triage false positive): not acted on.

## Local Review
**Status**: complete
**When**: 2026-11-24 09:50 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Dispatch**: resumed (agent a526ab9be98da966f, resume 1 of 3)
**Verdict**: approved

**PR**: #53 at `cda7509`
**Depth**: Deep (reason: round 2 narrow check per handoff, round 1 had no must-fix: the three round-1 suggestions and the fix diff `e954dbc..04c9f6f` outside `.agent/`)
**Must-fix**: 0 | **Suggestions**: 0
**Cross-model**: skipped (narrow check per handoff, owner's standing rule)

Checked:
- `504b756`: all five quotations match `USERAGREEMENT.TXT` word for word once whitespace is normalised (checked by script). They cover every redistribution term: section 2, both section 3 paragraphs on copying, the liability sentence, and the preamble. The README's own statements (unaltered files, a partial exchange set, the agreement displayed and the site referenced, acceptance not enforceable) neither leave out nor soften anything. The fixtures' sha256 still match the README: `10a94bcf…40da8f` and `27b503af…599f848f`.
- `04c9f6f`: the guard runs after `rosdep init`/`update`, but `rosdep keys` reads only manifests and does not need them. `ROS_DISTRO` comes from the image. Rehearsal from the step's own `run:` block, nothing sourced, under `set -euo pipefail`:
  - as is: exit 0;
  - `nav2_msgs` added to `enc_updater/package.xml`: exit 1, printing `nav2_msgs, declared by enc_updater (src/s57_tools/enc_updater)`.
- `.gitattributes` (round-1 suggestion): I accept the triage ruling of false positive. A `core.autocrlf=true` clone leaves both fixtures' sha256 unchanged, and `git status` stays clean after touching the agreement.
- Neither new commit touches `.agent/`. All seven code, docs and CI commits cherry-pick cleanly, in order, onto `origin/jazzy` `d4f1283` (scratch worktree, removed).
- CI run 45417304066 on `cda7509`: success.
  - Rolling job: `Summary: 11 packages finished [1min 49s]`; `Summary: 159 tests, 0 errors, 0 failures, 18 skipped`; enc_updater 117 tests. The guard listed the 11-package closure, found no `nav2_*` key and passed.
  - Jazzy job: `Summary: 12 packages finished [2min 13s]`; `Summary: 216 tests, 0 errors, 0 failures, 29 skipped`; enc_updater 117 tests.

### Findings
- [ ] No issues found. LGTM.

## Integrated Review
**Status**: complete
**When**: 2026-11-24 09:59 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Dispatch**: resumed (agent a04baf4b92dbadaf5, resume 1 of 3)
**Reviews-read**: none

**PR**: #53 at `87bfe25`
**Sources**: 2 (Local Review @ `cda7509`, which covers `87bfe25` through a bookkeeping-only commit; CI run 45418748263)
**Cross-source confirmations**: 0
**CI**: all-pass

Round 2. Since my entry at `9a251c0`: the fix commits `504b756` (README) and `04c9f6f` (guard), the records commits `5298c4c` and `cda7509`, and `87bfe25`, which adds the Local Review (approved, nothing open, no cross-model reviewers). The `review_progress.sh sources` helper returns no open local findings. The pre-push review at `5f8d9c4` is now stale (`ci.yml` changed since), and its three items were ruled on in my previous entry: two fixed, one a false positive.

Checked the fixes myself at `87bfe25`:
- README (`marine_charts/test/data/README.md:24-64`): the five quotations match `USERAGREEMENT.TXT`, and the README's own statements match what my previous entry said the files do and do not meet. The fixtures' sha256 are unchanged (`10a94bcf…40da8f`, `27b503af…599f848f`).
- Guard (`.github/workflows/ci.yml:243-278`): the on-disk check is kept. `rosdep keys --ignore-src` runs over the closure paths and fails on `^nav2_`, naming the package that declares the key. The header now says what the guard does. The step is in the Rolling job only, so Jazzy's Nav2 keys never reach it.

GitHub: no reviews and no conversation comments on the pull request. The only check-runs are the two CI jobs; there is no Copilot review or check-run.

CI run 45418748263 on `87bfe25`, both jobs success:
- **build-and-test (jazzy)**, 5m46s (the runner was slower: the build took 3min 26s against 2min 9s in round 1, on the same code as `cda7509`). Strict rosdep: "#All required rosdeps installed successfully". Build `Summary: 12 packages finished [3min 26s]`. Test `Summary: 6 packages finished [5.94s]`. `Summary: 216 tests, 0 errors, 0 failures, 29 skipped`. `test_s57_catalog` passed. `117 passed in 1.41s`. The "Python tests ran" step printed `117 tests, 0 skipped, 0 errors`.
- **build-and-test (rolling)**, 3m20s. The changed guard listed the 11-package closure (the four packages, `grid_map_cmake_helpers`, `grid_map_core` and five from unh_marine_autonomy), reported no declared `nav2_` key and passed. Strict rosdep: all installed. Build `Summary: 11 packages finished [1min 43s]`. Test `Summary: 4 packages finished [3.48s]`. `Summary: 159 tests, 0 errors, 0 failures, 18 skipped`. `test_s57_catalog` passed. `117 passed in 0.81s`. The "Python tests ran" step printed `117 tests, 0 skipped, 0 errors`.
- cppcheck was skipped on both, as in round 1. The pull request text already says so.

### Findings
- [x] No open findings. Round 1's two valid findings are fixed in `504b756` and `04c9f6f` and checked above. The `.gitattributes` false positive stands.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 10:00 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: owner
**After**: merge
**Decision**: merge

The owner's answers, 2026-11-24: "Merge when CI is green (Recommended)" for pull request 53 (part 1), with the NOAA catalogue as the test fixture (he was shown what NOAA's agreement asks and the alternative of a synthetic catalogue); "Backport right after (Recommended)" for the hand backport to `jazzy` of the seven code, docs and CI commits; "Open it (Recommended)" for the follow-up issue in this repository. The host merges only after the CI run on the pushed records passes. Part 2 (`s57_grids`, `s57_layer`) is planned at the plan checkpoint after this merge.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 10:17 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (part 1 merged as pull request 53 at `ce1eb2b`; the plan details part 1 and only outlines part 2, so it is refreshed rather than implemented as it stands)
**After**: plan
**Decision**: revise

Part 1 (the four Nav2-free packages and CI on both distros) is merged; its backport to `jazzy` is pull request 55. The plan is refreshed for part 2: `s57_grids` and `s57_layer`, checked on Rolling by hand (owner: no CI job builds Nav2 source), on Jazzy by the existing CI job, which already builds and tests both. To carry into the refreshed plan: the causes the issue review and part 1's plan found in `s57_layer` (1, 8, 2, 6 with 15 `declareParameter` calls, 7, 13 in tests, and the new one: `nav2::LifecycleNode::create_client` returns Nav2's wrapper; candidate 17 on rolker/unh_marine_autonomy#415); how the two earlier costmap layers were ported (`bathymetry_layer`, `ais_layer`: a local parameter helper kept rather than Nav2's newer API, tracked in rolker/unh_marine_autonomy#428; parameter-override and `initialize()` tests proved by mutation); cause 11 for `grid_map_ros`, `grid_map_msgs` and `tf2_ros`; the pre-existing defect in `s57_grids`' `grid_publisher` (`abort_flag_` set at cleanup and never reset, so after a cleanup and a new configure every grid request returns null and is then dereferenced): the plan says what the facts are and puts the choice (fix in the port, or track) to the host; `s57_layer` is on the navigation path (it marks charted hazards in the costmap), so the plan's validation statement says how the port is shown not to change what it marks. Left-alone items go to rolker/s57_tools#54 as a comment drafted for the owner.

## Plan Authored
**Status**: complete
**When**: 2026-11-24 10:46 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Dispatch**: resumed (agent a6b786e00becb0946, resume 1 of 3)
**Plan**: `.agent/work-plans/issue-52/plan.md` at `007cfce`

Part 1 recorded as delivered (pull requests 53 and 55). Part 2 to implement now. On a scratch copy, seven changes make `s57_layer` build on Rolling and pass its 23 gtests: causes 1 and 8 (geodesy and grid_map_ros variables, 1.0.7 floor), 2, 6 (the local helper, a third copy), new cause 17 (free `rclcpp::create_client`), 7 and 13. `s57_grids` builds and passes unchanged; causes 9, 12, 14 and 16 were checked and no change is needed; cause 11 adds keys to both manifests. There are two new tests (overrides, and client-to-costmap), because a hard-coded prefix and a misnamed service both pass the existing 23. A Rolling costmap hand run against GDAL's own reading of the chart was piloted. The plan puts two defects that exist without the port to the host: the `abort_flag_` crash in `s57_grids`, reproduced (recommend fixing it in this port), and tile mis-addressing in `s57_layer`'s `worldToTile`, which leaves charted land free in a rolling window near its origin (recommend its own issue now). No workflow change. The draft cause 17 comment for #415 is in the plan.

## Plan Review
**Status**: complete
**When**: 2026-11-24 10:58 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: ready

**Issue**: #52 — Port s57_tools to ROS 2 Rolling
**Plan**: `.agent/work-plans/issue-52/plan.md` at `007cfce`
**Branch**: `feature/issue-52`
**Serves**: none written down (s57_tools has no goals or design document); for the workspace, the Project11 Rolling port, rolker/unh_marine_autonomy#415. Checked: the plan's line says the same.

Reviewed: Part 2 (`s57_grids`, `s57_layer`). Part 1 is a delivered record and was not re-reviewed.

What I ran (session scratch copy of `007cfce`, my own colcon workspace, Rolling, DDS domain 187 checked empty, `DISPLAY` unset; removed afterwards):
- Changes a to g applied by hand: `--packages-up-to s57_layer` builds (4 packages). `colcon test` on `s57_grids s57_layer`: the 23 existing gtests pass (12, 7, 3, 1) and `s57_grids` 6/6 ctest. Without my own scratch test this is the plan's 57 results with 11 skipped.
- `diff` of the ported `s57_layer.cpp` against the branch: only line 10, the helper lambda, line 124 and the subscription at 655 change. No line of `get_cost_from_grid`, `generateTile`, `updateCosts` or `worldToTile` is touched.
- A scratch gtest (`#define private public`; a hand-filled all-land grid; `ldd` shows the scratch `libs57_layer.so` is the one loaded) for the tile defect, listed under finding 1. I also ran a `charts.unsurveyed_cost = 77` override: it is declared as an integer and the member reads 77 on Rolling.
- Rolling CI guard re-run locally with cause 11's keys added to both manifests: the four-package closure is unchanged, its keys have no `nav2_`, and `grid_map_ros` and `grid_map_msgs` are outside it. `rosdep resolve` for every added key: all resolve on noble/jazzy. On resolute/rolling all resolve except `grid_map_core`, `grid_map_msgs` and `grid_map_ros`.
- `gh api` reads of upstream: rclcpp `jazzy` `create_client.hpp` and `lifecycle_node_impl.hpp` (the member calls the free function with the same QoS default and the same callback group), nav2 `jazzy` `layer.cpp` (`declareParameter` declares `name_ + "." + param` when it is not already declared), and grid_map `jazzy` `grid_map_ros/CMakeLists.txt` (it exports only include directories and libraries, no target, as 2.5.0 does).

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Two packages, 11 commits plus one optional, one fix per commit. The two pre-existing defects are put to the host, not folded in. |
| Issue alignment | Good | Every cause the issue review listed for these two packages is handled: 1, 8, 2, 6, 7, 13, 11 and new 17. Causes 9, 12 and 14 were checked and need no change. The two tests the issue review asked for (overrides, and `initialize()` with the client and subscription) are planned and mutation-checked. |
| File targeting | Good | Matches what my scratch port touched. Only the abort fix touches `s57_grids` sources. |
| Consequences | Needs work | The tile defect's risk statement names the wrong origin (finding 1). The abort fix leaves another null path (finding 2). |
| Principle alignment | Good | Test what breaks: two mutations that pass the 23 tests today become caught. Verify before claiming: the Jazzy and Rolling limits are stated. Minor: `ament_cmake_ros` is declared in a later commit than the one that first uses it (finding 3). |
| ADR compliance | Good | ADR-0013: entries and records are kept out of the code commits. |
| ROS conventions | Good | Cause 17's free `rclcpp::create_client` gives the same QoS (`ServicesQoS`) and the same callback group (default, `nullptr`) as Jazzy's member, and it compiles on both distros. The CMake-variable linking of `grid_map_ros` and `geodesy` is forced: neither exports a target, on Jazzy or Rolling. It matches `s57_grids/CMakeLists.txt:30-34` and bathymetry_layer. |

### Findings

1. **[Consequences]** (major, wording that goes to the owner) The tile defect is confirmed. The plan's account of when it happens is wrong, and its age is understated.
   - **What `x_origin_` is.** `x_origin_` and `y_origin_` are set only in `matchSize()` (`s57_layer.cpp:145-150`). That runs from `onInitialize` and from `LayeredCostmap::resizeMap`. `Costmap2DROS::on_configure` resizes the map to the `origin_x`/`origin_y` parameters (default 0) before the plugins are initialised (`costmap_2d_ros.cpp:149-152`), and a rolling window's `updateOrigin` never calls `matchSize`. So the reference point is the costmap's configured origin, by default the global frame's origin. It is not "where its costmap window started".
   - **When it triggers.** A rolling window is affected whenever its lower-left corner is west or south of that point: the robot within half a window east or north of the frame's origin, or anywhere west or south of it. On ben, `map` is anchored at the first position fix unless a map origin is configured (mru_transform README), so the defect is present from the first cycle.
   - **Static costmaps.** Never affected, because the origin captured is the window's own origin (scratch test 5).
   - **Age.** The truncation came with tiles in `181c034` (2021-09-21). `9938802` (2026-02-19) only renamed the members. The plan's line numbers (`:174-175`) are the post-port ones; on the branch the lines are `:159-160`.
   - Correct the plan's "What it means for the vehicle" text before the owner relies on it.
2. **[Consequences]** (minor) The abort fix makes this one cleanup path safe. It does not touch the dereference itself.
   - `checkForNewGrids` still dereferences an unchecked `grid` at `grid_publisher.cpp:259` and `:284`.
   - `getGrid` also returns null when `open()` fails (`s57_dataset.cpp:151-153`), for example a cell file replaced or removed after the catalogue scan, and when `llToMap` fails. Either aborts the node the same way.
   - The planned state test pins the `abort_flag_ = false` reset: deleting it fails the test. It cannot pin `pending_dataset_grids_.clear()`, because without a chart cell no job is pending at cleanup. Nor does it pin the crash, as the plan says.
   - Either add `if(!grid)` handling at the two dereferences in the same commit (the owner's call: a third change), or list the other null paths explicitly in the #54 comment.
3. **[Principle alignment]** (minor) Commit 5 introduces `ament_add_ros_isolated_gtest`, but `ament_cmake_ros` is declared only in commit 10. With strict rosdep, declare it in commit 5 so the commit stands alone. This also keeps the two-step push from depending on what `ros:jazzy-ros-core` happens to ship.
4. **[Principle alignment]** (minor) Both new tests need bounded polling, not fixed sleeps:
   - test 1's `get_client_names_and_types_by_node`, because graph discovery is asynchronous even in one process;
   - test 2's `service_is_ready`, the reply, the latched grid and the costmap.
   Test 2's window should be a static `LayeredCostmap` whose origin is at or below the grid, otherwise the tile defect can fail it. Say both in the plan.
5. **[Scope]** (note) The two-step push can be done on the Part 2 pull request into `rolling` rather than on the backport. The `ci.yml` Jazzy job runs on pull requests into `rolling` and builds `s57_layer`, and the Rolling job does not build it. That yields the same evidence (test 1 against Nav2's own `declareParameter`) one pull request earlier.

### Summary

The port is sound and verified as far as this machine allows. With changes a to g, both packages build on Rolling and the 23 existing tests pass. The marking functions are untouched. Cause 17's fix keeps Jazzy's client semantics, and the planned tests close the two gaps that compile-only evidence leaves. Ready for implementation once the tile-defect risk text (finding 1) is corrected for the owner. Findings 2 to 5 can be taken up during implementation.

### Recommended Actions

- [ ] Rewrite the plan's tile-defect consequence (finding 1): reference point, trigger, static case, age `181c034`.
- [ ] Abort fix: decide about the `if(!grid)` guard, or name the other null paths for #54 (finding 2).
- [ ] Move `ament_cmake_ros` into commit 5 (finding 3).
- [ ] Specify the polling and the window placement for tests 1 and 2 (finding 4).
- [ ] Consider doing the two-step push on the `rolling` pull request (finding 5).

### Tile addressing (for the host and the owner)

- **(a) Which positions, and what follows.** `worldToTile` truncates toward zero, so for `(x - x_origin_)/(resolution_*tile_size_)` in (-1, 0) it returns 0 instead of -1, and every negative tile index is one too high.
  - A tile's bounds come from its id (`x_origin_ + id*T` at `:314`, `:341`), and `updateCosts` copies by an offset computed from that id (`:469-474`). The data that is copied lands in the right place: no two regions share a tile, and nothing is stale data from somewhere else.
  - What goes wrong is the loop range. `updateCosts` starts at `worldToTile(mapToWorld(min_i))`, a cell centre, so whenever the window's minimum is below the origin the westmost (or southmost) partial tile is never iterated or generated. Its cells receive nothing from the layer.
  - Those cells are left at the master grid's reset value: FREE_SPACE with `track_unknown_space: false`, NO_INFORMATION with `true`. With `allow_uncharted: false` they are also not marked NO_INFORMATION. The strip is up to one tile wide (`tile_size_` cells, default 100), at the window's west and/or south edge.
  - It is always "free or unknown where it should be lethal". It is never "lethal where it should be free".
- **(b) When.** Only rolling windows are affected: the window's lower-left corner west or south of the costmap's configured `origin_x`/`origin_y`, by default the global frame's origin. On ben's `map_tide` (anchored at the first fix unless configured), that is:
  - local costmap (350 m, 0.5 m): the strip is up to 50 m wide and begins 175 m from the vessel;
  - global costmap (5000 m, 1 m, `track_unknown_space: true`): up to 100 m of unknown begins 2500 m away.
  Because the strip is always at the window's edge, the cells under and near the vessel stay correct while half the window exceeds one tile. The exposure is plans made through the strip (global path, coverage), which are then corrected only as the strip slides inward. Inflation from the missing land is missing too.
- **(c) Reproduced.** Scratch gtest, 10x10 cells at 1 m:
  - rolling window, robot at (0, 0), tile 100: 25 of 100 cells lethal, should be 100 (the west 5 columns and south 5 rows free);
  - the same window uncharted with `allow_uncharted: false`: 25 of 100 cells NO_INFORMATION;
  - rolling window at (-20, -20) with tile 4: 81 of 100;
  - rolling window at (+20, +20): 100 of 100;
  - static window with origin -5: 100 of 100.
  With `std::floor` in a scratch copy, all five give 100. That copy was discarded.
- **(d) Why the 23 tests missed it.** Every test that calls `updateCosts` uses a non-rolling 10x10 `LayeredCostmap` at origin 0 (`test_allow_uncharted.cpp:52`, `test_depth_costs.cpp:358`, `test_tide_offset.cpp:96` etc.), so the window minimum always equals `x_origin_`. No test moves the origin.
- **(e) Age.** Since `181c034` (2021-09-21). It is identical on `jazzy` (`git diff origin/jazzy` on the file is empty).
- **Order.** The facts support a separate issue and pull request (B). They do not support gating it on Part 2.
  - The fix lines and the port's lines are disjoint, so either order cherry-picks cleanly, and the port's "no marking line changed" claim holds either way.
  - The `ci.yml` Jazzy job runs on pull requests into `rolling` and builds `s57_layer`, so a fix pull request can be checked on Jazzy by CI before Part 2. On Rolling it can only be checked by hand after Part 2.
  - The defect is live on the deployed Jazzy code today. Landing it first or in parallel shortens the exposure and lets Part 2's hand run show full agreement.
  - The scratch test above is the shape of its regression test.

### `abort_flag_` (for the host and the owner)

- **Mechanism, confirmed by reading.** The flag is set at `grid_publisher.cpp:148` and never reset (`grid_publisher.h:122`). `getGrid` then returns null at `s57_dataset.cpp:177`, and `checkForNewGrids` dereferences it at `:259` (and at `:284` when `publish_costmaps` is on) if the label was requested. `:487` is guarded. I did not re-run the crash.
- **Who can trigger it.** Only an operator: `ros2 lifecycle set`, or ros2launch_gui's Cleanup entry (`launch_description_widget.py:39`). No lifecycle manager lists `s57_grids` in ben (`navigation_launch.py:46-56`). `s57_grids_launch.py` transitions once and respawns the node unconfigured.
- **The costmap afterwards.**
  - Grids already received keep marking.
  - Tiles of a chart listed in the last reply but never delivered get NO_INFORMATION, whatever `allow_uncharted` says (`all_charts_available` false).
  - Areas the window reaches later are treated as uncharted: with `allow_uncharted: true` they are left free on the local costmap and unknown on the global one.
  - The layer logs an unthrottled WARN, "The get datasets service is not ready", every cycle. It does not log an ERROR.
- **The proposed test.** It pins the reset (mutation-checked), not the `clear()` and not the crash (finding 2).

## Checkpoint
**Status**: complete
**When**: 2026-11-24 10:58 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: owner
**After**: plan
**Decision**: proceed

The Plan Review at `007cfce` found part 2's plan ready, with five findings, and confirmed two pre-existing defects on the hazard path. The owner answered both on 2026-11-24. **The tile-addressing defect in `s57_layer` (`worldToTile`, `s57_layer.cpp:157-162`): "Own issue now, fix in parallel (Recommended)".** It is rolker/s57_tools#56, driven on its own branch; **part 2 does not touch `worldToTile` or any other marking line**, and its hand run on Rolling states the defect where it shows (a rolling window reaching west or south of the costmap's configured origin) instead of working round it silently. **The `abort_flag_` defect in `s57_grids`: "Fix in Part 2, with null checks (Recommended)".** So commit 12 is in: reset the flag on configure and drop aborted jobs on cleanup, and check for a missing grid at the two dereferences (`grid_publisher.cpp:259` and `:284`), so a chart that fails to open cannot abort the node either; with a test that pins the reset and, if achievable without a chart cell, the null path; its own commit, backported. Review findings, applied: (1) the plan's account of the tile defect is superseded by the review's (the reference point is the costmap's configured origin; rolling windows only; the line dates from `181c034`, 2021); nothing in part 2's records repeats the old wording. (3) `ament_cmake_ros` is declared in the commit that first uses `ament_add_ros_isolated_gtest`. (4) Both new tests use bounded polling, not fixed sleeps, and the client-to-costmap test uses a static costmap with its origin at or below the grid, so the tile defect cannot fail it. (5) The two-step push is done on this pull request into `rolling`: the branch is first pushed up to the overrides test, so the Jazzy job runs that test against Nav2's own `declareParameter`, then the rest. Host decisions: a third copy of the parameter helper, as the two earlier layers have; the before/after costmap comparison on the owner's Jazzy machine is offered, not required.

## Implementation
**Status**: complete
**When**: 2026-11-24 11:38 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Branch**: feature/issue-52 at `2a9d081`

Part 2 of 2 (`s57_grids`, `s57_layer`), per the plan and the Checkpoint of 10:58 (commit 12 in, with null checks; tile addressing left to #56). Twelve code commits, none touching `.agent/`, each ending "Part of #52. Backport to jazzy.", then one records commit (`2a9d081`, hand-run scripts and raw results under `hand-runs/`, not backported):

1. `f14c0a0` s57_layer: link geodesy and grid_map_ros through their CMake variables (#415 causes 1 and 8)
2. `98182eb` s57_layer: state the geodesy 1.0.7 floor that geodesy/ecef.h needs (#415 cause 8)
3. `3b63f8b` s57_layer: include tf2/utils.hpp (#415 cause 2)
4. `5d44873` s57_layer: include Nav2's lifecycle node from where each distro has it, in the tests (#415 cause 13)
5. `354e860` s57_layer: test that every parameter override is read under the layer's name. **The first push stops here.** `ament_cmake_ros` is declared in this commit.
6. `1eeec53` s57_layer: declare parameters without Layer::declareParameter (#415 cause 6)
7. `ac29320` s57_layer: create the get_datasets client with rclcpp's free function (#415 cause 17)
8. `02d7617` s57_layer: create grid subscriptions with rclcpp's free function (#415 cause 7)
9. `a91812a` s57_layer: test that initialize()'s client reaches get_datasets and a grid reaches the costmap
10. `91a4d4e` s57_layer: declare the packages it uses (#415 cause 11)
11. `fa07b2c` s57_grids: declare the packages it uses (#415 cause 11)
12. `8ab87bf` s57_grids: start each configuration with grid jobs enabled, and skip a chart that gives no grid

On Rolling `s57_layer` first compiles at commit 8, so commits 1 to 7 were not built one by one here; the sequence was built and tested where it first compiles and at the head. All twelve cherry-pick cleanly, in order, onto `origin/jazzy` `89b552d` (scratch worktree, removed). Commit 5 on Jazzy, by reading `origin/jazzy` of navigation2 (`c704655`) and rclcpp/geometry2 `jazzy`: the test uses `nav2_util::LifecycleNode(name, "", options)`, `Layer::isEnabled`, `initialize(..., WeakPtr, nullptr)`, `get_client_names_and_types_by_node` and `tf2_ros/buffer.hpp`, all present; Jazzy's `Layer::declareParameter` declares `name_ + "." + param` unless declared, and its node's `create_client` is rclcpp_lifecycle's member, so the overrides test should pass there unchanged. Not run on Jazzy.

### Verification (Rolling, this machine; tree `8ab87bf`; DDS domains 163 and 177, each checked empty with `ros2 node list --no-daemon`; `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; `DISPLAY` unset; everything started was stopped)

- **Underlay** (`/home/roland/project11/rolling/underlay`): navigation2 install built 2026-11-19 from `dcffa939` (clone 08:51, `libnav2_costmap_2d_core.so` 12:09); the checkout is `9113449e` (fast-forward on 11-23), and `git diff dcffa939 9113449e -- nav2_ros_common nav2_costmap_2d nav2_util` is empty. grid_map `0ece1b5` (2.5.0) and geographic_info `f70b81a` (geodesy 1.0.7), checkout and install alike (one clone each in the reflog).
- **Clean build** from empty `build/ install/ log/`: `MAKEFLAGS=-j4 ./build.sh --parallel-workers 4 --packages-up-to s57_grids s57_layer --cmake-args -DBUILD_TESTING=ON` → `Summary: 4 packages finished [32.0s]`. Warnings in the two packages: only the `tf2_ros/buffer.h` (7) and `transform_listener.h` (4) header deprecations, from the old tests and `grid_publisher.h`.
- **Full test run**: `./test.sh --parallel-workers 4 --packages-select s57_grids s57_layer` → `s57_grids` `100% tests passed, 0 tests failed out of 7` (launch test, `test_reconfigure` 3 tests, lint); `s57_layer` `100% tests passed, 0 tests failed out of 8` (`test_allow_uncharted` 3, `test_tide_offset` 7, `test_depth_costs` 12, `test_initialize` 9, `test_plugin_loading` 1, lint); `colcon test-result`: `Summary: 73 tests, 0 errors, 0 failures, 13 skipped` (cppcheck).
- **Repeats, each run a separate process**: `test_initialize` 50/50, `test_reconfigure` 50/50; `test_depth_costs`, `test_tide_offset`, `test_allow_uncharted`, `test_plugin_loading` 10/10 each; launch test 20/20 as separate `ctest -R test_node_launch` runs, and `--repeat until-fail:20 -V` shows 20 passes. No failure in any run.
- **rosdep**: every added key resolves with `--os=ubuntu:noble --rosdistro=jazzy` (`geographic_msgs`, `grid_map_core`, `grid_map_msgs`, `grid_map_ros`, `tf2`, `tf2_ros`, `ament_cmake_ros`, `geometry_msgs`, `nav_msgs`, `tf2_geometry_msgs`, `launch`, `launch_ros`, `rclpy`, `rcl_interfaces`); with `--os=ubuntu:resolute --rosdistro=rolling` all resolve except `grid_map_core`, `grid_map_msgs`, `grid_map_ros` ("no rosdep rule": source-only on Rolling, from the underlay). `rosdep check --ignore-src --from-paths s57_grids s57_layer` (no `-r`): "All system dependencies have been satisfied". `ament_xmllint` on both manifests: "No problems found".
- **Part 1's Rolling CI steps, re-run** from the workflow's own `run:` blocks (extracted from `ci.yml`), `env -i` plus `HOME`, `PATH`, `ROS_DISTRO=rolling`, the job's `PACKAGES`, on a `git archive` of HEAD with grid_map at `0ece1b5` and unh_marine_autonomy `rolling` `0ca4a58` (local clones): guard exit 0, closure the same 11 packages as Part 1 (the four, `grid_map_cmake_helpers`, `grid_map_core`, five from unh_marine_autonomy), no declared `nav2_` key; strict rosdep (`--simulate` plus `rosdep check`, no `-r`) "All system dependencies have been satisfied".
- pre-commit 4.7.0 (scratch venv) on every changed file and the records: all hooks pass.

### Mutations (scratch copy of HEAD built outside the source tree; each mutated test run with the scratch build directory first on `LD_LIBRARY_PATH`, `ldd` showing the scratch `libs57_layer.so` loaded each time; raw output in `hand-runs/mutations_*.txt`)

Baseline: `test_initialize` 9/9, old tests 12, 7, 3, 1 pass, `test_reconfigure` 3/3.
- Helper hard-codes `"chart_layer."`: `test_initialize` fails 8 of 9 (all but `ReadsEnabledFalse`); the 23 old tests all pass, as the plan found.
- Each read-back misnamed (`name_+".<p>_x"`): `enabled` fails 5 (`ReadsLethalDecisionOverrides`, `ReadsAllowUnchartedOverride`, `ReadsTideOverrides`, `DefaultsEnableTheLayer`, `ChartReachesTheCostmap`); `minimum_depth`, `maximum_caution_depth`, `overhead_clearance`, `unsurveyed_cost`, `depth_costs` each fail `ReadsLethalDecisionOverrides`; `allow_uncharted` fails `ReadsAllowUnchartedOverride`; `chart_datum_frame`, `sea_surface_frame`, `tide_invalidate_threshold` each fail `ReadsTideOverrides`; `s57_grids_namespace` fails `ClientUsesTheOverriddenNamespace` and `ChartReachesTheCostmap`; `get_datasets_service` fails `ClientUsesTheOverriddenServiceName`. `update_timeout`, `tile_size`, `buffer_fraction`: nothing fails (declared-only, as planned).
- Service misnamed (`service_name += "_x"`): fails `ClientUsesTheOverriddenServiceName`, `ClientUsesTheOverriddenNamespace`, `ChartReachesTheCostmap`; the 23 old tests pass.
- Grid topic without the namespace prefix: fails `ChartReachesTheCostmap`; the 23 old tests pass.
- `abort_flag_ = false` deleted: fails `ConfigureAfterCleanupClearsTheAbortFlag`. `pending_dataset_grids_.clear()` deleted: fails `CleanupDropsAbortedJobs`. Missing-grid check disabled: the test process aborts (exit -6) with `__shared_ptr_deref ... Assertion '__p != nullptr' failed`.
- `unsurveyed_cost` declared from an `unsigned char`: declared as an integer, and 77 reads back as 77 on Rolling (no failure).

### The `s57_grids` fix (commit 12)

- `on_configure` clears `abort_flag_`; `on_cleanup` drops the jobs it waited on; `checkForNewGrids` checks the grid once, before both dereferences (`toMessage(*grid)` and `toOccupancyGrid(*grid, ...)`, `grid_publisher.cpp:259` and `:284` on the base), logs an error naming the chart and skips it: no publisher made, `requested_grids_to_publish_` and the start time erased, so the next `get_datasets` that lists the chart tries again. One check rather than two because both uses are in the same block and a skipped chart must not create the empty `grid_publishers_` entry that marks a chart as done.
- `pending_dataset_grids_`, `tf_buffer_` and `abort_flag_` are made `protected` **in place** (access specifiers only), so declaration and destruction order are unchanged; moving them would have destroyed `tf_buffer_` before the `tf_listener_` that references it.
- **Reproduction on Rolling** (`hand-runs/repro_abort.py`, NOAA cells US5PSMCC and US5PSMCD from unh_marine_simulation `55926d0` `asv_sim/enc`, as an ENC_ROOT of per-cell folders in the scratchpad; static earth→map ENU at 43.07 N, 70.75 W):
  - before (binary built at `fa07b2c`): configure, activate, request west: "Grid ready: US5PSMCC.000 (1.74826 s from request to publish)"; deactivate, cleanup, configure, activate (all `True`), request east: node exited with -6, "`_Tp* std::__shared_ptr_deref(_Tp*) [with _Tp = grid_map::GridMap]: Assertion '__p != nullptr' failed.`"
  - control, same binary, no cycle: both "Grid ready", node alive.
  - after (`8ab87bf`): both "Grid ready: US5PSMCC.000 (1.74795 s …)" and "Grid ready: US5PSMCD.000 (1.84657 s …)" across the cycle; node alive, exit 0 on SIGTERM.
  - missing cell (ENC_ROOT = marine_charts' catalogue fixture, `minimum_scale -1`): before, exit -6 with the same assertion; after, "No grid for chart US5PSMCC.000 (it could not be opened or placed in the map frame); not publishing it." and the node alive.
- **What `test_reconfigure` pins**: the reset (configure, cleanup, configure through the lifecycle API); the dropping of an aborted job (a finished future holding no grid, put in by hand, because no job can be in flight without a chart cell); and the missing-grid path end to end in-process: Part 1's fixture is a catalogue whose one chart has no cell file, the test serves the request (`minimum_scale -1`, because an unopenable cell's scale reads 0), and the node logs the error on `/rosout` and the process lives. **What it does not pin**: the crash after a real cleanup cycle with a chart that opens (needs a chart cell); a chart that opens but cannot be placed in the map frame (`llToMap` failing); the precompute path, which lists charts by position alone and reaches the same grid job. It reads the fixture through `../marine_charts/test/data/enc_root` (`s57_grids/CMakeLists.txt:113`), so it needs the whole repository checked out; without it the test fails, not passes.

### The costmap hand run (`hand-runs/costmap_run.py`, `oracle.py`; tree `8ab87bf`)

Setup: `nav2_costmap_2d` (navigation2 install `dcffa939`) with only `s57_layer::S57Layer` ("chart": `minimum_depth` 2, `maximum_caution_depth` 6, `s57_grids_namespace` "/"), 1000 × 1000 cells at 1 m, `track_unknown_space` false; `s57_grids` (`8ab87bf`) serving US5PSMCC and US5PSMCD; static earth→map ENU at 43.07 N, 70.75 W and map→base_link at the robot position. The oracle is GDAL's own S-57 reading of the same cells: a sample is "lethal" if in LNDARE or in DEPARE/DRGARE with DRVAL1 < 2, "deep" if in DEPARE/DRGARE with DRVAL1 ≥ 6 and not land. Every 10th cell each way is sampled (10,000 per window); a disagreement is "at a boundary" when the oracle's class changes within 7.5 m (the grid is 3.75 m). Both cells were received in every run that overlaps them.

| Window | Oracle lethal → costmap lethal | Oracle deep → costmap free/ramp | Disagreements not at a boundary |
|---|---|---|---|
| static, origin (1500, 0) | 5289 of 5549 | 3005 of 3025 | 222 land samples **free** in x 2000–2037 (the cell seam, below); 8 deep samples lethal in one cluster near (1900, 850), the three checked under charted pontoons (PONTON) |
| rolling, robot (1000, 1000), window [500, 1500]² | 7181 of 7191 | 2260 of 2310 | 19 deep samples lethal along one line from (905, 775) to (785, 995), the three checked on a charted obstruction (OBSTRN) |
| rolling, robot (2500, 500), window [2000, 3000] × [0, 1000] | 3961 of 4223 | 4415 of 4415 | 222 land samples **free** in x 2000–2037 (the cell seam) |
| rolling, robot (0, 0), window [-500, 500]² | 8100 of 10000 (all land) | none | **1900 land samples free, all in the west strip x [-500, -400) and south strip y [-500, -400): 190,000 cells, the tile-addressing defect, rolker/s57_tools#56** |

PONTON and OBSTRN are rasterised lethal by `s57_grids` (`marine_charts/src/s57_dataset.cpp`, cases 95 and 86); the oracle does not model them, so those are correct marks. The rest of the disagreements are at chart boundaries (lethal → ramp, deep → lethal within 7.5 m).

- **#56 shows where expected**: only the window reaching west and south of the costmap's configured origin (0, 0), and only its west and south strip, one tile (100 m) wide. Everywhere else in that window agrees with the chart. Not worked round: the tests use a static costmap with its origin at the grid's south-west.
- **New, pre-existing: the cell seam.** US5PSMCC and US5PSMCD meet at 70.725 W, x ≈ 2036 m. The layer's tile [2000, 2100] has its centre (2050) east of the seam, so `grid_map::SubmapGeometry` fails for the west cell (`getSubmapInformation` returns false when the requested centre is outside the grid, grid_map `GridMapMath.cpp:338-341`, the same on grid_map's `jazzy` branch), `generateTile` (`s57_layer.cpp:425`) skips it, and the strip of the tile on the west cell's side, about 36 m by 1000 m, gets no chart data: charted land left free in a static and a rolling window alike. Any chart edge inside a tile whose centre is outside that chart behaves the same; overlapping coarser charts would fill it with coarser data. `s57_layer.cpp` is identical on `origin/jazzy` and `origin/rolling`, so this is on Jazzy too by reading (not run). **A grounding risk like #56; recommend its own issue.** Not touched here: it is in `generateTile`.

### Differences from the plan and the checkpoint

- Commit 12's subject adds "and skip a chart that gives no grid" (the owner's null checks); one check guards both dereferences (reason above). Its test also pins `pending_dataset_grids_.clear()` (by a hand-made aborted job), which the plan said could not be pinned; it pins the null path through the catalogue fixture. `ament_cmake_ros` and `rcl_interfaces` are declared as test dependencies of `s57_grids` in commit 12, where they are first used.
- Commit 12 makes three `GridPublisher` members protected in place instead of only `abort_flag_` (the test needs the pending jobs and the tf buffer too).
- The overrides test (commit 5) has 8 cases, not one: `enabled` is read both ways (true differs from Nav2's starting false; false differs from the default) plus a defaults case. `depth_costs` is overridden to false, which suppresses the ramp, so the test switches the protected member back on by hand after checking it, to read `minimum_depth`, `maximum_caution_depth` and the tide.
- Test 2's chart grid is 110 m square at a quarter-cell offset, not 20 m square: a 20 m grid did not reach the costmap, because of the tile-centre behaviour above and a grid_map precision edge case (a request clipped to a grid edge at map coordinate 0 loses its epsilon, `GridMapMath.cpp:262-267`), and an integer-aligned grid wrote lethal cells into column 0 of the next row (the edge-wrap defect below). Both are documented in the test; neither is worked round in the layer.
- The new tests include `tf2_ros/buffer.hpp`, which exists on Jazzy too; the old tests keep `buffer.h`.
- Plan's repeat counts were 50 for the new tests: done (the handoff asked for at least 30). One extra hand-run window (robot (2500, 500)) shows the seam in a rolling window.
- `nav2_ros_common` is not declared: the tests include it only where `__has_include` finds it, and Jazzy has no such key.

### Left alone (for the comment on rolker/s57_tools#54)

- `s57_grids`: `GridCreationContext`'s constructor calls `lookupTransform(map, "earth")` (`marine_charts/src/grid_creation_context.cpp:14`) from the `checkForNewGrids` timer (`s57_grids/src/grid_publisher.cpp:236`) with no `try`: a missing transform throws out of the timer (not reproduced; the new test waits for the transform first).
- `s57_grids`: `on_cleanup` (`grid_publisher.cpp:150-170`) keeps `grid_publishers_`, `costmap_publishers_`, `dataset_grids_`, `requested_grids_`, `requested_grids_to_publish_`, `grid_request_start_times_` and `did_precompute_`.
- `s57_grids`: `s57_grids_launch.py:18-19` respawns the node, but its `LifecycleTransition` (`:22`) runs once, so a respawned node stays unconfigured.
- `s57_grids`: after commit 12 a chart that cannot be opened is retried, with an unthrottled ERROR, on every `get_datasets` that lists it (`grid_publisher.cpp:259`).
- `s57_layer`: `unsurveyed_cost` is declared from an `unsigned char` (`s57_layer.cpp:60-61`, `s57_layer.h:97`): it is an integer parameter, and the override reads back correctly on Rolling, but a value outside 0 to 255 wraps silently (no range check).
- `s57_layer`, **new**: chart data dropped at chart edges inside a tile whose centre is outside the chart (`s57_layer.cpp:425`; grid_map `GridMapMath.cpp:338-341`), shown above at the US5PSMCC/US5PSMCD seam. Recommend its own issue.
- `s57_layer`, **new**: `generateTile` (`s57_layer.cpp:438-455`) uses `Costmap2D::worldToMapEnforceBounds`, which returns `size` (one past the last cell) when a coordinate equals the tile's east or north edge exactly (navigation2 `costmap_2d.cpp:335-346` on Rolling, `:328-339` on Jazzy). A grid cell edge exactly on the tile's east edge then writes into column 0 of the next row (seen in a test draft: 9 water cells lethal); exactly on the north edge, the write goes past the end of the tile's array. Needs exact floating-point alignment, which real chart grids rarely give.
- Tests include the deprecated `tf2_ros/buffer.h`: `s57_layer/test/test_allow_uncharted.cpp:10`, `test_tide_offset.cpp:14`, `test_depth_costs.cpp:14`; and `s57_grids/src/grid_publisher.h:15-16` (`buffer.h`, `transform_listener.h`).
- Both manifests are format 2 (`s57_grids/package.xml:1`, `s57_layer/package.xml:1`): `ament_xmllint` checks well-formedness only.
- `s57_layer/package.xml:21` declares `marine_charts`, and `CMakeLists.txt:71` exports it, though nothing in the package includes it.
- cppcheck skips on both distros (13 results skipped here).

### Draft comment for rolker/unh_marine_autonomy#415 (cause 17)

> **Cause 17: Nav2's lifecycle node wraps `create_client`.** On Rolling, `nav2::LifecycleNode::create_client<SrvT>(name, use_internal_executor = false)` returns `nav2::ServiceClient<SrvT>::SharedPtr` (`nav2_ros_common/lifecycle_node.hpp:211`, built by `nav2::interfaces::create_client`, `interface_factories.hpp:255`), not `rclcpp::Client<SrvT>::SharedPtr`. Code that stores the result in an `rclcpp::Client` pointer fails to compile (`no match for 'operator='` between the two). Jazzy's `nav2_util::LifecycleNode` inherits rclcpp_lifecycle's member, `create_client(name, qos = rclcpp::ServicesQoS(), group = nullptr)`.
>
> Remedy that builds on both distros and keeps the same client: `rclcpp::create_client<SrvT>(node->get_node_base_interface(), node->get_node_graph_interface(), node->get_node_services_interface(), name)`, the free function Jazzy's member calls, with the same defaults on both (rclcpp `create_client.hpp`). It is the client counterpart of cause 7. Found in `s57_layer` (rolker/s57_tools#52, part 2, commit `ac29320`), which keeps `service_is_ready`, `async_send_request` and `prune_requests_older_than` on an `rclcpp::Client`.
>
> Compiling is not evidence the client still reaches its service: `s57_layer`'s 23 existing tests pass with the service name deliberately misnamed. Its new `test_initialize` checks that the ROS graph lists exactly one `GetDatasets` client on the layer's own node under the configured name, and serves `get_datasets` in-process to follow a chart from the request, through the reply's latched grid subscription, to lethal cells in the costmap. The misnamed service fails three of its cases. Checked on Rolling; on Jazzy by CI.
>
> Where else: any Nav2 plugin or node that calls `create_client` on Nav2's node type and keeps an `rclcpp::Client`. Search for `create_client<` in code built on Nav2 before trusting a clean build.

### Validation statement for the pull request

- **No marking line changed.** `git diff origin/rolling -- s57_layer/src/s57_layer.cpp` has four hunks: line 10 (`tf2/utils.hpp`, `3b63f8b`), the parameter helper inserted in `onInitialize` (+11, `1eeec53`), the client creation in `onInitialize` (+6 −1, `ac29320`), and the subscription creation in `getDatasetsCallback` (+6 −1, `02d7617`). `s57_layer.h` is unchanged. The bodies of `get_cost_from_grid`, `generateTile`, `updateCosts`, `worldToTile`, `updateBounds`, `matchSize`, `reset`, `worldToLatLon`, `llToWorld` and `mapGridCallback` are byte-identical to `origin/rolling` (compared by extraction).
- **What the two new tests pin.** The overrides test: all 15 parameters declared under the layer's own name (not `chart_layer.`, not bare) and read back through what the layer does: the lethal decisions of `get_cost_from_grid`, the uncharted handling of `updateCosts`, the tide offset of `updateBounds`, and the `get_datasets` client's name on the ROS graph. `tile_size`, `update_timeout` and `buffer_fraction` are checked as declared only. The client-to-costmap test: the request reaches an in-process `get_datasets`, the layer's node subscribes once to `<namespace>/datasets/<topic>`, and the chart's land becomes `LETHAL_OBSTACLE` and its water `FREE_SPACE` in a static costmap. A hard-coded `chart_layer.` prefix, a misnamed service and a wrong topic prefix, each of which passes the 23 existing tests, fail them; so does a misnamed read of each of the 12 observable parameters.
- **`s57_grids`.** A cleanup and a new configure no longer leave grid jobs aborted, aborted jobs are dropped, and a chart that gives no grid is logged and skipped instead of aborting the node. Reproduced before and shown gone after with NOAA cells on Rolling; the reset, the drop and the missing-cell path are pinned by `test_reconfigure`; the crash after a real cleanup with an openable chart is not.
- **The hand run on Rolling** (one pair of NOAA cells, one layer configuration, four 1 km windows; no vessel, no planner, no other layer or inflation using the costmap): the ported layer fetches both charts through Rolling's Nav2 and marks land and shallow water lethal where GDAL's own reading of the cells says they are, except in two places, both pre-existing. One is the tile-addressing defect, rolker/s57_tools#56, known and not changed here: 190,000 cells free in the west and south strip of the window that reaches west and south of the configured origin. The other is a newly found gap at the seam between the two cells: a strip about 36 m wide left free, in a static and a rolling window alike, because `generateTile` drops a chart that does not contain the tile's centre.
- **Jazzy**: the existing CI job (which builds and tests both packages with strict rosdep) is the check; nothing was run on Jazzy locally. The branch is pushed first up to `354e860`, so the Jazzy job runs the overrides test against Nav2's own `Layer::declareParameter` before the helper replaces it.

### Unverified

Everything on Jazzy (build, tests, the abort reproduction, the hand run, the seam gap): by reading only. A before/after costmap is not possible on Rolling (the pre-port tree does not compile); the deadpool comparison remains on offer. The seam gap with overlapping coarser charts present (here there is none). The `llToMap`-failure null path. The crash-level test with a real cleanup cycle.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-24 11:51 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: approved

**Branch**: feature/issue-52 at `28924ab` (code head `8ab87bf`)
**Base**: origin/rolling
**Depth**: Deep (reason: 31 files, about 2,500 lines; s57_layer is on the hazard-marking path and is backported to the deployed Jazzy code)
**Must-fix**: 0 | **Suggestions**: 4
**Round**: 1 | **Ship**: recommended (no must-fix findings; the suggestions can be applied or tracked)

Part 2 (`s57_grids`, `s57_layer`). Specialists: governance, plan drift, a Claude adversarial subagent, and Gemini and Codex (both completed). Static analysis was not run separately: the implementer's pre-commit run is recorded.

### What I ran or read
- **No marking line changed.** I read the whole of `git diff origin/rolling...HEAD -- s57_layer/src`. `s57_layer.h` is unchanged. `s57_layer.cpp` has four hunks: the include, the helper lambda, the client and the subscription. `get_cost_from_grid`, `generateTile`, `updateCosts`, `updateBounds`, `worldToTile` and `matchSize` are untouched.
- **The helper matches Jazzy.** Nav2 `origin/jazzy` (`c704655`), `layer.cpp:76-87` and `node_utils.hpp:95-104`: `Layer::declareParameter` declares `name_+"."+param` with the default descriptor when it is not already declared. Its only other effect, `local_params_.insert`, is never read on Jazzy. The `ParameterValue(...)` expressions are unchanged, so `unsurveyed_cost` is still an integer, from the `unsigned char` promoted to `int`. Rolling `nav2::LifecycleNode` does not wrap `declare_parameter` or `has_parameter`.
- **The free functions match Jazzy.**
  - Client: Jazzy's `LifecycleNode::create_client` (`lifecycle_node_impl.hpp:137-145`) calls the same free function with the same node interfaces, `ServicesQoS` and a null group.
  - Subscription: Jazzy's `create_subscription` member calls `rclcpp::create_subscription(*this, ...)` with default options, and `create_subscription.hpp:176` takes `NodeT &`.
  - On Rolling, Nav2's wrapper only adds parameter QoS overrides and passes the callback group through. Bypassing it changes no activation, executor or callback group.
- **`s57_grids` threading.**
  - `main.cpp` uses a single-threaded executor, so the lifecycle transitions, the timer and the services never run concurrently.
  - `on_cleanup` waits on every job before `clear()`, and the reset in `on_configure` can only run after that. No worker can still be reading the flag, so the reset is race-free.
  - A healthy chart takes the unchanged branch.
  - A chart skipped for having no grid is retried only when the layer next sends a request, which happens when its window moves past the buffer (`s57_layer.cpp:238-313`). Until then, that chart's tiles stay incomplete and NO_INFORMATION. That fails safe. Before the fix, the node aborted.
- **Mutations of my own**, built in a scratch workspace from `git archive`. `ldd` showed the scratch `libs57_layer.so`, with its directory first on `LD_LIBRARY_PATH`:
  - grid subscription with volatile QoS `rclcpp::QoS(1)`: `ChartReachesTheCostmap` fails after 10 s;
  - null-grid branch without `done_grids.push_back`: `ChartThatCannotBeOpenedDoesNotAbortTheNode` fails at `:175`.
- **Repeats under load**: `ctest --repeat until-fail:20` for `test_initialize` and for `test_reconfigure` (isolated runner, `ROS_DOMAIN_ID` 193 checked empty), alongside a `make -j4` loop with load average 6 to 9: 20 of 20 each.
  - An earlier attempt that started the load from the same shell failed in the runner with `ModuleNotFoundError: ament_cmake_test` before any test ran. Re-run from a separate shell, it passed. I attribute it to how I ran it, not to the tests.
- **Jazzy, by reading.**
  - `tf2/utils.hpp`, `tf2_ros/buffer.hpp` and `static_transform_broadcaster.hpp` exist on geometry2 `jazzy`.
  - `ament_add_ros_isolated_gtest` and its `domain_coordinator` runner exist on ament_cmake_ros `jazzy`.
  - `get_subscriptions_info_by_topic` and `get_node_*_interface` exist on Jazzy's `LifecycleNode`.
  - grid_map `origin/jazzy` `GridMapMath.cpp` and `SubmapGeometry` are identical to `0ece1b5`.
  - `354e860` declares `ament_cmake_ros`, and its test uses only APIs present on Jazzy.
  - `ci.yml`: the Jazzy job runs on pull requests into `rolling` and builds and tests both packages with strict rosdep. The Rolling job's `PACKAGES` and its guard leave both out.
- **Commits.** None of the 12 code commits touches `.agent/`. Each has one subject, one cause, and the trailer "Part of #52. Backport to jazzy.".

### Pre-existing `generateTile` defects (scratch ASan executable compiling `s57_layer.cpp` itself; not findings of this diff)
- **Seam.** Reproduced in a static costmap at origin 1000. A land chart covering the west 36.25 m or 49.25 m of a tile: 0 of 3,430 and 0 of 4,704 land cells are lethal. A chart with its seam at 50.25 m or 64.25 m, so that it covers the tile's centre, paints every land cell. The gap is up to just under half a tile on whichever side does not hold the centre: 50 m on the global costmap, 25 m on the local one.
  - **A second mechanism near map x = 0 or y = 0.** At origin 0, a tile corner at coordinate 0 that is clipped to a grid edge loses grid_map's epsilon (`GridMapMath.cpp:262-267`). The chart is then dropped even though it covers the tile's centre. With the seam inside tile [0, 100], both charts were dropped and the whole tile was empty.
- **Edge.** Reproduced with a grid cell edge exactly on a tile edge.
  - East edge: 99 lethal cells written into column 0 of the next row.
  - North edge: the read one row past the 10,000-byte array always happens (ASan). The write happens only where that byte reads 255. Real chart grids are placed from lat/lon, so the alignment needs bit-exact coincidence.
- The committed `ChartReachesTheCostmap` avoids all three on purpose (110 m grid, quarter-cell offset, no clipped corner at 0) and says so. It asserts nothing false.

### Cross-model
- **Gemini, rejected:**
  - `.hpp` headers missing on Jazzy: they exist.
  - The `/rosout` race: no other ERROR is logged on that logger along that path.
  - Defaults of the 4-argument `create_client` on Jazzy: they exist.
- **Pre-existing, outside the diff:** Gemini's null node and future exceptions; Codex 1, the TF throw at a request, already listed for #54; Gemini 9 and Codex 4, CMake export.
- **Codex 2 and 3** are pre-existing. Their consequences go under suggestion 2.

### Findings
- [ ] (suggestion) Two comments in commit 12 say more than the code does. A job that finished before the cleanup does hold a grid, and it is dropped too; it is rebuilt on the next request. The null branch's "or when a cleanup aborted the job" can no longer happen, because cleanup clears the pending jobs. — `s57_grids/src/grid_publisher.cpp:163-164`, `:253-256`
- [ ] (suggestion) Add the consequences of state kept across a cleanup to the #54 comment:
  - `requested_grids_` survives the cleanup, so a reconfigure with a different `enc_root` while a label is queued dereferences a null dataset (`grid_publisher.cpp:230-231`);
  - after a `map_frame` or resolution change, latched grids are served stale;
  - after a cleanup and configure cycle, a chart that was aborted is regenerated only when the layer's window next moves. — `s57_grids/src/grid_publisher.cpp:150-171`
- [ ] (suggestion) `TEST_ENC_ROOT` reads the sibling package's source fixture. That works in this repository and in CI, but not in a standalone or bloom test build. Note it for #54. — `s57_grids/CMakeLists.txt:113`
- [ ] (suggestion, the owner's) The new issue for the seam gap should include the map-axis variant. The edge defect is an out-of-bounds read every time and a conditional write. Both are pre-existing. — `s57_layer/src/s57_layer.cpp:425`, `:438-455`

## Checkpoint
**Status**: complete
**When**: 2026-11-24 11:53 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (pre-push review round 1 approved part 2 with no must-fix; three outside reviewers ran)
**After**: publish
**Decision**: publish

Read: the Local Review (Pre-Push) at `8ab87bf`, approved, four suggestions. Published in two steps, as the plan checkpoint set: the branch is first pushed up to `354e860` and the pull request opened there, so the Jazzy job runs the parameter-overrides test against Nav2's own `Layer::declareParameter`; the rest follows once that run is in. Suggestion 1 (two comments in commit 12 that say more than the code does) is not deferred: it is fixed in the fix pass after the pull request's review, with whatever that review finds. Suggestions 2 and 3 go into the comment on rolker/s57_tools#54 at merge. Suggestion 4, and the review's confirmation of two more pre-existing `generateTile` defects (chart data dropped where a tile's centre is outside the chart's grid; a read and conditional write one past a row when a grid cell edge falls exactly on a tile edge), go to the owner as issue text; neither is changed here.

## Integrated Review
**Status**: complete
**When**: 2026-11-24 12:06 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)
**Reviews-read**: none

**PR**: #58 at `454ae59`
**Sources**: 2 (Local Review (Pre-Push) @ `28924ab` covering the head via bookkeeping, CI rollup); 0 pull-request reviews, 0 inline comments, 0 conversation comments
**Cross-source confirmations**: 0
**CI**: all-pass

CI at `454ae59`, run 45443691212: `build-and-test (jazzy)` and `build-and-test (rolling)` SUCCESS. Jazzy: `Summary: 232 tests, 0 errors, 0 failures, 31 skipped`; `s57_layer` 8/8 CTest entries (`test_initialize` `[  PASSED  ] 9 tests.`); `s57_grids` 7/7 (`test_reconfigure` `[  PASSED  ] 3 tests.`, launch test passed); strict `rosdep install --ignore-src -y --from-paths $paths` (no skip keys) ended `#All required rosdeps installed successfully`. Rolling: `PACKAGES: s57_msgs marine_charts s57_to_geotiff enc_updater`, its closure guard passed, `Summary: 159 tests, 0 errors, 0 failures, 18 skipped`; neither `s57_grids` nor `s57_layer` is built there. Run 45442344940 at `354e860`: Jazzy `test_initialize` `[  PASSED  ] 8 tests.` (4/8 Passed) against Nav2's own `Layer::declareParameter`, `226 tests, 0 errors, 0 failures, 30 skipped`. No pull-request review, inline comment or conversation comment exists; the pre-push review's verdict was approved, no must-fix.

### Findings
- [x] (suggestion, Local Review (Pre-Push) @ `28924ab`, decided: fix before merge) Two comments in commit `8ab87bf` say more than the code does: a job that finished before the cleanup holds a grid and is dropped too (rebuilt on the next request), and "or when a cleanup aborted the job" can no longer happen because cleanup clears pending jobs — `s57_grids/src/grid_publisher.cpp:163-164`, `:253-256`
- [x] (suggestion, Local Review (Pre-Push) @ `28924ab`, settled by the owner: goes into a comment on #54 at merge, not a code change) State kept across a cleanup: `requested_grids_` survives, a different `enc_root` with a queued label dereferences a null dataset; stale latched grids after a frame or resolution change; an aborted chart is regenerated only when the layer's window next moves — `s57_grids/src/grid_publisher.cpp:150-171` (deferred: routed, not fixed: goes into the comment on rolker/s57_tools#54 at merge)
- [x] (suggestion, Local Review (Pre-Push) @ `28924ab`, settled by the owner: goes into a comment on #54 at merge, not a code change) `TEST_ENC_ROOT` reads the sibling package's source fixture; fails in a standalone or bloom test build — `s57_grids/CMakeLists.txt:113` (deferred: routed, not fixed: goes into the comment on rolker/s57_tools#54 at merge)
- [x] (suggestion, Local Review (Pre-Push) @ `28924ab`, settled by the owner: carried by #59 (seam gap, includes the map-axis variant) and the #54 list (edge read and conditional write)) pre-existing `generateTile` defects, no change in this pull request — `s57_layer/src/s57_layer.cpp:425`, `:438-455` (deferred: routed, not fixed: carried by rolker/s57_tools#59 and the #54 list)

### False positives
- none (no external review comments to dismiss)

### Pull request body
- "Results of both runs are added below." — still a placeholder; the host adds the two runs (`354e860` run 45442344940, `454ae59` run 45443691212) above.
- "Review. ... One suggestion ... is corrected in this pull request before merge." — true only once the fix pass lands; the other three suggestions are routed to #54 and #59, which the body already lists.
- No statement found that the records contradict: twelve code commits, no Rolling CI job for these two packages, Jazzy CI the only Jazzy evidence, and the Rolling guard all match the runs.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 12:07 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (one open item, comment wording, already set to be fixed before merge)
**After**: findings
**Decision**: address

Read: the Integrated Review for pull request 58 at `454ae59`: no reviews on the pull request, CI green on both jobs. The fix pass corrects the two comments in `s57_grids/src/grid_publisher.cpp` (`:163-164`, `:253-256`) and nothing else. The entry's three other boxes are routed, not fix-pass work, and the fix pass ticks them as routed: state kept across a cleanup and the `TEST_ENC_ROOT` source path go into the comment on rolker/s57_tools#54 at merge; the `generateTile` defects are rolker/s57_tools#59 (the chart-seam gap, opened on the owner's yes, with the map-axis variant in its text) and the #54 list (the read past a row).

## Implementation
**Status**: complete
**When**: 2026-11-24 12:07 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)

**PR**: #58 at `d331930`
**Addressed**: Integrated Review at `454ae59` (2026-11-24 12:06 -05:00)
**Commits**: d331930

### Actions
- [x] Two comments corrected, comments only, no code line changed; `s57_grids` rebuilt (`--parallel-workers 4`) — `s57_grids/src/grid_publisher.cpp:163-165`, `:254-256`
- [x] State kept across a cleanup — `grid_publisher.cpp:150-171` (routed, not fixed: goes into the comment on #54 at merge)
- [x] `TEST_ENC_ROOT` source fixture path — `s57_grids/CMakeLists.txt:113` (routed, not fixed: comment on #54 at merge)
- [x] pre-existing `generateTile` defects — `s57_layer/src/s57_layer.cpp:425`, `:438-455` (routed, not fixed: #59 and the #54 list)

## Local Review
**Status**: complete
**When**: 2026-11-24 12:08 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)
**Verdict**: approved

**PR**: #58 at `74d2467`
**Depth**: Light (reason: narrow re-review of one comment-only commit, d331930, per handoff; no cross-model or adversarial reviewers)
**Must-fix**: 0 | **Suggestions**: 0

### Findings
- [ ] No issues found. LGTM.

Checked: `git diff 454ae59..74d2467 -- . ':!.agent'` changes two comment blocks in `s57_grids/src/grid_publisher.cpp` and nothing else. on_cleanup sets abort_flag_, waits on every valid future, then clears the whole map, so finished jobs are dropped too; grid_publishers_ lacks their labels, so the next request rebuilds. The null branch is reached only by getGrid's two null returns: open() failing, or llToMap failing (an abort also returns null, but cleanup clears the futures before any tick reads them, so the branch never sees it). Commit d331930 touches no .agent file; message ends "Part of #52. Backport to jazzy.". CI at review time: build-and-test (jazzy) and (rolling) both pending. No reviews or comments on the PR since the triage.

## Integrated Review
**Status**: complete
**When**: 2026-11-24 12:16 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)
**Reviews-read**: none

**PR**: #58 at `813c836`
**Sources**: 2 (Local Review @ `74d2467` covering the head via bookkeeping, CI rollup); 0 pull-request reviews, 0 inline comments, 0 conversation comments
**Cross-source confirmations**: 0
**CI**: failures-noted

The four settled items of the previous Integrated Review (at `454ae59`) stay settled. The newest Local Review (at `74d2467`) approved the comment-only commit d331930 with no finding. No review, inline comment or conversation comment exists on the pull request.

CI. Run 45445136398 (head `74d2467`): `build-and-test (rolling)` passed; `build-and-test (jazzy)` FAILED: `2/7 Test #2: test_reconfigure .................***Timeout  60.01 sec`, `Finished <<< s57_grids [1min 2s]	[ with test failures ]`, `Summary: 230 tests, 1 error, 1 failure, 31 skipped`. Everything the test printed, in full: `Running with ROS_DOMAIN_ID 1`, `Running main() from /opt/ros/jazzy/src/gtest_vendor/src/gtest_main.cc`, `[==========] Running 3 tests from 1 test suite.`, `[----------] 3 tests from ReconfigureTest`, `[ RUN      ] ReconfigureTest.ConfigureAfterCleanupClearsTheAbortFlag`, `found 1 charts` (17:12:21.57), then nothing until the timeout kill at 17:13:21.4. No `[       OK ]`, no second `found 1 charts` (a second `configure()` would print it again). Parallel at that moment: colcon had finished marine_charts, s57_msgs and s57_to_geotiff; the `enc_updater` package's 117 pytest cases ran from 17:12:20.8 to 17:12:22.1, overlapping the first second of the test; no other package was running during the other 59 s. Run 45443691212 (head `454ae59`, only comment changes since): same test passed in 0.45 s, `[  PASSED  ] 3 tests.`. Head `813c836` adds only progress.md; its run 45446251629 was still in progress at this triage, so CI at the head is unknown.

### Findings
- [x] (must-fix, CI rollup) Intermittent hang of `test_reconfigure` on Jazzy, test added by this pull request for a defect fixed by this pull request; it must be diagnosed in the fix pass, not re-run away, and the pull request is not mergeable on a green re-run. The fix pass must establish (1) where it hangs: reproduce with `timeout`/`gdb -batch -ex "thread apply all bt"` or a loop of the test binary under load (`stress`/parallel copies, 200+ iterations, ideally in the Jazzy CI image or on deadpool; 50/50 and 20/20 on Rolling prove nothing about Jazzy), and take thread backtraces of the hung process; (2) whether it is the test's own waiting (the first case has no wait: it calls `configure()`, `cleanup()`, `configure()`, so a test-side wait is not a candidate there; `waitFor` and `spin_until_future_complete` are bounded to 10 s in the other cases) or inside the node (`on_cleanup`, `on_configure`, the destructor, a worker thread); (3) whether it is a test-only defect or a deadlock or unbounded wait in `s57_grids` that a vessel could also hit (a lifecycle configure followed shortly by cleanup or shutdown); (4) the fix and a test that makes the failure deterministic or at least a loop that no longer hangs. Hypotheses, not established: (a) most likely, a race in `tf2_ros::TransformListener` between construction and destruction: `on_configure` creates it with its own spinner thread (`s57_grids/src/grid_publisher.cpp:119`; tf2_ros starts `std::thread([&]{executor_->spin();})` right after construction), and `on_cleanup` resets it (`:168`) whose destructor calls `executor_->cancel()` then joins the thread; if the cancel runs before the new thread has entered `spin()`, the cancel is lost and `join()` never returns. This test does configure then cleanup within microseconds, the only place in the suite (and in field use, a configure immediately followed by cleanup or shutdown) where the window is open; it fits the lack of any output after `found 1 charts` (the hang is after the catalogue is built in `on_configure`, `:41`, in the first cleanup or the destruction of the node's listener) and the passing runs that spend time between creation and destruction. If so it is a real, rare hang in the node (vessel-reachable on a fast configure/cleanup cycle by a lifecycle manager), and the fix is in `s57_grids` (e.g. create the listener with `spin_thread=false` and spin it on the node's executor, or ensure the thread is running before cleanup), not a test change. (b) `on_cleanup` waits on every valid future in `pending_dataset_grids_` (`:160-162`) with no bound; unlikely here, since this case starts no job (`requested_grids_` is empty and the chart has no cell), but it is the same unbounded wait if a job ever blocks, and worth a bounded wait or a log. (c) A second `configure()` blocking on `S57Catalog` (`marine_charts/src/s57_catalog.cpp:55-75`, GDAL/OGR opens) is less likely: it would have printed `found 1 charts` again only after returning, so it cannot be excluded from the log, but nothing in it waits on a thread. (d) A test-only cause (the isolated runner's `ROS_DOMAIN_ID 1` shared with the `s57_grids` launch test's node, or leftover discovery from the previous test) cannot hang `configure()` and `cleanup()`, which make no graph call that blocks, so it is the least likely. — `s57_grids/test/test_reconfigure.cpp`, `s57_grids/src/grid_publisher.cpp:119`, `:160-168` (fixed in 3542b04: hypothesis (a), confirmed by source and a scratch reproducer: on Jazzy, rclcpp's `Executor::cancel()` before the listener's thread enters `spin()` is lost (ros2/rclcpp#3172, fixed on Rolling by #3174 only), so `~TransformListener` in `on_cleanup` never joins; the listener now has no thread of its own)
- [x] (suggestion, pull request body) Fill the body's "Results of both runs are added below." placeholder and add the failed run 45445136398 and its diagnosis; carried over from the previous entry, still open. (deferred: the pull request body is the host's to edit (no GitHub posts from a fix pass); the text for it, with run 45445136398 and the diagnosis, is in the Implementation entry below)

### False positives
- none (no external review comments to dismiss)

## Checkpoint
**Status**: complete
**When**: 2026-11-24 12:16 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (a must-fix from CI: an intermittent hang is diagnosed, not re-run)
**After**: findings
**Decision**: address

Read: the Integrated Review for pull request 58. The Jazzy job failed on `74d2467`: `test_reconfigure` hung in its first case (configure, cleanup, configure) after the first configure's catalogue line and was killed at 60 s; it passed on the previous head, which differs only in comments. The fix pass establishes where it hangs and whether the cause is in the test or in `s57_grids` (the triage's leading hypothesis, unverified: `tf2_ros::TransformListener`'s own spin thread, started at configure and joined at cleanup, where a cancel that lands before the thread spins is lost), fixes the cause, and shows a long repeat loop without a hang. If the cause is in the node and a vessel can reach it, the fix is made in the node, in its own commit, backported with the rest.

## Implementation
**Status**: complete
**When**: 2026-11-24 12:58 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**PR**: #58 at `3542b04`
**Addressed**: Integrated Review at `813c836` (2026-11-24 12:16 -05:00)
**Commits**: `3542b04` (code: s57_grids: serve the TF listener on the node's executor), `1c96fbf` (records: tick the finding)

### Actions
- [x] (must-fix) Intermittent Jazzy hang of `test_reconfigure`: diagnosed and fixed in the node — `s57_grids/src/grid_publisher.cpp:118-131`, `s57_grids/test/test_reconfigure.cpp`
- [x] (suggestion) Pull request body — deferred: the body is the host's to edit; text below

### Mechanism (hypothesis (a), confirmed by source; Jazzy run not possible here)
- `on_configure` built `tf2_ros::TransformListener(*tf_buffer_)`: its own hidden node and a dedicated thread (geometry2 `jazzy`, `tf2_ros/src/transform_listener.cpp:42-67`; `include/tf2_ros/transform_listener.hpp:266-269` makes a `SingleThreadedExecutor` and `std::thread([&]{executor_->spin();})`). `on_cleanup` reset it (`grid_publisher.cpp:168` at `74d2467`); `~TransformListener` (`transform_listener.cpp:97-103`) calls `executor_->cancel()` then `join()`.
- rclcpp `jazzy`: `Executor::cancel()` (`src/rclcpp/executor.cpp:455-465`) only does `spinning.store(false)` and triggers the interrupt guard condition; `SingleThreadedExecutor::spin()` (`src/rclcpp/executors/single_threaded_executor.cpp:27-45`) starts with `spinning.exchange(true)` and loops `while (rclcpp::ok(context_) && spinning.load())`. A cancel that lands before the thread reaches `spin()` is overwritten: the guard condition wakes one wait, the loop goes on, and with no `/tf` traffic the thread waits forever, so the join in `on_cleanup` never returns. This is ros2/rclcpp#3172 ("Race condition in executor when canceling before spin"), fixed on Rolling by ros2/rclcpp#3174 (merged 2026-07-31: a separate `cancel_requested_` flag held until the next spin consumes it; Rolling `single_threaded_executor.cpp:30-47`, `executor.cpp:493-504`; installed rclcpp 33.2.0 has it, `executor.hpp:571-576`). Not backported: rclcpp `jazzy` head `354a21b9` has no `cancel_requested_`; a maintainer comment on #3174 says the change alters class layout and may not be ABI compatible.
- Fits the CI log: the first case printed the first configure's `found 1 charts` and nothing after it; the hang is in the first `cleanup()`. The window is from the thread's creation in `on_configure` to its first `spin()` entry: normally well under a millisecond, which the test's configure-then-cleanup with nothing between can still lose when the CPU is busy (enc_updater's 117 pytest cases were running in parallel at that second).
- Hypotheses (b) to (d) ruled out for this hang: no job was pending (b); the second configure never started (c); nothing in configure/cleanup waits on the graph (d).

### Reproduction (Rolling, this machine; what is reproduced and what is inferred)
- No gdb, eu-stack or pstack on this machine (ptrace_scope 1), so no thread backtraces of a hung Jazzy process: Jazzy cannot be run here. Rolling's rclcpp has the fix, so `test_reconfigure` cannot hang here, and did not: pre-fix binary (`74d2467`) 1500 runs, 0 hangs.
- Scratch reproducer (scratchpad, removed): `TransformListener::init(spin_thread=true)`'s set-up and `~TransformListener`'s teardown line for line on Rolling's real rclcpp, with an executor subclass whose `spin()` and `cancel()` are Jazzy's bodies copied verbatim (sound: the loop uses the same `get_next_executable`/`execute_any_executable`/guard-condition machinery; Rolling's own `cancel_requested_` is never set by it). A hang is detected when the thread is still alive 2 s after `cancel()`; the reproducer then prints `is_spinning()`:
  - Jazzy semantics, cancel straight after thread start: 5 of 5 hang (`HANG: spin thread still running 2 s after cancel(); is_spinning()=1`; a second cancel releases it, proving the first was lost).
  - Jazzy semantics, 1 ms between thread start and cancel (no delay injected in the thread): idle 0 of 500; under load (20 busy loops on 20 CPUs) 26 of 300. This is the intermittency CI showed.
  - Rolling's stock executor, same set-up: 0 of 2000 idle, 0 of 2000 under load, 0 of 300 with a 2 ms delay injected at the thread's start (the window deliberately widened).
  - Real `tf2_ros::TransformListener(buffer)` build-and-destroy on Rolling: 0 of 2000 idle and under load. With `spin_thread=false` on a node (the fix): 0 of 2000 idle and under load.
- Inferred, not run: the same hang in the released Jazzy binaries (the code read is the `jazzy` branch heads; the released 28.1.x/0.36.x packages are cut from them).

### Whose defect, vessel reach
- Upstream (rclcpp on Jazzy), reached through tf2_ros' default listener; the node's use was correct API use. Rolling does not have it.
- A vessel on Jazzy can reach it: a `cleanup` transition, or the node's destruction (the `tf_listener_` member is destroyed the same way), any time after `configure` but before the listener's thread first enters `spin()`. That is normally sub-millisecond, so in practice a lifecycle manager or script that configures and immediately cleans up or shuts down (a rollback after another node's failure) on a loaded CPU. The effect: `on_cleanup` blocks the node's only executor thread, so lifecycle services, `get_datasets` and grid publishing all stop. Inferred from the loop condition `rclcpp::ok(context_)`: a SIGINT (context shutdown) would release it.

### The fix (`3542b04`)
- The listener subscribes on the GridPublisher node with `spin_thread = false`: no hidden node, no executor, no thread, nothing to cancel or join. Rolling: `TransformListener(*tf_buffer_, *this, false)` (NodeInterfaces); Jazzy: `TransformListener(*tf_buffer_, this, false)` (the `NodeT &&` template, `transform_listener.hpp:108-129` on `jazzy`; exact template match beats the `(BufferCore &, bool, bool)` constructor's pointer-to-bool conversion), chosen by `TF2_ROS_VERSION_GTE(0, 44, 0)` (NodeInterfaces came in geometry2 0.44.0, `46220a65`; Jazzy is 0.36.x). Jazzy branch not compiled here.
- What changes for the running node: `/tf` and `/tf_static` callbacks run on the node's own single-threaded executor (`main.cpp`) instead of a dedicated thread; the subscriptions now belong to the grid publisher node in the graph (no `transform_listener_impl_*` node). TF messages wait behind the node's timers and services; the longest is `checkForNewGrids`, which can block up to 10 ms per pending job (`wait_for(10ms)`); `/tf` QoS depth 100 queues meanwhile. All lookups (`grid_publisher.cpp` `tryPrecompute`, `GridCreationContext`) use `TimePointZero` and no timeout, so none needed the thread; `setUsingDedicatedThread` stays false, so a future lookup with a timeout would fail at once rather than wait: worth knowing before adding one.
- Rejected: keeping the thread and waiting until it spins before the reset (the listener's executor is private; nothing to observe); a node-owned executor and thread with a cancel-until-joined loop (reimplements tf2_ros for no gain over the node's executor); a delay between configure and cleanup (hides the window, does not close it).
- Test: `ChartThatCannotBeOpenedDoesNotAbortTheNode` now spins the node's executor while waiting for `map -> earth` (needed: nothing else serves the listener now). Complement, not the fix: each test has a 30 s watchdog that prints `[  FAILED  ] ReconfigureTest.<name> did not finish within 30 s: a lifecycle transition or a wait is hung` and exits 1, before CTest's 60 s. Not exercised by a real hang here (none occurs on Rolling).

### Proof (Rolling)
- `s57_grids` and `s57_layer` suites (`test.sh --packages-select s57_grids s57_layer --parallel-workers 2`): 7 of 7 and 8 of 8 passed; `Summary: 73 tests, 0 errors, 0 failures, 13 skipped`.
- `test_reconfigure` (fixed build) as separate processes, 4 in parallel, 16 busy loops, 60 s KILL per run: 500 runs, 499 passed, 0 hangs, 1 segfault; then 1500 runs, 1498 passed, 0 hangs, 2 segfaults. Pre-fix binary, same loop: 1500 runs, 1496 passed, 0 hangs, 4 segfaults.
- The segfaults are a separate, pre-existing defect, not from this change: an LD_PRELOAD SIGSEGV handler (dladdr backtrace) shows every one in a Fast DDS thread, `TypeLookupRequestListener::process_request` → `check_get_type_...` → `answer_request` → `TypeLookupManager::send` → `WriterHistory::create_change` → `pthread_mutex_lock` (addr 0x10), during a test case (node set-up or teardown), libfastdds.so.3.6 (Rolling). Inferred cause: a type-lookup request from another process in the same domain answered while a writer is being destroyed; my loop ran all copies in one domain, which `ament_add_ros_isolated_gtest` does not do in CI. Not fixed here; for the owner to weigh (a Fast DDS upstream issue).
- Hand run (`hand-runs/repro_abort.py`, NOAA US5PSMCC and US5PSMCD from unh_marine_simulation `bf57f58` `asv_sim/enc` as per-cell folders, fixed node binary): configure, activate (`True`), request west, "Grid ready: US5PSMCC.000 (1.90115 s from request to publish)"; deactivate, cleanup, configure, activate (all `True`); request east, "Grid ready: US5PSMCD.000 (2.05317 s …)"; node alive, exit 0. A grid is only made after the job looks up `map -> earth`, so transforms reach the node through its own executor, across a cleanup cycle.
- CI data point: run 45446691940 on the records-only head `cd4aa27`: Jazzy and Rolling both passed.

### Unverified until Jazzy CI
- That the Jazzy branch of the `#if` compiles and the hang is gone there (no Jazzy here). A green Jazzy job is weak evidence alone (the pre-fix code passed 2 of 3); the source mechanism and the reproducer are the argument.

### For the owner
- Upstream (draft, not posted), for ros2/rclcpp: "Backport request: #3174 (cancel before spin is lost, #3172) to Jazzy. On Jazzy, `tf2_ros::TransformListener` with its default dedicated thread hangs in its destructor when destroyed before the thread first enters `spin()`: `~TransformListener` cancels then joins, `Executor::cancel()` only stores `spinning = false`, and `SingleThreadedExecutor::spin()` sets it back to true. A lifecycle node that builds its listener in `on_configure` and resets it in `on_cleanup` hangs its cleanup intermittently under CPU load (seen in CI). #3174 changed class layout; an ABI-compatible variant for Jazzy (for example re-checking a cancel generation counter stored outside the class, or `spin()` not overwriting a cancel issued after the executor was constructed) would fix every Jazzy user of the default listener." Downstream packages using `TransformListener(buffer)` and resetting it soon after (marine_sidescan_mosaic's `mosaic_node.cpp:167` builds one the same way) share the exposure on Jazzy.
- Pull request body text for the host (replaces the "Results of both runs are added below." placeholder): "Jazzy run 45445136398 hung in `test_reconfigure` (first case, killed at 60 s): ros2/rclcpp#3172 on Jazzy, a cancel before the TF listener's spin thread starts is lost and `on_cleanup`'s join never returns. Fixed in `3542b04` by serving the listener on the node's executor (no thread of its own); see the Implementation entry in progress.md."
