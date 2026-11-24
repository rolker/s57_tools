# Plan: Port s57_tools to ROS 2 Rolling

## Issue

https://github.com/rolker/s57_tools/issues/52 (part of rolker/unh_marine_autonomy#415)

## Context

Six packages; the owner chose two parts (checkpoint, 2026-11-24): **Part 1**
(`s57_msgs`, `marine_charts`, `s57_to_geotiff`, `enc_updater`, CI on both
distros) is **the part to implement now**. **Part 2** (`s57_grids`,
`s57_layer`) is outlined here and planned again after part 1 merges. Each part
is its own pull request into `rolling` saying `Part of #52`, then a hand
backport to `jazzy` by `git cherry-pick -x`. `origin/rolling` exists and equals
`jazzy` (`d4f1283`). This machine: Ubuntu 26.04, Rolling, GCC 15, GDAL 3.12.2;
**Jazzy is verified by CI only.**

---

## Part 1 — implement now

### Evidence (session scratch copies, Rolling, `ROS_DOMAIN_ID=219`, checked empty)

**Baseline** (`git archive HEAD`, underlay and core installs sourced,
`--packages-up-to` the four): `s57_msgs` builds, 4 tests pass; `marine_charts`
fails on `CPL_CVSID("$Id$")` in `src/iso8211/ddffield.cpp:39`, `ddfutils.cpp:37`,
`ddfmodule.cpp:43`, `ddfsubfielddefn.cpp:43`, `ddfrecord.cpp:44`,
`ddffielddefn.cpp:42`; `s57_to_geotiff` and `enc_updater` not processed. With
only the six lines deleted: all four build; `colcon test` fails `enc_updater`
("Ran 0 tests ... NO TESTS RAN", exit 5; setuptools warns "Unknown distribution
option: 'tests_require'"); the rest pass (38 results, 17 skipped).

**With every part 1 code change below**: 4 packages build, `colcon test`
exit 0, **159 results, 0 failures, 18 skipped** (cppcheck): `enc_updater`
117 pytest (flake8, pep257 included), `marine_charts` 2 new gtests + lint,
`s57_to_geotiff` 6 gtests + lint, `s57_msgs` lint. `ament_xmllint` on all six
manifests: no problems. Warnings left: `tf2_ros/buffer.h` deprecation (via
`marine_charts/include/marine_charts/grid_creation_context.h:6`), CMake
`cmake_minimum_required` < 3.10 (three packages), `-Wparentheses` at
`marine_charts/src/s57_catalog.cpp:55`.

**#415 causes against the four** (text search plus the builds above):

| Cause | Hits in part 1 |
|---|---|
| 1, 2, 5, 6, 7, 8, 10, 13, 14, 16, new 17 | none (no `ament_target_dependencies`, no removed tf2 `.h`, no `.warn(`, no Nav2, no geodesy, no `gz4d`, no TF listener/broadcaster, no `Waitable`) |
| 3 | the six lines above |
| 4 | `enc_updater/setup.py:20` |
| 9 | none: `ament_xmllint` passes all six; `enc_updater` runs no xmllint test, so CI checks it by hand (step 5) |
| 11 | one: `enc_updater/enc_updater/nav_liveness.py:39,42` runs `ros2 node list`, which `ros2node` provides; the manifest declares only `ros2cli`. Tests mock `subprocess.run`, so nothing fails; on a machine without `ros2node` the interlock always refuses the swap. Every other include and import is declared (GDAL, `geographic_msgs`, `geometry_msgs`, `grid_map_core`, `tf2_ros`, `tf2_geometry_msgs`, `rclcpp`, `marine_autonomy`, `marine_vertical_datum`, `python3-gdal`, `python3-yaml`, `ament_index_python`, pytest). All external keys of both closures resolve on noble/jazzy and resolute/rolling except `grid_map_core` on Rolling (cloned, below). No launch files in the four. |
| 12 | none concurrent: `GDALAllRegister()` runs once per `S57Catalog` constructor (`s57_catalog.cpp:15`; `s57_to_geotiff` builds one per process, `exporter.cpp:543`, no threads) and once per `SyntheticCell` in `test_exporter.cpp:25`, sequential gtests in one thread. (`s57_grids` is part 2.) |
| 15 | none: modules are `config`, `datum_provisioner`, `downloader`, `health`, `nav_liveness`, `regenerator`, `registry`, `selection`; flake8 passes on Rolling |

### Cause 3 and the chart reader

- **Fix: delete the six `CPL_CVSID("$Id$")` lines.** GDAL 3.12's `cpl_port.h`
  no longer defines the macro; GDAL 3.8.4's (Jazzy/noble) still does, but
  upstream's own `frmts/iso8211/ddfrecord.cpp` at `v3.8.4` already has no such
  line (checked), and nothing references `cpl_cvsid`. Alternative: wrap each in
  `#ifdef CPL_CVSID`; rejected, it keeps dead lines and diverges further from
  upstream. unh_marine_autonomy has no vendored copy and made no cause 3 commit
  (`git log --grep` finds none); #415 only says the lines can be deleted.
- **What the vendored reader does in this product.** Only `S57Catalog`'s
  constructor uses it, to read the exchange set's `CATALOG.031`: for each `CATD`
  record whose `FILE` ends `.000` it makes an `S57Dataset` with bounds from
  `SLAT/WLON/NLAT/ELON`. Chart cells themselves are read by GDAL's own OGR S-57
  driver (`GDALOpenEx`, `s57_dataset.cpp:63,469`). If the catalogue reads no
  field, the constructor falls back to scanning `<enc_root>/<cell>/<cell>.000`.
- **What the existing tests prove: nothing about it.** `s57_to_geotiff`'s 6
  gtests build a synthetic in-memory OGR dataset (`MEM`/`Memory` driver) and
  call `exportCell` directly; no test constructs `S57Catalog` or touches
  `src/iso8211`. `marine_charts` has lint only.
- **The check (step 2):** `marine_charts/test/test_s57_catalog.cpp`, 2 gtests,
  over `test/data/enc_root/CATALOG.031`, the real catalogue from NOAA's
  `US5PSMCC.zip` (1347 bytes, sha256 `10a94bcf…40da8f`, downloaded 2026-11-24;
  NOAA's user agreement permits redistribution, as `asv_sim/enc` in
  unh_marine_simulation already relies on). It holds 8 `CATD` records (itself,
  four `.TXT`, `.000`, `.001`, `.002`); its subfields are ASCII, so the expected
  values are read from the bytes, independently of any ISO 8211 code: one
  dataset, `US5PSMCC.000`, bounds 43.05/-70.8/43.125/-70.725; box queries inside
  and outside select 1 and 0. No cell file is in the directory, so a reader
  failure falls to the scan, finds nothing and fails both tests. Rehearsed on
  Rolling: pass; **mutation-checked**, each fails both tests: float extraction
  halved in `ddfsubfielddefn.cpp:473`; `SLAT` read as `NLAT` in
  `s57_catalog.cpp`; catalogue path misnamed (fallback path). Alternative
  considered: reading a whole `.000` cell through the vendored reader and
  comparing feature counts with GDAL's driver (no `ogrinfo` here; `gdal-bin` is
  not installed); rejected as testing a path the product never uses.
- **Jazzy (GDAL 3.8) against Rolling (GDAL 3.12):** the same test, with the
  same fixed expected values, runs in both CI jobs; the vendored code differs
  between them only in the CPL headers it compiles against. Jazzy is CI only.

### Approach: commits, in order (each builds and tests alone; none touches `.agent/`)

1. **`marine_charts: drop CPL_CVSID from the vendored ISO 8211 reader (#415 cause 3)`**
   — the six lines. Backported.
2. **`marine_charts: test reading an S-57 exchange-set catalogue`** — the test
   above; `test/data/enc_root/CATALOG.031`; `test/data/README.md` (source URL,
   date, sha256, the redistribution sentence from NOAA's agreement; not the
   agreement's CRLF text, which the whitespace hooks would rewrite);
   `ament_add_gtest` with `TEST_ENC_ROOT` as a compile definition; the test also
   links `${geographic_msgs_TARGETS} ${geometry_msgs_TARGETS} tf2_ros::tf2_ros`,
   because `marine_charts` links these `PRIVATE` though its public headers
   include them (left alone, below); `<test_depend>ament_cmake_gtest</test_depend>`.
   Confirm with `pre-commit run --files` that the hooks treat `CATALOG.031` as
   binary (it holds 0x1e/0x1f) and leave it byte-identical. Backported.
3. **`enc_updater: tests_require to extras_require (#415 cause 4)`** — the
   multi-line `extras_require={'test': ['pytest']}` form of unh_marine_autonomy
   `c52bcb35`. 0 → 117 tests. Backported.
4. **`enc_updater: declare ros2node (#415 cause 11)`** — `<exec_depend>ros2node</exec_depend>`.
   Alternative: leave it for the follow-up issue (it fails nothing). Backported.
5. **`ci: Jazzy job builds all six packages; Rolling job for the four`** — one
   `ci.yml` for both branches (below). Backported whole (on `jazzy` the Rolling
   job skips).

Then the records commit (`.agent/work-plans/issue-52/`), not backported.

### CI design (`.github/workflows/ci.yml`, modelled on mru_transform and unh_marine_simulation at `rolling`)

- Triggers: push and pull_request on `[rolling, jazzy]`; `actions/checkout@v6`;
  `rosdep init` guarded; a header comment saying what each job builds and why.
- **Both jobs:** workspace = this checkout + a shallow **full** clone of
  rolker/unh_marine_autonomy at the job's distro branch. The closure
  `--packages-up-to` our packages takes from it exactly `marine_autonomy`,
  `marine_interfaces`, `marine_tiled_raster_store`, `marine_vertical_datum`,
  `marine_bathymetry_store` (checked at `jazzy` `ff6bd70` and `rolling`
  `0ca4a58`); `bathymetry_layer` and the other packages sit on disk outside it.
  Alternative: sparse clone of those five folders (mru_transform's way);
  rejected because a new internal dependency would break the job instead of
  joining the closure. `rosdep install --ignore-src -y --from-paths <closure>`
  with **no `-r`**, closure from `colcon list --paths-only`, refused if empty.
  A "Python tests ran" step (unh_marine_simulation's script) on
  `build/enc_updater/pytest.xml`.
- **Jazzy job** (`ros:jazzy-ros-core`): adds `geographic_info` **pinned at
  `f70b81a438172cd7a066dc1b18314d70e0eb6389`** with `COLCON_IGNORE` in
  `geographic_msgs` and `geographic_info` (cause 8). It stays: the Jazzy job
  still builds today's `s57_layer`, which includes `geodesy/ecef.h`; it is not
  only part 2's. Builds and tests all six (`grid_map_core`, `grid_map_ros`,
  `nav2_costmap_2d`, `nav2_util` from Jazzy binaries, as today; no Nav2 source).
  The test step keeps today's `source install/setup.bash` (the `s57_grids`
  launch test runs under it today; dropping it cannot be checked here).
- **Rolling job** (`ros:rolling-ros-core`, `if: (github.base_ref || github.ref_name) != 'jazzy'`):
  adds grid_map **pinned at `0ece1b53ca03199527d0d42913cf2567d0d6a913`** (tag
  `2.5.0`, the `rolling` head and the underlay's commit; alternative: the
  moving `rolling` branch). The closure takes only `grid_map_core` and
  `grid_map_cmake_helpers`. A guard step fails the job if `colcon list
  --names-only` of the closure contains `nav2_*`, `grid_map_ros` or
  `bathymetry_layer`. `ament_xmllint enc_updater/package.xml` (cause 9 has no
  other check there). Build `--packages-up-to s57_msgs marine_charts
  s57_to_geotiff enc_updater`; test `--packages-select` the four without
  sourcing the install.
- **Rehearsal** (fresh workspace, `env -i`, only `/opt/ros/rolling` sourced,
  clones fetched as the job would): closure as above, 11 packages, no Nav2;
  `rosdep install --simulate` and `rosdep check` with no `-r`: all satisfied
  (this machine already has the packages, so the apt step itself is not
  rehearsed); build 48 s wall (2 min 43 s CPU, 20 cores); test 3 s; 159
  results, 0 failures. Not rehearsable: the containers, apt installs, the Jazzy
  job.
- **Time estimates:** Rolling job about 5 min; Jazzy job about 5 to 6 min.
  Resting on: today's Jazzy job 3.3 to 3.6 min (runs of 2026-08-22, four
  packages with Nav2 binaries); latest Jazzy jobs of mru_transform 2.3 min and
  unh_marine_simulation 4.8 min (2026-11-23); the
  closure's 2.7 CPU-minutes, about 1 to 1.5 min on a 4-vCPU runner.
- **What the first Jazzy run is expected to show:** the first CI build of
  `s57_to_geotiff` and `enc_updater` anywhere (6 gtests, 117 pytest), their
  first cppcheck (Jazzy's runs; Rolling's skips) and the new catalogue test
  under GDAL 3.8. Expected to pass; a cppcheck or flake8 finding there would be
  a pre-existing defect, fixed in its own commit.

### Files to Change (part 1)

| File | Change |
|---|---|
| `marine_charts/src/iso8211/{ddffield,ddfutils,ddfmodule,ddfsubfielddefn,ddfrecord,ddffielddefn}.cpp` | delete `CPL_CVSID` line |
| `marine_charts/test/test_s57_catalog.cpp`, `test/data/enc_root/CATALOG.031`, `test/data/README.md` | new |
| `marine_charts/CMakeLists.txt`, `package.xml` | gtest, `ament_cmake_gtest` test_depend |
| `enc_updater/setup.py`, `package.xml` | `extras_require`; `ros2node` |
| `.github/workflows/ci.yml` | two jobs as above |

---

## Part 2 — outline only (planned again after part 1 merges)

- **Causes, from the issue review** (scratch build): `s57_layer/CMakeLists.txt:28`
  `ament_target_dependencies` (1) with `geodesy` (8: `${geodesy_INCLUDE_DIRS}`/
  `${geodesy_LIBRARIES}`, `<depend version_gte="1.0.7">`,
  `find_package(geodesy 1.0.7 REQUIRED)`; `ecef.h` at `s57_layer.cpp:5`);
  `s57_layer.cpp:10` `tf2/utils.h` (2); 15 `declareParameter` calls,
  `s57_layer.cpp:35-121` (6); `s57_layer.cpp:124` `create_client` now returns
  `nav2::ServiceClient` (**new, candidate cause 17**: free
  `rclcpp::create_client`, keeps `prune_requests_older_than` and
  `async_send_request`); `s57_layer.cpp:655` `create_subscription` (7);
  `test_tide_offset.cpp` (5 places), `test_allow_uncharted.cpp:47`,
  `test_depth_costs.cpp` on `nav2_util::LifecycleNode` (13). To check by hand:
  cause 11 (`s57_layer` uses `grid_map_ros`/`grid_map_msgs`/`tf2_ros` without
  declaring them), cause 9 (both manifests are format 2), cause 12
  (`grid_publisher.cpp:37` registers GDAL in configure while `std::async` grid
  jobs from line 226 may still run across a reconfigure). `s57_grids` built and
  passed its launch test on Rolling unchanged (cause 14 form at
  `grid_publisher.cpp:115` is the safe one).
- **Remedies as before:** a local `declareParameter` helper and the free
  `rclcpp::create_subscription`/`create_client`, as `bathymetry_layer` and
  `ais_layer` kept (owner: no big changes; Nav2's newer API is
  rolker/unh_marine_autonomy#428).
- **Tests it needs:** `s57_layer` is on the navigation path (marks charted
  hazards, land, restricted areas, depth cost); its 23 gtests cover what it
  marks, none covers `initialize()`, the subscription or the client. Add an
  overrides test (layer named other than its default, all 15 parameters
  overridden to values differing from the members' starting values, declared
  under the full name only) and an `initialize()` test (subscription and
  client on the node, `__has_include` switch for the node type), each
  read-back mutation-checked by misnaming it once.
- **Hand runs on Rolling:** build and test against the underlay's navigation2
  `9113449e`, grid_map `0ece1b5`, geodesy `f70b81a`; `s57_grids` launch on the
  `US5PSMCC` cells; record the runs. No costmap or simulation run is possible in
  CI, and none is planned: say so in the records, as #418 did.
- **Risks:** a silently ignored operator parameter (cause 6); the new client
  type; a subscription on a hidden node; Jazzy CI is the only Jazzy check.

---

## Left alone in part 1 (for the follow-up issue, drafted for the owner)

- `marine_charts` public headers include `geographic_msgs`, `geometry_msgs`,
  `tf2_ros`, `rclcpp` but link them `PRIVATE` and do not export `tf2_ros` or
  `rclcpp`; consumers name them themselves.
- `tf2_ros/buffer.h` deprecated (`grid_creation_context.h:6`); CMake minimum
  below 3.10 (`s57_msgs`, `marine_charts`, `s57_to_geotiff`); `-Wparentheses`
  `s57_catalog.cpp:55`; `marine_charts` description `TODO`.
- No test reads a real S-57 cell through GDAL's driver (`S57Dataset::open`,
  `exporter.cpp:488`, `runExport`), or the catalogue's fallback scan.
- cppcheck is skipped on Rolling (18 results).
- `ros2node`, if the owner prefers step 4 left out.

## Prior Art

#415 causes 3, 4, 9, 11, 12, 15 and their fixes (unh_marine_autonomy `c52bcb35`,
`83ceede6`); mru_transform's and unh_marine_navigation's two-job workflows
(guarded `rosdep init`, strict closure rosdep, pinned `f70b81a`); unh_marine_simulation's
"Python tests ran" step; unh_marine_autonomy's Rolling job (full clones kept out
by package selector); NOAA cells already vendored in `asv_sim/enc`. Taken as is.

## Serves

None written down (s57_tools has no goals or design document); for the
workspace, the Project11 Rolling port, rolker/unh_marine_autonomy#415.

## Design Picture

No change to the design picture; the project keeps no design document.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Smallest fix per cause; the one test added is the evidence the checkpoint asked for; improvements listed for a follow-up. |
| Test what breaks | The reader had no test; the new one is mutation-checked. The Python guard catches a 0-test run. |
| Verify before claiming | Baseline and full change set run on Rolling; Jazzy stated as CI only. |
| Name the rule before bending it | `ros2node` goes beyond "smallest build fix"; flagged as a choice. |
| Look for prior art | Earlier ports' workflows and fixes reused. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0013 progress.md entries | Yes | `## Plan Authored` entry; records in their own commit. |
| Others | No | Project-internal port. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| s57_tools `rolling` | `core.repos:22` pin in unh_marine_autonomy `rolling` | No — separate small change there (issue says so) |
| new cause 17 | comment on #415 after part 2 | No — part 2 |
| part 1 merged | hand backport of commits 1–5 to `jazzy` | Yes, owner's routine |

## Open Questions

- Step 4 (`ros2node`) now, or in the follow-up? Recommend now: one line, backports cleanly.
- Commit the 1347-byte NOAA catalogue as a fixture? Recommend yes (real data, ASCII-checkable).
- grid_map pinned by SHA rather than the `rolling` branch? Recommend the SHA.

## Estimated Scope

Two pull requests into `rolling` (part 1 now, part 2 later), each with a hand backport.
