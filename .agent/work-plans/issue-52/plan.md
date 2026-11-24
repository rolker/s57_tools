# Plan: Port s57_tools to ROS 2 Rolling

## Issue

https://github.com/rolker/s57_tools/issues/52 (part of rolker/unh_marine_autonomy#415)

## Context

The owner cut the issue into two parts (checkpoint, 2026-11-24). Each part is
its own pull request into `rolling` saying `Part of #52`, followed by a hand
backport to `jazzy` (`git cherry-pick -x`). Follow-up items go to
rolker/s57_tools#54 as a comment the host drafts.

This machine is Ubuntu 26.04 with Rolling, GCC 15 and GDAL 3.12.2. **Jazzy is
verified by CI only.** No CI job builds Nav2 source, so on Rolling
`s57_grids` and `s57_layer` are checked by hand.

---

## Part 1: delivered

`s57_msgs`, `marine_charts`, `s57_to_geotiff`, `enc_updater` and CI on both
distros were merged as pull request 53 (`ce1eb2b`) and backported as pull
request 55 (`89b552d` on `jazzy`). What it did:

- **Cause 3:** deleted the six `CPL_CVSID` lines.
- **Catalogue test:** a gtest reading NOAA's `CATALOG.031` through the vendored
  ISO 8211 reader, mutation-checked.
- **Cause 4:** `extras_require` in `enc_updater` (117 tests now run).
- **Cause 11:** `enc_updater` declares `ros2node`.
- **CI:**
  - The Jazzy job builds and tests all six packages (216 tests), with strict
    rosdep and geodesy pinned at `f70b81a`.
  - The Rolling job builds and tests the four Nav2-free packages (159 tests),
    with grid_map pinned at `0ece1b5` and unh_marine_autonomy at `rolling`.
  - The Rolling job has a guard against Nav2 source or keys entering its
    package closure.

Evidence for all of this is in `progress.md` (Implementation, reviews, CI runs
45415089806, 45417304066 and 45418748263).

---

## Part 2: implement now (`s57_grids`, `s57_layer`)

### Evidence (session scratch copies; Rolling; DDS domain 211, checked empty)

**Underlay this was checked against** (`/home/roland/project11/rolling/underlay`).
The install is older than the checkout for navigation2, so the commit each
install was built from is recorded:

- **navigation2:** the checkout is `9113449e`; the install was built on
  2026-11-19 from `dcffa939`. The reflog shows `dcffa939` cloned at 08:51, and
  the last builds ran at 12:06 and 14:45; the fast-forward to `9113449e` came on
  11-23. `git diff dcffa939 9113449e -- nav2_ros_common nav2_costmap_2d` is
  empty, so the install has the same API as the checkout.
- **grid_map:** `0ece1b5` (2.5.0), for both the checkout and the install.
- **geographic_info:** `f70b81a`, for both.

**Baseline** (`git archive` of this branch, `5f4fef3`, built with
`--packages-up-to s57_grids s57_layer`):

- `s57_grids` builds; ctest reports 6/6 (launch test with 2 results, plus lint).
- `s57_layer` fails at configure: `CMakeLists.txt:28`, `ament_target_dependencies`.

**Each change on the scratch copy, in the order the compiler demanded:**

| # | Change | #415 cause |
|---|---|---|
| a | `ament_target_dependencies(geodesy grid_map_ros)` replaced. `geodesy`: `${geodesy_INCLUDE_DIRS}` PRIVATE and `${geodesy_LIBRARIES}` PRIVATE, as bathymetry_layer `3968c5e5` did. `grid_map_ros` exports no CMake target on 2.5.0 (only `_INCLUDE_DIRS`/`_LIBRARIES`; a first try with `grid_map_ros::grid_map_ros` failed), so its variables go PUBLIC, as `s57_grids/CMakeLists.txt` already links it. The existing list moves to keyword `PUBLIC`, so the exported interface is unchanged. | 1, 8 |
| b | `find_package(geodesy 1.0.7 REQUIRED)` and `<depend version_gte="1.0.7">geodesy</depend>`; `s57_layer.cpp:5` includes `geodesy/ecef.h`. | 8 |
| c | `s57_layer.cpp:10`: `tf2/utils.h` changed to `tf2/utils.hpp`. | 2 |
| d | The 15 `declareParameter` calls (`s57_layer.cpp:35-121`) get a local lambda of the same name, copied from bathymetry_layer `048f5209`. It declares `getFullName(param)` unless that name is already declared. | 6 |
| e | `s57_layer.cpp:124`: `node->create_client<GetDatasets>(name)` changed to `rclcpp::create_client<GetDatasets>(node->get_node_base_interface(), ..._graph_..., ..._services_..., name)`. | **17 (new)** |
| f | `s57_layer.cpp:655`: `node_.lock()->create_subscription<GridMap>(...)` changed to `rclcpp::create_subscription<GridMap>(node, ...)`. It needs an lvalue `node` (an rvalue from `lock()` does not bind), as ais_layer `0e3a7cc`. | 7 |
| g | Tests: `nav2_util/lifecycle_node.hpp` changed to the `__has_include("nav2_ros_common/lifecycle_node.hpp")` switch with a `TestLifecycleNode` alias, as bathymetry_layer `ade5e896`. This touches 9 constructions in `test_tide_offset.cpp` (5), `test_allow_uncharted.cpp` (3) and `test_depth_costs.cpp` (1). | 13 |

**Result with a to g:** both packages build. ctest reports
`s57_layer` 7/7, and the 23 gtests pass (`test_depth_costs` 12,
`test_tide_offset` 7, `test_allow_uncharted` 3, `test_plugin_loading` 1).
`colcon test-result`: `Summary: 57 tests, 0 errors, 0 failures, 11 skipped`
(the skipped ones are cppcheck). The only warning left is the
`tf2_ros/buffer.h` deprecation, in the tests.

**The other causes:**

- **9:** `ament_xmllint` on both manifests: no problems. Both are format 2,
  so this checks well-formedness only.
- **11, s57_layer:** it uses `grid_map_core`, `grid_map_msgs`, `grid_map_ros`,
  `geographic_msgs`, `tf2` and `tf2_ros` but declares none of them. It also
  declares `marine_charts`, which it does not include.
- **11, s57_grids:** it uses `geometry_msgs`, `grid_map_msgs`, `nav_msgs` and
  `tf2_geometry_msgs` undeclared. Its installed launch file uses `launch` and
  `launch_ros`, and its launch test uses `rclpy`.
- **11, key resolution:** every added key resolves on noble/jazzy. On
  resolute/rolling all resolve except the `grid_map_*` keys, which the Rolling
  CI job never sees.
- **12, not concurrent (confirmed):** `GDALAllRegister()` runs only in
  `S57Catalog`'s constructor (`grid_publisher.cpp:37`, in `on_configure`).
  `on_cleanup` (`:146-163`) waits on every pending future before it returns,
  and a lifecycle node configures again only after a cleanup. The grid jobs call
  `GDALOpenEx`, never the registration.
- **14, unchanged (confirmed):** `grid_publisher.cpp:115` is
  `make_shared<TransformListener>(*tf_buffer_)`. That is the one-argument
  `TransformListener(BufferCore&, bool spin_thread = true, ...)` on Rolling
  (`transform_listener.hpp:119`) and the same default on Jazzy.
- **5, 10, 15, 16:** none (no `.warn(`, no `gz4d`, no module shadowing, no
  `Waitable`).

**What the existing 23 tests miss (mutations, each built and run on the
scratch copy):**

- Helper declares the bare name: **8 failures**. This is caught, because the
  layer stays disabled.
- Helper hard-codes `"chart_layer."` (the name every existing test uses):
  **0 failures**.
- Service name misnamed (`+= "_x"`): **0 failures**.

So the YAML path and the chart-request path are untested today.

### What the layer is and what it calls

The layer is on the navigation path: it marks charted land, restricted areas,
overhead clearance, unsurveyed and caution areas, discrete hazards and the depth
ramp into the Nav2 costmap. It runs this sequence:

1. `updateBounds` calls `get_datasets` on `s57_grids` (service
   `<s57_grids_namespace>get_datasets`, or `get_datasets_service`) whenever the
   window nears its buffered bounds. It also uses `prune_requests_older_than`
   on the client.
2. `getDatasetsCallback` subscribes to `<ns>datasets/<topic>` with latched QoS
   for each chart in the reply.
3. `mapGridCallback` stores the grids.
4. `generateTile` and `updateCosts` turn the grids into costs.

**Cause 17:** on Rolling, `nav2::LifecycleNode::create_client` returns
`nav2::ServiceClient` (`nav2_ros_common/lifecycle_node.hpp:211`,
`interface_factories.hpp:255`). That type does not assign to the
`rclcpp::Client::SharedPtr` member. Jazzy's member is rclcpp_lifecycle's
`create_client(name, qos = ServicesQoS(), group = nullptr)`. The free function
with the same three interfaces and those defaults exists on both distros: the
Rolling header shows it, and rclcpp `jazzy` `create_client.hpp:43-49` was
fetched to check. So change e keeps the same client, QoS and callback group,
not Nav2's wrapper.

### Approach: commits, in order

Each commit stands alone and builds on Jazzy in sequence. None touches
`.agent/`. All are backported unless marked otherwise.

1. `s57_layer: link geodesy and grid_map_ros through their CMake variables (#415 causes 1 and 8)`: change a.
2. `s57_layer: state the geodesy 1.0.7 floor that geodesy/ecef.h needs (#415 cause 8)`: change b. The comment says the floor is necessary but not sufficient (`f70b81a`).
3. `s57_layer: include tf2/utils.hpp (#415 cause 2)`: change c.
4. `s57_layer: include Nav2's lifecycle node from where each distro has it, in the tests (#415 cause 13)`: change g.
5. `s57_layer: test that every parameter override is read under the layer's name`: new `test/test_initialize.cpp`, test 1 below. It comes **before** the helper so Jazzy can run it against Nav2's own `Layer::declareParameter` (see the optional check below).
6. `s57_layer: declare parameters without Layer::declareParameter (#415 cause 6)`: change d.
7. `s57_layer: create the get_datasets client with rclcpp's free function (#415 cause 17)`: change e.
8. `s57_layer: create grid subscriptions with rclcpp's free function (#415 cause 7)`: change f.
9. `s57_layer: test that initialize()'s client reaches get_datasets and a grid reaches the costmap`: test 2 below.
10. `s57_layer: declare the packages it uses (#415 cause 11)`: `grid_map_core`, `grid_map_msgs`, `grid_map_ros`, `geographic_msgs`, `tf2`, `tf2_ros`, plus `ament_cmake_ros` as a test dependency. `marine_charts` stays: removing a declaration is not a port fix.
11. `s57_grids: declare the packages it uses (#415 cause 11)`: `geometry_msgs`, `grid_map_msgs`, `nav_msgs`, `tf2_geometry_msgs`; `launch` and `launch_ros` as exec dependencies; `rclpy` as a test dependency.
12. **Only if the host chooses to fix it** (see the `abort_flag_` section):
    `s57_grids: start each configuration with grid jobs enabled`. It adds
    `abort_flag_ = false` at the top of `on_configure` and
    `pending_dataset_grids_.clear()` after the waits in `on_cleanup`, plus its
    test.

After these come the records commit(s), not backported. Commit 5 compiles on
Jazzy at its position (it needs only commit 4's switch). On Rolling the package
compiles only after commit 8, as with the earlier layers.

**Optional Jazzy evidence the earlier layers lacked** (#415, part 3 comment).
In the backport pull request, push first up to commit 5. Jazzy CI then runs the
overrides test against Nav2's own `declareParameter`. Then push the rest.

### Cause 6: the helper and the two new tests

**Helper.** I recommend a third copy of the bathymetry_layer and ais_layer
lambda, in `onInitialize` (about 8 lines).

- **Alternative 1:** share one of the existing copies. Neither is in a library
  `s57_layer` can link without depending on unh_marine_autonomy or marine_ais,
  so sharing means a cross-repository change. cube_bathymetry chose a copy for
  the same reason.
- **Alternative 2:** Nav2's `declare_or_get_parameter`. It is Rolling-only, so
  it would need a distro switch, and the owner's rule keeps big changes out of
  the port (rolker/unh_marine_autonomy#428).

**Test 1: overrides** (commit 5, `ament_add_ros_isolated_gtest`, as ais_layer
`97e9311` does for its DDS-touching test).

- **Setup:**
  - The layer is initialised as `"charts"`, a name no existing test uses, on a
    `TestLifecycleNode` whose `parameter_overrides` set all 15 parameters.
  - Nothing is declared before initialisation.
  - Each override differs from the member's starting value and from its
    declared default.
- **Declared-name checks:** every `charts.<p>` is declared, no
  `chart_layer.<p>` is declared, and no bare `<p>` is declared.
- **Read-backs:**
  - **Lethal decisions, observed through the protected `get_cost_from_grid`
    on a crafted grid:**
    - `minimum_depth` (2.5; default 0): depth 2.4 is lethal and 2.6 is not.
    - `maximum_caution_depth` (7.5; default 5): the ramp still raises cost at
      6.0.
    - `overhead_clearance` (20; default 10): overhead 15 is lethal.
    - `unsurveyed_cost` (77; default 100): an unsurveyed cell returns 77.
      Declared from an `unsigned char`, this one is the read most likely to
      misbehave. If it fails on both distros it is pre-existing, and goes to
      #54 with a note.
    - `depth_costs` (false): the protected `depth_costs_` member, and a hazard
      cell is lethal while depth is ignored.
  - **Whether charts arrive at all:** `get_datasets_service` (`/test_<pid>/svc`)
    and, in a second case without it, `s57_grids_namespace` (`test_<pid>`, so
    the service is `test_<pid>/get_datasets`). Both are observed on the graph
    with `get_client_names_and_types_by_node(<node>)`: the client must sit on
    the layer's own node under exactly that name. A wrong name means no charts.
    With `allow_uncharted` defaulting to true, the water is then silently free.
    **This is the most dangerous silent failure.**
  - **`allow_uncharted`** (false): `updateCosts` on a tile with no charts
    writes `NO_INFORMATION`. The existing test does this only with a
    pre-declared parameter.
  - **Tide:** `chart_datum_frame` and `sea_surface_frame`, with a transform
    between the two overridden frames only. `updateBounds` then sets the
    protected `tide_offset_`. `tide_invalidate_threshold` (0.3) is checked
    with a 0.2 change that leaves the offset alone.
  - **`enabled`:** proved in a defaults case, because Nav2's constructor sets
    `false` (the bathymetry lesson).
  - **Declared only, as bathymetry_layer did:** `tile_size`,
    `update_timeout`, `buffer_fraction`. They are private, and have no cheap
    observable.
- **Mutation check of each read-back:** misname its `get_parameter` once and see
  a test fail. Also hard-code `"chart_layer."` in the helper; that mutation
  passes all 23 existing tests today.

**Test 2: client, subscription, grid** (commit 9, same file).

- **Setup:**
  - One process, two nodes on a per-pid namespace: the layer's
    `TestLifecycleNode`, and a helper `rclcpp::Node` serving `GetDatasets` and
    publishing a latched `GridMap`. The grid is built with
    `grid_map::GridMapRosConverter::toMessage`: 20 m by 20 m, with one quadrant
    `elevation = 1.0` (land).
  - The `tf2_ros::Buffer` gets a static `earth`→`map` transform, so
    `worldToLatLon` works.
  - A `LayeredCostmap` with a window over the grid.
- **What it asserts:**
  - After `updateBounds`, the request reaches the helper's service (its
    callback counts 1).
  - After spinning, the layer's node has exactly one `GridMap` subscription
    on `<ns>datasets/<topic>` (counted per node, as ais_layer `c18c8f0`).
  - After the grid arrives and `updateBounds`/`updateCosts` run, the land
    quadrant is `LETHAL_OBSTACLE` in the master grid and the water quadrant is
    not.
- **What it catches:** a misnamed service (no request: fails), a subscription
  on the wrong topic or node (no grid: fails), and a client wrapper that drops
  replies.
- **Mutation-checked** by the misnamed service and a wrong topic prefix.
- **Trap from Part 1:** check with `ldd` which `libs57_layer.so` each mutated
  test loads.

### How the port is shown not to change what the layer marks

**What the 23 existing tests cover.** The 12 depth-cost tests cover
`get_cost_from_grid` in both modes: the ramp, hazard, land, restricted,
caution, and suppressed mode. The 7 tide tests cover tide-adjusted depth and
tile invalidation. The 3 allow-uncharted tests cover `updateCosts` on uncharted
tiles. The plugin test covers pluginlib loading.

**What they do not cover:** operator overrides, the request/subscription path
(both shown above to pass under mutation), `generateTile` compositing of real
grids, and Nav2's own layered costmap.

**What the port changes.** It changes no line of `get_cost_from_grid`,
`generateTile`, `updateCosts` or `worldToTile`. Its diff is
`onInitialize`, one line in `getDatasetsCallback`, CMake, manifests and tests.
So a change in marking can come only from three places:

1. A parameter read differently. Test 1 covers this.
2. Charts not arriving. Test 2 and the hand run cover this.
3. Rolling's Nav2 costmap behaving differently. Only the hand run covers this,
   and only on Rolling.

**A before/after costmap is not possible on Rolling:** the pre-port tree does
not compile there. On Jazzy it is possible only off this machine. The owner's
Jazzy machine (deadpool) could run the same scripted hand run at `jazzy`
`89b552d` and at the backport branch, and compare cell by cell. I offer it as an
option; it is not part of the plan.

**The hand run on Rolling (piloted for this plan).**

- **Setup:**
  - `nav2_costmap_2d` (the install built from navigation2 `dcffa939`) with
    only `s57_layer::S57Layer` ("chart", `minimum_depth` 2,
    `maximum_caution_depth` 6, `s57_grids_namespace` "/"), 1 m resolution,
    1000 by 1000 cells.
  - `s57_grids` with the NOAA cells US5PSMCC/US5PSMCD from
    unh_marine_simulation `asv_sim/enc` (`bf57f58`), as an ENC_ROOT of
    per-cell folders.
  - A static `earth`→`map` transform (ENU at 43.07 N, 70.75 W) and
    `map`→`base_link`.
- **The oracle is independent of the layer:** GDAL's own S-57 driver reads
  `LNDARE` and `DEPARE.DRVAL1` from the same cell. A sampled costmap cell is
  expected lethal if it is in land or `DRVAL1 < minimum_depth`.
- **What the pilot showed:**
  - The client reached `/get_datasets`, the subscription received
    `US5PSMCC.000`, and a costmap was published. This is cause 7 and cause 17
    working end to end against Rolling's Nav2.
  - **Static window:** 1,000,000 of 1,000,000 cells lethal, and the oracle
    says 100% of that window is land or shallower than 2 m. Agreement.
  - **Rolling window, robot at (1000, 1000):** 6779 of 6779 sampled land cells
    lethal, with water cells free or ramped.
  - **Rolling window, robot at the map origin (0, 0):** the bottom row and left
    column of tiles (19 of 100) came out **free**, where the chart says land or
    shallow. See "Found defect 2" below.
- **What the hand run shows:** on Rolling, the ported layer fetches charts and
  marks land and shallow water lethal where GDAL says they are, for the
  configurations run.
- **What it does not show:** that Jazzy marks the same (no Jazzy here), any
  composite with other layers or inflation, the planner's behaviour, or any
  simulation.
- **Honest claim:** the marking code is unchanged by the port; its inputs (the
  parameters and the charts) are proved by mutation-checked tests on both
  distros (Jazzy by CI); and on Rolling one hand run agrees with an independent
  reading of the chart, except for a defect that exists without the port.

### The `abort_flag_` defect in `s57_grids` (facts; the host decides)

**The defect:**

- `abort_flag_` is set in `on_cleanup` (`grid_publisher.cpp:148`) and never
  reset (it is initialised at `grid_publisher.h:122`).
- After a cleanup and a new configure, every new `getGrid` returns null at
  `s57_dataset.cpp:177`.
- `checkForNewGrids` then dereferences it at `grid_publisher.cpp:259` (and
  `:284` when `publish_costmaps` is on). `:487` is guarded by `if(grid)` at
  `:484`.
- `on_cleanup` also leaves aborted futures in `pending_dataset_grids_`. A job
  cut short by the cleanup is then dereferenced in the same way after the next
  configure.

**Reproduced on Rolling (baseline build, scripted):**

- configure, activate, request the west cell: "Grid ready".
- deactivate, cleanup, configure, activate, request the east cell: the node
  aborts with `__shared_ptr_deref ... Assertion '__p != nullptr' failed`
  (exit 250).
- **Control without the cycle:** the east cell becomes ready.
- **With the two-line fix on the scratch copy:** both cells are ready and the
  node stays alive.

**Reachable?** Nothing in the project cleans this node up automatically. ben's
launch configures and activates it once, and no lifecycle manager lists it. An
operator can do it: ros2launch_gui offers Cleanup on inactive lifecycle nodes,
and so does `ros2 lifecycle set`. `s57_grids_launch.py` has `respawn=True` but
transitions only once, so the respawned node stays **unconfigured**. The layer
then logs "service is not ready" and gets no charts. With
`allow_uncharted: true` (the default) the chart's hazards are then **silently
absent** from the costmap. This affects both distros, and it is not a port
cause.

**Test that would pin it:** a gtest compiling `grid_publisher.cpp` (it is only
in the executable today), on a subclass with `abort_flag_` made protected. It
runs configure, cleanup, configure through the lifecycle API and asserts the
flag is clear and no pending job is left. That is mutation-checked by deleting
the reset. It checks the state, not the crash. A crash-level test needs a chart
cell with features in the repository: the NOAA cells are 1.0 to 1.2 MB, over
the 500 KB pre-commit limit, and a GDAL-written synthetic cell came out with
zero extents in a pilot. The scripted before/after run goes in the records.

**Choice and recommendation:**

- **(A)** Fix it in this port as commit 12: 2 lines, its test and the recorded
  repro. Backportable.
- **(B)** Leave it and track it in #54.

**I recommend (A).** It is small, it is reproduced, it is on the chart path,
and it changes no marking code.

### Found defect 2: tile addressing in `s57_layer` (new; the host decides)

**The defect:** `worldToTile` (`s57_layer.cpp:174-175`) truncates toward zero
(`int ix = (x - x_origin_)/resolution_/tile_size_`). For coordinates below the
origin captured at `matchSize`, the tile is then mis-addressed.

**Evidence:**

- In the rolling-window hand run with the robot at the map origin, 19 tiles
  (the bottom row and left column, 190,000 cells) were left free where the
  chart says land or shallow.
- Changing to `std::floor` on the scratch copy cleared it in the same run
  (1,000,000 of 1,000,000 lethal, matching the oracle). The change was then
  reverted.
- bathymetry_layer's `worldToTile` (`bathymetry_layer.cpp:349-354`) already
  uses `std::floor`.
- The lines are unchanged by the port and go back at least to `b5be9f3`
  (2026-04-14). The arithmetic is distro-independent, so it is almost surely
  on Jazzy too. That is not verified here; deadpool or a unit test would show
  it.

**What it means for the vehicle:** a boat working west or south of where its
costmap window started can have charted land and shoals missing from a band of
the costmap. **That is a grounding risk.**

**Choice and recommendation:**

- **(A)** Fix it in this port.
- **(B)** Open its own issue now and fix it in its own pull request right
  after Part 2: about 2 lines plus a unit test of `updateBounds`/`updateCosts`
  with a window straddling the origin, and before/after costmaps.

**I recommend (B).** The port's validation rests on "no marking line changed",
and a fix that changes what the layer marks deserves its own review. Its
urgency should go to the owner now, not wait for #54.

### CI

**No workflow change.**

- Every key added in commits 10 and 11 resolves on noble, so the Jazzy job's
  strict rosdep install takes them.
- geodesy is already pinned at `f70b81a` (Part 1).
- The Rolling job's closure excludes both packages, and its guard checks only
  that closure.

**What the Jazzy job will show for Part 2:** the ported `s57_layer` (helper,
free functions, header switch) compiles against Jazzy's Nav2. The 23 old tests
plus tests 1 and 2 pass against Jazzy's `Layer`, plus the abort test if chosen.

**What only the Rolling hand runs show:** everything on Rolling for these two
packages.

### Hand runs on Rolling that stand in for CI

These go in the Implementation entry. Scripts go under
`.agent/work-plans/issue-52/hand-runs/` in a records commit, not backported.

1. **Clean build:** `./build.sh --parallel-workers 4 --packages-up-to s57_grids s57_layer --cmake-args -DBUILD_TESTING=ON`, from empty build folders, recording the summary line. Record the underlay commits above.
2. **Full test run:** `./test.sh --parallel-workers 4 --packages-select s57_grids s57_layer`. Record the `colcon test-result --verbose` lines and the `Summary` line.
3. **Repeats as separate processes:**
   - DDS domain above 150, checked empty first with `ros2 node list --no-daemon`;
     `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
   - Each new gtest binary 50 times, and the old four 10 times each.
   - `s57_grids`' launch test 20 times (`ctest -R test_node_launch --repeat until-fail:20`).
   - Report pass counts.
4. **rosdep:** `rosdep resolve` of every added key with `--os=ubuntu:noble --rosdistro=jazzy` and `--os=ubuntu:resolute --rosdistro=rolling`. `rosdep check --from-paths` the two packages, with no `-r`.
5. **Mutations:**
   - each read-back of test 1 misnamed once;
   - hard-coded prefix in the helper;
   - misnamed service and wrong topic prefix for test 2;
   - the abort reset deleted (if commit 12);
   - each with `ldd` proof of the loaded library.
6. **The costmap hand run** above, both windows and robot positions (0, 0) and
   (1000, 1000), with the oracle's counts. Record defect 2 as found, not fixed.
7. **The abort repro**, before and after (if commit 12).

The hosted core install needs nothing rebuilt: Part 2 changes no package it
builds.

### Files to change

| File | Change |
|---|---|
| `s57_layer/CMakeLists.txt` | causes 1 and 8 (commits 1, 2); `ament_add_ros_isolated_gtest(test_initialize ...)` (commit 5) |
| `s57_layer/package.xml` | geodesy floor (2); cause 11 keys and `ament_cmake_ros` (10) |
| `s57_layer/src/s57_layer.cpp` | lines 10, 35-124, 655 (commits 3, 6, 7, 8) |
| `s57_layer/test/test_{tide_offset,allow_uncharted,depth_costs}.cpp` | node switch (4) |
| `s57_layer/test/test_initialize.cpp` | new (5, 9) |
| `s57_grids/package.xml` | cause 11 keys (11) |
| `s57_grids/src/grid_publisher.{h,cpp}`, `s57_grids/CMakeLists.txt`, `s57_grids/test/test_reconfigure.cpp` | only with commit 12 |

### Draft comment for rolker/unh_marine_autonomy#415 (the host shows it to the owner)

> **Cause 17: Nav2's lifecycle node wraps `create_client`.** On Rolling,
> `nav2::LifecycleNode::create_client<SrvT>(name, use_internal_executor = false)`
> returns `nav2::ServiceClient<SrvT>::SharedPtr`
> (`nav2_ros_common/lifecycle_node.hpp:211`, built by
> `nav2::interfaces::create_client`), not `rclcpp::Client<SrvT>::SharedPtr`.
> Code that stores the result in an `rclcpp::Client` pointer fails:
> `no match for 'operator='` between the two. Jazzy's `nav2_util::LifecycleNode`
> inherits rclcpp_lifecycle's member, `create_client(name, qos = rclcpp::ServicesQoS(), group = nullptr)`.
>
> Remedy that builds on both distros and keeps the same client:
> `rclcpp::create_client<SrvT>(node->get_node_base_interface(), node->get_node_graph_interface(), node->get_node_services_interface(), name)`,
> the free function with the same defaults on both (rclcpp `create_client.hpp`).
> This is the client counterpart of cause 7. Found in `s57_layer`, which keeps
> `prune_requests_older_than` and `async_send_request` on an `rclcpp::Client`
> (rolker/s57_tools#52, part 2). Compiling is not evidence the client still
> reaches its service: `s57_layer`'s existing 23 tests pass with the service
> name deliberately misnamed. Its new test serves `get_datasets` in-process
> and checks that the request arrives and the reply's grid subscription
> delivers a grid to the costmap.
>
> Where else: any Nav2 plugin or node that calls `create_client` on Nav2's
> node type and keeps an `rclcpp::Client`. Search for `create_client<` in
> code built on Nav2 before trusting a clean build.

---

## Left alone (for the #54 comment the host drafts), Part 2 additions

- **`s57_grids`:**
  - `GridCreationContext`'s constructor calls `lookupTransform(map, "earth")`
    (`grid_creation_context.cpp:14`) inside the `checkForNewGrids` timer
    (`grid_publisher.cpp:229`) with no `try`. A missing `earth` transform
    throws out of a timer callback (not reproduced).
  - `on_cleanup` keeps `grid_publishers_`, `requested_grids_to_publish_` and
    `dataset_grids_`.
  - `s57_grids_launch.py` respawns the node but does not re-run the transitions.
  - Defect 2, and the abort defect, if not fixed here.
- **`s57_layer`:** `unsurveyed_cost` is declared from an `unsigned char` (if
  test 1 shows a problem). The tests include the deprecated
  `tf2_ros/buffer.h`. cppcheck skips on both distros.
- **Both:** the manifests are format 2 (no schema check).

## Prior Art

- bathymetry_layer: `3968c5e5`, `4673869a`, `ade5e896`, `048f5209`, and its
  tests `1245f373`, `8dfdaa98`.
- ais_layer: `0e3a7cc`, `3a6fdca`, `97e9311`, `c18c8f0`.
- The #415 comments on causes 6, 7, 13 and 14.
- bathymetry_layer's floor-based `worldToTile`.

All taken as is. Nav2's `declare_or_get_parameter` was rejected (#428).

## Serves

None written down (s57_tools has no goals or design document). For the
workspace: the Project11 Rolling port, rolker/unh_marine_autonomy#415.

## Design Picture

No change to the design picture; the project keeps no design document.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | One fix per cause. The two found defects are put to the host, not folded in silently. |
| Test what breaks | Two mutations that pass the 23 existing tests become caught. The marking claim rests on an independent oracle. |
| Verify before claiming | Jazzy is CI only. The before/after costmap is stated as impossible here. The underlay install commit is recorded. |
| Name the rule before bending it | Commit 12 bends "no big changes"; it is presented as a choice. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0013 | Yes | `## Plan Authored` entry; records in their own commits. |
| Others | No | Project-internal port. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| cause 17 found | comment on #415 | Drafted above; posted by the host |
| s57_tools `rolling` complete | `core.repos` pin in unh_marine_autonomy | No, separate small change |
| defect 2 | its own issue | Drafted on the host's yes |

## Open Questions

- **Commit 12 (abort fix) in this port?** I recommend yes.
- **Defect 2 (tile addressing):** I recommend its own issue now and a fix right
  after Part 2, shown to the owner as a grounding risk.
- **Push the backport in two steps** for Jazzy evidence of Nav2's own
  `declareParameter`? I recommend yes; it is cheap.
- **Offer the deadpool before/after costmap comparison to the owner?** I
  recommend offering it, not requiring it.

## Estimated Scope

Part 2 is one pull request into `rolling` (11 or 12 commits) and its hand
backport.
