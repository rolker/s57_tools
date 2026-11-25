---
issue: 63
---

# Issue #63 — s57_layer generateTile reads and writes one cell past a tile row when a chart cell edge lies exactly on the tile edge

## Issue Review
**Status**: complete
**When**: 2026-11-24 19:27 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**Issue**: #63

### Scope Assessment

**Well-scoped?** Yes. It is one loop in `S57Layer::generateTile` (`s57_layer.cpp:628-648` on `rolling` at `a1ba225` and on `jazzy` at `c8f253e`). The fix is a few lines and a test. The issue is right about the defect and its two effects. Some details need correcting (below), and the crash it calls "plausible" now reproduces.
**Right repo?** Yes. Nav2's `worldToMapEnforceBounds` does what its code says: it clamps only a point strictly beyond the far edge. Using it for a far corner that can lie on that edge is this layer's mistake.
**Serves**: none written down for the project. In practice: the vessels' costmaps show charted hazards where they are charted, and the costmap node does not corrupt its own memory.
**Design picture?** No change. The project has no design document, and the layer's README does not describe how tiles are composited.
**Dependencies**: #57 is merged on both branches and does not touch this loop. #59 changes the same function, a few lines above (see Recommendations for the order).

#### The defect re-established on this branch

I used a scratch harness outside the tree. It compiles a copy of `s57_layer.cpp` (private made protected) into the executable and calls `generateTile` directly over 4 x 4 tiles of one chart that covers all of them, so #59 plays no part. The executable also defines its own `Costmap2D::getCost(mx, my)` and `setCost(mx, my, c)`. These take the place of Nav2's (checked with `nm`), count every out-of-range index, and either do the access as Nav2 does or skip it. Chart cells are land, water or NaN by a hash of their index. Each tile cell is compared with an expectation computed independently from the chart: the value of the chart cell that overlaps it with positive area and carries data, taking the east-most, then north-most, which is the order `SubmapIterator` visits them in. Builds: Release, and AddressSanitizer with recovery.

Read and write counts, 1 m, layer origin 0, chart cells the size of tile cells, 16 tiles:

| Chart cell edges on | 4-cell tiles: reads out of range / past buffer / writes out of range | 100-cell tiles | Wrong cells (4-cell / 100-cell) |
|---|---|---|---|
| east and north tile edges | 272 / 176 / 48 | 6416 / 3248 / 1584 | 26 of 256 / 814 of 160000 |
| east edge only (y a quarter cell off) | 128 / 32 / 48 | | 26 of 256 |
| north edge only (x a quarter cell off) | 128 / 128 / 0 | | 0 |
| neither (0.37 cell off) | 0 | 0 | 0 |

The same counts at 0.5 m. All the wrong cells are the east-edge write, which lands in cell `(0, j+1)`. The 4-cell counts match the issue's figures for `AboveOriginControl` (272 / 48 / 176). That is a coincidence of set-up (same tile size, resolution and alignment); the committed tests themselves were not re-run.

- **AddressSanitizer** sees it once the accessors are compiled into the executable: `heap-buffer-overflow READ of size 1` in `Costmap2D::getCost`, called from `generateTile` at `s57_layer.cpp:643`. It reported no write past the buffer, because the bytes after the buffer under ASan's allocator never read 255.
- **Crash at BEN's tile size (100 cells)**, Release build, real accesses, 20 rounds of 16 tiles per run. Aligned at 1 m: 3 of 11 runs died with SIGSEGV. Aligned at 0.5 m: 4 of 5. Offset 0.37 cell: 0 of 5. Each surviving aligned run made 164-203 writes past a tile's buffer. A signal handler's backtrace shows the crash in `std::_Rb_tree_increment` inside `generateTile`, which is a `std::map` node corrupted by the write. The next heap chunk after a 10000-byte tile buffer can be a map node or another tile's buffer, which is full of 255, so the write reaches live data. A north-edge index reaches up to `tile_size + 1` bytes past the end.
- Painting from a chart cell a whole tile width away is confirmed. With east-first iteration, the east-edge write reaches `(0, j+1)` before that cell's own chart cell is visited, so it wins whenever the two values differ.

#### Each statement in the issue, checked against the source

- Line numbers: the loop is now `s57_layer.cpp:628-648` (the call at 629/634, the read at 643, the write at 645), the same on `rolling` and `jazzy`. The `:438-455` in the issue predates #57. `costmap_2d.cpp`: `worldToMapEnforceBounds` at `:328` on Rolling and `:321` on `origin/jazzy`. `getCost`/`setCost` are not inline on either (`:265/275` Rolling, `:264/274` Jazzy). `getIndex` is `my * size_x + mx` on both. The layer source is identical on `rolling` and `jazzy` here, and gitcloud's `jazzy` of s57_tools has no commits that GitHub's lacks (same loop).
- **The condition, corrected.** The function clamps when `wx > resolution*size + origin` and otherwise truncates `(wx - origin)/resolution`. The index `size` therefore comes out when the coordinate is not strictly beyond the computed edge but the quotient rounds to `size`. That includes a coordinate 1-2 ulps *below* the edge (seen at origins -12.3 with 0.5 m and 0.1 m). It also excludes some coordinates exactly on the edge, where the quotient rounds just under `size` (origin -4321.1 at 0.1 m, origin 3.7 at 0.05 m gave `size - 1`). "Exactly on the edge" is the usual case, not the rule.
- **West and south edges:** a coordinate on the tile's origin gives 0, and one below it, including negative coordinates, clamps to 0. Neither gives an out-of-range index. Clamping does make a chart cell that only touches the tile from the west or south paint column or row 0. That chart cell is visited last, so the cell goes wrong only when its own chart cell is NaN.
- **Near corner** (by reading, not measured): a chart cell that only touches the east edge from outside gets a near index of `size` and a far index clamped to `size - 1`, so its loop is empty. If rounding put its west edge an ulp beyond the edge, the near index would clamp to `size - 1` and the cell would paint column `size - 1`. Such a cell is visited first, so it would win. A fix must not clamp the near index down to `size - 1`.
- **East edge:** confirmed. `(size, j)` is `(0, j+1)`, the read always happens, and the write happens when that cell is still 255. In the last row, `(size, size-1)` is the first byte past the buffer.
- **North edge:** confirmed. `(x, size)` is `x + 1` bytes past the end. The read always happens, and the write happens when the byte there reads 255.
- **"ASan does not report it, because the accesses are made inside Nav2's library"**: true of an ordinary build. It reports it with the accessors compiled into the executable (above).
- **"About 1 run in 40", "a plausible cause"**: now reproduced directly, 7 of 16 aligned runs at 100-cell tiles against 0 of 5 offset.
- **Interior coincidences** are not in the issue. A chart cell whose far edge lies on an interior tile-cell edge also claims the next tile cell. East-first order makes this harmless unless that neighbour's chart cell has no data (NaN, or `NO_INFORMATION` from `get_cost_from_grid`). Then the neighbour's value spreads one cell into it: 1195 of 6400 cells with 20% NaN chart cells at 20-cell tiles. This also defeats "leave the cell unclaimed so a coarser chart can supply data" (`get_cost_from_grid`'s comment) for that one cell.

#### Every other index computation

- `s57_layer` `generateTile`: this loop is the only use of `worldToMap*`, `getCost` or `setCost` (grep).
- `s57_layer` `updateCosts` (`:660-717`): raw index arithmetic on half-open ranges. `start_i >= -tile_offset_x`, `i < min(max_i, tile_size - tile_offset_x)`, and `j + tile_offset_y < tile_size` keep every source index in `[0, tile_size)`, and the target index is inside Nav2's bounds. A negative `i_count` gives an empty loop; the `getIndex` arithmetic before it reads nothing. **Not affected.**
- `s57_layer` `updateBounds`, `worldToTile`, `recordUncovered`, `chooseStrip` (the #57 strip code): world coordinates and tile IDs only, with no cell index. **Not affected.**
- `marine_charts` rasterization uses grid_map's checked `getIndex`. **Not affected.**
- `bathymetry_layer` (`unh_marine_autonomy` at `0ca4a587`): `generateTile` loops over `0..tile_size-1` and writes with `setCost(tx, ty)`. `updateCosts` (`:1171-1233`) is the same half-open copy as s57's. **Not affected.**
- `ais_layer` (`marine_ais` at `4ded14c`): `paintSweptPose` (`:635-636`) takes its far corner from `worldToMapEnforceBounds`, so it can get `size`. The circle test rejects that column before any access, though, because its centre lies at least half a cell beyond the reach, and the row is skipped before its index is formed. `clearLastPaint` (`:858-865`) passes `xn + 1, yn + 1` to `resetMapToValue`, which would write past the buffer if `xn` were `size`. Its corners are cell centres, though, which stay half a cell off any edge as the window moves by whole cells. **Not affected today.** Both rely on that reasoning, not on a bounds check. Worth knowing; no issue needed.

#### How likely with real charts

The layer reads only `s57_grids` grids. `S57Dataset::getGrid` (`marine_charts/src/s57_dataset.cpp:157-162`) places each grid at the centre of the chart's lat/lon box, converted through ECEF into the map frame, with resolution `chart scale x 0.0003125 x resolution_factor` (6.25 m at 1:20,000). Nothing snaps the grid to anything. The layer's origin is the costmap's origin, which comes from the robot's position (rolling) or the configured origin, and which Nav2 keeps on its own grid. A chart cell edge therefore falls on a tile edge only if two independently computed doubles agree to within an ulp or two (about 1e-12 m at these magnitudes). That is a floating-point accident with negligible probability per chart, not something that happens by construction. It is systematic only for grids built aligned to the map frame: the tests (#56 and #57 keep their charts a quarter cell off for this reason), scratch harnesses, or a future change that snaps chart grids to the costmap grid.

Not established: no NOAA cell was run through `s57_grids` here (the TF to a map frame was not set up). The conclusion rests on reading the code, which has no snapping step. Urgency: low on a vessel, but the effect, when it happens, is heap corruption and a crash of the costmap node, so it should be fixed. The owner has said no boats are running in the near future.

#### Fix options (all three were built and run in the harness over 432 scenarios: 1, 0.5 and 0.1 m; 4-, 20- and 100-cell tiles; chart cells 1x and 2x the tile cell; three origins; four alignments; 0% and 20% NaN)

| Option | Out-of-range accesses | East-edge wrap | Interior coincidence | Output where no edge coincides |
|---|---|---|---|---|
| Today | 270689 | yes | the touching neighbour is claimed; spreads into NaN cells | |
| (a) clamp the far indices to `tile_size - 1` | 0 | gone | unchanged: over the scan 3368867 cells differ from the expectation, exactly as today with the out-of-range accesses skipped (736279 of them spread into cells without data, the rest the rounding effect below) | identical to today in every scenario (0 cells differ) |
| (b) half-open: `lo = max(0, floor((a - o)/r))`, `hi = min(T - 1, ceil((b - o)/r) - 1)` from the tile's own origin and resolution | 0 | gone | a chart cell that only touches a tile cell paints nothing; spread at exact coincidence gone (e.g. 31408 → 0 at 1 m, 100-cell tiles, 20% NaN; over the scan 2951484 cells differ, 396989 of them spread, the rest the rounding effect below) | identical to today in every scenario |
| (c) a conversion that cannot return `size` (`worldToMap`, which fails on the edge; or sampling the chart at each tile-cell centre) | | | | `worldToMap` needs a fallback for a failed corner, which brings back (a) or (b). Centre sampling changes the result everywhere a chart cell edge cuts a tile cell, and drops chart cells finer than a tile cell (a small rock). Not a minimal fix. |

(a) and (b) also avoid the near-corner trap above: neither clamps the near index downward.

Neither (a) nor (b) removes a larger effect that the scan exposed, which is not #63. A tile cell straddled by two chart cells takes the east (or north) one's value, however small its share. When chart cell edges lie a hair west or south of the tile-cell edges, each chart cell claims a sliver of the cell to its west, and the whole tile then shows its chart shifted one cell west and south. That holds from offsets of -1e-9 to -1e-6 cell at 1 m: 79859 of 160000 cells differ from the expectation, and 0 at +1e-9. At non-binary resolutions or origins, "aligned" charts are aligned only to within rounding, so this decides most cells there. The rows at origin 3.7 and at 0.1 m show it under every option. With real charts the straddled cells sit wherever chart edges fall, so hazard boundaries move by up to one costmap cell, toward the south-west. This is the compositing rule, not an out-of-range access, and it is outside this issue.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Test what breaks | Watch | A value test catches the east-edge write (the wrapped cell differs when the pattern differs). The north-edge access writes only when the byte past the buffer reads 255, so no value test fails on it reliably. Only a bounds-checking accessor or ASan sees it. At 0.1 m no chart is exactly aligned, and the straddle rule makes a cell-centre expectation fail with or without the fix. |
| A change includes its consequences | Watch | `test_rolling_window.cpp:519-522` and `test_initialize.cpp:414-416` give #63 as a reason for the quarter-cell offset. After the fix that reason is gone (the #59 one stays). |
| Only what's needed | OK | One loop. The straddle rule and `ais_layer`'s reasoning-only guards stay out. |
| Verify before claiming | Watch | The issue's condition, line numbers and the ASan and crash statements need the corrections above. The issue text is the owner's, so they go in this comment, not an edit. |
| Small steps; step back when they stop converging | OK | Fits one commit for a `git cherry-pick -x` backport to `jazzy`. |
| Leave in a project only what it chose to carry | OK | Records go in `.agent/work-plans/issue-63/` in the PR to `rolling`, as settled. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0013 — progress.md entry types | Yes | This review is an `## Issue Review` entry. |
| 0002 — Worktree isolation | Yes | Work is in the issue's worktree. |
| 0019 — `repo_pin` | Yes | Base `rolling`, then a hand backport to `jazzy`, as settled. |
| 0017 — Design document is the current picture | No | No design change; the project keeps no design document. |

### Consequences

- The two test comments named above.
- `s57_layer/README.md` does not describe tile compositing. Nothing to update.
- The Jazzy backport: the loop and the Nav2 functions are identical on `jazzy`, so the commit should cherry-pick cleanly. The Jazzy CI job is the only Jazzy check (not installed here).

### Recommendations

- Fix with (b), half-open tile-cell ranges with the far index clamped to `tile_size - 1` and the near index clamped only up to 0, computed from the tile's own origin and resolution. It removes the out-of-range accesses, the east-edge wrap, and the one-cell spread at coincident interior edges, and it leaves every non-coincident case identical to today. (a) is the smallest change that stops the memory errors and the wrap. It leaves a touching chart cell able to claim a tile cell that its own chart cell left empty. If the planner chooses (a), say so and why.
- Test at binary-exact geometry (1 m and 0.5 m, plus 0.25 m if cheap, integer origins) with a chart whose cell edges lie on a tile's east edge, its north edge, both, and interior cell edges only (chart cells 2x the tile cell, offset by a cell). Use a land/water pattern that differs across each tile edge and some NaN cells, and check every cell against an expectation from the chart cell under each cell centre. Without the fix the east case fails by value; with (b) the NaN cells fail without it too. For the north-edge access, either define bounds-checking `Costmap2D::getCost/setCost(mx, my)` in the test executable and assert zero out-of-range calls (they are not inline on Jazzy or Rolling, so the test's definitions take the place of Nav2's), or state in the plan that only the scratch harness and ASan check it. At 0.1 m, assert at most that no access is out of range; a cell-by-cell expectation there depends on rounding (see the straddle note).
- Leave the quarter-cell offsets in the #56/#57/`test_initialize` charts as they are, since they also keep those tests independent of the straddle rule. Edit their comments to drop #63 as a reason. `AboveOriginControl` and `NonBinaryResolutionOffsetRoundsToNearestCell` trigger the defect harmlessly. With the bounds-checking accessors in that executable they become checks of the fix as well.
- Keep the order #63 then #59. The fix touches `:628-638`, and #59 touches the `SubmapGeometry` call at `:609-617`, so the two hunks are separate and cherry-pick independently. #59's tests will put chart edges on tile edges (a chart boundary inside a tile), which this defect would confound if it were still there.
- Owner's decision: whether to have an issue drafted for the straddle rule (a tile cell cut by a chart cell edge takes the east/north chart cell's value however small its share, so the whole tile shows the chart shifted one cell south-west when chart edges lie just south-west of the tile-cell edges). It is a pre-existing compositing choice, not this defect, and changing it changes output everywhere. Per the port rule it gets its own issue for his approval or is left alone.

### Actions
- [ ] Fix with (b), half-open tile-cell ranges with the far index clamped to `tile_size - 1` and the near index clamped only up to 0, computed from the tile's own origin and resolution. It removes the out-of-range accesses, the east-edge wrap, and the one-cell spread at coincident interior edges, and it leaves every non-coincident case identical to today. (a) is the smallest change that stops the memory errors and the wrap. It leaves a touching chart cell able to claim a tile cell that its own chart cell left empty. If the planner chooses (a), say so and why.
- [ ] Test at binary-exact geometry (1 m and 0.5 m, plus 0.25 m if cheap, integer origins) with a chart whose cell edges lie on a tile's east edge, its north edge, both, and interior cell edges only (chart cells 2x the tile cell, offset by a cell). Use a land/water pattern that differs across each tile edge and some NaN cells, and check every cell against an expectation from the chart cell under each cell centre. Without the fix the east case fails by value; with (b) the NaN cells fail without it too. For the north-edge access, either define bounds-checking `Costmap2D::getCost/setCost(mx, my)` in the test executable and assert zero out-of-range calls (they are not inline on Jazzy or Rolling, so the test's definitions take the place of Nav2's), or state in the plan that only the scratch harness and ASan check it. At 0.1 m, assert at most that no access is out of range; a cell-by-cell expectation there depends on rounding (see the straddle note).
- [ ] Leave the quarter-cell offsets in the #56/#57/`test_initialize` charts as they are, since they also keep those tests independent of the straddle rule. Edit their comments to drop #63 as a reason. `AboveOriginControl` and `NonBinaryResolutionOffsetRoundsToNearestCell` trigger the defect harmlessly. With the bounds-checking accessors in that executable they become checks of the fix as well.
- [ ] Keep the order #63 then #59. The fix touches `:628-638`, and #59 touches the `SubmapGeometry` call at `:609-617`, so the two hunks are separate and cherry-pick independently. #59's tests will put chart edges on tile edges (a chart boundary inside a tile), which this defect would confound if it were still there.
- [ ] Owner's decision: whether to have an issue drafted for the straddle rule (a tile cell cut by a chart cell edge takes the east/north chart cell's value however small its share, so the whole tile shows the chart shifted one cell south-west when chart edges lie just south-west of the tile-cell edges). It is a pre-existing compositing choice, not this defect, and changing it changes output everywhere. Per the port rule it gets its own issue for his approval or is left alone.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 19:28 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the issue review confirmed the defect and the scope; its corrections are in its comment on the issue; the choice between its two fixes is a mechanism choice inside the issue's goal)
**After**: issue-actions
**Decision**: proceed

Read: the Issue Review, status complete. Scope stands: no out-of-range access in `generateTile`'s chart-cell loop (`s57_layer.cpp:628-648`), with a test. For the planner: (1) **the fix is the review's option (b): half-open ranges, `floor` for the near index and `ceil - 1` for the far one, the far index clamped to `tile_size - 1` and the near index clamped only up to 0** (never clamp the near index down to `tile_size - 1`: a cell just outside the east edge would then paint the last column and, visited first, win). It is chosen over the plain clamp because the plain clamp leaves a chart cell that only touches a tile cell's edge claiming that cell, which is the same mistake at interior edges; **on the condition that the planner shows the output is cell-for-cell identical to today's wherever no chart cell edge coincides with a tile-cell edge** (the review's 432 scenarios, plus real-chart-like grids at non-integer offsets and resolutions, including chart cells finer and coarser than tile cells). If that cannot be shown, fall back to the plain clamp (a) and say so. (2) Tests: binary-exact geometry (1 m and 0.5 m, integer origins) with chart cell edges on the tile's east edge, north edge, both, and interior edges only; a land and water pattern with some no-data cells; every cell asserted against an expectation computed independently of the layer; **bounds-checking copies of `Costmap2D::getCost`/`setCost` defined in the test executable so an out-of-range access fails the test even when the value happens to be right** (check this links and takes effect on Jazzy as on Rolling: both declare them non-inline; the Jazzy CI job is the first Jazzy run); at 0.1 m, where exact alignment cannot be constructed, assert only that no access is out of range. Each test shown failing on today's source. (3) The comments in `test_rolling_window.cpp` and `test_initialize.cpp` that explain their quarter-cell offsets by this defect are updated; the offsets stay. (4) Not in this issue: rolker/s57_tools#59 (next; its hunk is the `SubmapGeometry` call just above); and the review's separate finding that a tile cell cut by a chart cell edge takes the east or north chart cell's value however small its share, so chart edges a hair off the tile-cell edges shift the painted chart by a cell: the host puts that to the owner.

## Plan Authored
**Status**: complete
**When**: 2026-11-24 19:39 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Plan**: `.agent/work-plans/issue-63/plan.md` at `c7479c9`

Option (b): each chart cell's tile-cell range from half-open intervals (near `max(0, floor(q))`, far `min(tile_size - 1, ceil(q) - 1)`), replacing the two `worldToMapEnforceBounds` calls. Measured identical to today over 20000 random real-chart-like cases (0 differing cells, 0 coincidences); in 3000 near-aligned cases every difference where no quotient is an integer (303 cells) traces to a chart cell where today's code accesses out of range or paints from a chart cell wholly outside the tile. Tested in a new `test_tile_edges` executable that interposes bounds-checking `Costmap2D::getCost`/`setCost` (checked sound on Rolling and on the Jazzy 1.3.14 binary, and self-checking by call count); prototype fails 5 of 6 tests on today's source and passes with the fix.

## Plan Review
**Status**: complete
**When**: 2026-11-24 19:47 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: ready

**Issue**: #63 — s57_layer generateTile reads and writes one cell past a tile row when a chart cell edge lies exactly on the tile edge
**Plan**: `.agent/work-plans/issue-63/plan.md` at `c7479c9`
**Branch**: `feature/issue-63`

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | One loop in hazard-marking code (`s57_layer.cpp:628-638`), one new test executable, two comment rewordings. One code commit, cherry-pickable to `jazzy` (`s57_layer/src` is identical on `rolling` and `origin/jazzy` at `c8f253e`). |
| Issue alignment | Good | Option (b), as the checkpoint chose. The checkpoint's condition is met; independently re-measured below. The issue review's recommendations are taken: test comments, bounds-checking accessors, access-only at 0.1 m, offsets kept. Its "0.25 m if cheap" is not taken (optional). Serves: "The project keeps no goals or design document; in practice: vessels' costmaps show charted hazards where they are charted, and the costmap node does not corrupt its memory." Checked, and accepted for a project with no goals written down. |
| File targeting | Good | `s57_layer.cpp`, new `test/test_tile_edges.cpp`, `CMakeLists.txt`, and the two test comments. `grep` finds no other `worldToMap*`/`getCost`/`setCost` use in the layer. `updateCosts` is already half-open (issue review). |
| Consequences | Good | README says nothing on compositing. #59 is a separate hunk. The two test comments are included. |
| Principle alignment | Good | Test what breaks: each case fails on today's source by value or by access (planner's prototype; see finding 3 on keeping the expectation independent). Only what's needed: four index lines. Design picture: none; the project has no design document. Prior art: the issue review's harness, and the half-open ranges `updateCosts`/`bathymetry_layer` already use. |
| ADR compliance | Good | 0013 (entries), 0002 (worktree), 0019 (base `rolling`, `cherry-pick -x` to `jazzy`). 0017 is not triggered. |
| ROS conventions | Good | No topic, QoS or parameter change. Nav2 `Costmap2D` formulas are identical on both distros (issue review). The test fixture is a fixed, non-rolling costmap, so the Jazzy/Rolling rolling-window size difference cannot enter. |

### Independent checks

**Arithmetic.** `world_min_x` (`x_origin_ + id.first*resolution_*tile_size_`) and `resolution_` are the exact values passed to the tile's `Costmap2D` constructor, which stores them unchanged (`costmap_2d.cpp:53`). `worldToMap*` computes `(wx - origin_x_)/resolution_`. The planned quotient is therefore bit-identical to Nav2's. Loop variables are `int`, so `tile_min > tile_max` gives no iterations. Every case below was checked:
- a chart cell wholly inside;
- straddling each edge;
- larger than the tile: `[0, T-1]`;
- touching from outside: empty;
- touching from inside: only the overlapped cells;
- zero width: one cell if `q` is not an integer, none if it is.

2,000,000 random chart cell edges within ±3 ulps of a tile or tile-cell edge (origins -12.3 and ±1e5, resolutions 1, 0.5, 0.25, 0.1, 0.05, 0.3, tiles 1, 4 and 100) gave 0 indices outside `[0, T-1]` in a non-empty loop. Today's far index came out as `T` 27,815 times. NaN coordinates give the whole tile (`std::max(0.0, NaN)` is 0 and `std::min(last, NaN)` is `last`), where today's code has an undefined cast. The clamps are one-sided (finding 1).

**Equivalence, own harness.** Rolling's real `Costmap2D` (`worldToMapEnforceBounds` for today's loop) and real grid_map `SubmapGeometry`/`SubmapIterator`. Both loops run against checked accessors, with the east wrap kept visible by value. The expectation is computed in `long double` in chart-index space, measured from the chart's east/north edge (positive-area overlap, data, east-most then north-most, finest chart first). It ignores any chart whose `SubmapGeometry` fails, so #59 does not count as a difference.

The generator differs from the planner's:
- resolutions 1, 0.5, 0.25, 0.1, 0.05 m; tiles 1, 4 and 100;
- chart/tile cell ratios log-uniform in 0.2-40, plus 0.03-0.13 and 2-7 tile widths;
- one to three charts per case, each partial on any side;
- layer origins 0, ±10 m, ±1e6 m and -1e5..-2e5 m;
- tile IDs ±20 or down to -3000;
- 20% no data.

| Set | Cases | Tile cells | Differing cells | Today wrong vs expectation | Planned wrong | Out of range today / planned |
|---|---|---|---|---|---|---|
| Random, seed 4242 | 20000 | 66.2 M | **0** | 0 | 0 | 0 / 0 |
| Near-aligned (snapped to tile-cell edges, jitter 0, ±1 ulp, ±1e-12, ±1e-9 m), seed 4243 | 6000 | 20.1 M | 74081 in 590 cases | 1386 unambiguous + rest within rounding | 0 unambiguous | 30840 / 0 |
| Near-aligned, seed 99 | 6000 | 19.6 M | 74834 in 644 cases | 2557 unambiguous | 0 unambiguous | 38824 / 0 |

Of the differing cells, the planned code was right in every one where no chart cell edge lies within 1e-7 cell of the tile cell's edge (1386 and 2557). Today's code was never right where the planned code was wrong. The rest (72695 and 72277) are within rounding of an edge, where the "right" owner depends on rounding.

Per case, the planner's statement holds exactly. In every case where today's code makes no out-of-range access and writes no tile cell from a chart cell whose own double quotients say it only touches that cell or lies outside the tile (4873 of 6000 and 4794 of 6000), not one cell differs.

**Chart cells wholly outside the tile.** These are the planner's second and third categories.
- **Random geometry:** 0 writes in 66.2 M cells.
- **Near-aligned geometry:** the largest gap between such a chart cell and the tile cell it painted was 1.4e-9 tile cells. It is never a full cell away.
- **Mechanism:** `getSubmapInformation` finds each corner's chart cell as `trunc((map_east - corner)/res)` (`GridMapMath.cpp:156`, `:74-77`).
  - The tile's west/south corner lying exactly on a chart cell edge selects the chart cell beyond it. This happens by construction at exact alignment.
  - At the east/north corner it happens only when rounding goes the other way.
  - `boundPositionToRange`'s epsilon applies only at a chart's own boundary, and moves the corner inward.

So such a chart cell touches the tile only within rounding and overlaps it with zero area. Not painting from it is clearly correct.

**Painting rule.** `incrementIndexForSubmap` (`GridMapMath.cpp:542-548`) runs index 1 (north to south) inner and index 0 (east to west) outer. With write-if-unknown, the first chart cell with data that the loop reaches wins. The fix changes neither the order nor the write rule. It changes only the set of tile cells a chart cell may claim, to those it overlaps with positive area (in the double quotient).

**Interposition.**
- **On this machine:** a scratch shared library calling `Costmap2D::getCost`/`setCost` gets `R_X86_64_JUMP_SLOT` relocations, and an executable defining both exports them `GLOBAL DEFAULT` and receives the calls. This holds with `-Wl,--as-needed`, with `-flto=auto`, and with `-Wl,-Bsymbolic-functions` on the executable; the latter two are Ubuntu resolute's `dpkg-buildflags` LDFLAGS here.
- **`-Bsymbolic-functions` and LTO:** they bind only a library's calls to its own definitions. The layer defines neither function, so its calls are interposed whatever flags Nav2 or the layer is built with. Static linking of Nav2 would be a duplicate-symbol link error. Nav2 making them inline in the header would be a redefinition compile error. Both failures are loud.
- **Standard and platform:** by the C++ standard this is an ODR violation (no diagnostic required). Under ELF it is defined symbol interposition, the same mechanism as malloc replacement.
- **The remaining silent failure:** a platform where the executable's definition is not preferred, for example macOS two-level namespaces or Windows; neither is a target. The call-count assertion turns that into a failure. A CI failure for that reason is acceptable.
- **Correction to the plan:** on this machine's Rolling underlay, `libnav2_costmap_2d_core.so` *does* call both through its own PLT (`JUMP_SLOT` at `0x1fa280`/`0x1faaa8`, no `DT_SYMBOLIC`). Nav2-internal calls are interposed too, which is harmless (finding 2).

Recommendation: keep the interposition as planned. It checks every access the hazard path makes, wherever the arithmetic goes wrong, including future changes from #59. An extracted `tileCellRange()` pure function would test only the arithmetic, would not have shown the north overrun, and changes hazard-marking code's structure for a test. It is not needed. ASan builds would duplicate the build.

### Findings

1. **[Principle alignment: arithmetic], minor.** The clamps are one-sided. The plan's sentence "Clamping in `double` before the cast keeps the cast in range" is not true as stated:
   - `tile_min` is unbounded above and `tile_max` is unbounded below;
   - `q_near` = 1e300 or `+inf`, or `q_far` = `-inf`, reaches `static_cast<int>` out of range, which is undefined.

   It is unreachable today, because `SubmapIterator` yields only chart cells that overlap the tile or touch it within rounding (measured gap ≤ 1.4e-9 cell). It would matter if #59 iterates a wider superset, as the plan's Consequences invites. Resolution: either clamp the other side too, at the cost of two calls in the same four lines, or make the comment state the reliance on the submap region. The two clamps are `std::min(double(tile_size_), ...)` on the near index and `std::max(-1.0, ...)` on the far one. The clamp is recommended: the cast is then defined for every double, including NaN, which gives the full tile.
2. **[File targeting: test], minor.** The plan says `libnav2_costmap_2d_core.so` "has no PLT relocations of its own for them, so Nav2-internal calls are not checked". That holds for the Jazzy binary the planner inspected, but not for this machine's Rolling underlay: Nav2's own calls are interposed there too. Harmless, but the test's comment should not repeat the claim.
   - **Why the safeguard needs tightening:** Nav2-internal calls can satisfy "at least one call", so that safeguard proves the layer's calls arrived only by inference.
   - **Resolution:** count calls on the tiles' own costmaps (`size_x_ == tile_size`, as the fixture's master costmap is 12 x 12 and the tiles 4 x 4), or assert the count is at least the number of tile cells painted.
3. **[Principle alignment: test], minor.** The expectation is described as computed "from the chart grid alone using the painting rule", but how is not specified. It must not reuse the layer's `floor`/`ceil` quotient on `(edge - world_min)/resolution`, or a shared mistake passes both. At the binary-exact 1 m and 0.5 m geometry the plan uses, compute it in integer cell counts, or in chart-index space from the chart's own edge, and say so in the test.
4. **[Issue alignment], info.** The equivalence statement for the pull request, as checked here:

   > Output is identical to today's in every tile where today's code makes no out-of-range access and writes no tile cell from a chart cell that, by the same double quotients, only touches that cell or lies outside the tile. Such chart cells arise only where a chart cell edge lies within floating-point rounding of a tile or tile-cell edge: never in 66.2 M cells of non-aligned geometry, and never more than 1.4e-9 cell away in near-aligned geometry. Where the output differs, the new value is the positive-area owner in every cell not itself within rounding of an edge.

### Summary

The fix is correct and bounded for every input that can reach it. The checkpoint's equivalence condition is independently confirmed: 0 differing cells over 66.2 M random, real-chart-like tile cells, and every unambiguous difference in near-aligned geometry favours the new code. The wholly-outside-the-tile painting that it removes happens only at zero-area contact within rounding. The interposition test is sound on ELF and fails loudly where it would not take. Ready for implementation; the three minor findings can be folded in while implementing.

### Recommended Actions

- [ ] Clamp `tile_min` above (to `tile_size_`) and `tile_max` below (to -1) in `double` before the casts, or reword the comment to state the reliance on `SubmapIterator`'s region.
- [ ] In `test_tile_edges`, drop the "Nav2-internal calls are not checked" claim, and make the "at least one call" assertion specific to tile-sized costmaps (or to the number of painted cells).
- [ ] Compute the test's expectation without the layer's quotient arithmetic (integer cell counts at binary-exact geometry), and say so in the test.
- [ ] Use the equivalence statement in finding 4 in the pull request.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 19:48 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the plan review's verdict is ready; its findings are implementation details; its independent run confirmed the condition the host set on the half-open form)
**After**: plan
**Decision**: proceed

Read: the Plan Review at `c7479c9`, verdict ready, three minor findings and an information note. The half-open form stays: the review's own comparison (20,000 random cases, 66.2 million cells, a different generator and seeds) found 0 differing cells on non-aligned geometry, and in near-aligned geometry the new code is right in every differing cell not itself within rounding of an edge. Findings applied: (1) **both indices are clamped on both sides in double before the cast** (`std::min(double(tile_size_), ...)` on the near index and `std::max(-1.0, ...)` on the far one, added to the plan's one-sided clamps), so the cast is defined for every double; the comment says so and does not rely on the submap region. (2) The test's "at least one call" safeguard is made specific to the tiles: calls are counted on costmaps of the tile's size, or the count is at least the number of tile cells painted, since on Rolling Nav2's own calls are interposed too; the test's comment does not claim otherwise. (3) The expectation is computed in integer cell counts or in chart-index space from the chart's own edges, never with the layer's quotient, and the test says so. (4) The pull request carries the review's equivalence statement. The out-of-range check stays as the plan has it (bounds-checking accessors defined in the test executable); an extracted arithmetic function was weighed and not taken, because it would not have shown the north-edge access and restructures hazard-marking code.

## Implementation
**Status**: complete
**When**: 2026-11-24 20:07 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Branch**: feature/issue-63 at `4250a7f`

Code commit `4250a7f` "s57_layer: half-open tile-cell ranges in generateTile (#63)" (no `.agent/` file; applies cleanly to `origin/jazzy` at `c8f253e`, where `s57_layer` is identical).

- `s57_layer/src/s57_layer.cpp`: the two `worldToMapEnforceBounds` calls replaced by `near = max(0.0, min(double(tile_size_), floor(q_near)))` and `far = min(tile_size_ - 1.0, max(-1.0, ceil(q_far) - 1.0))`, `q = (edge - world_min)/resolution_`; loops, write-if-unknown and the `SubmapGeometry` call unchanged. Clamped on both sides in double (checkpoint finding 1). **NaN quotient: empty range, deliberately** (the inner `min`/`max` returns its first argument, the bound beyond the tile: near = `tile_size_`, far = -1); said in the comment. Not constructible through the layer: `SubmapGeometry` on a chart with a NaN position returns `ok=false` (checked), so the loop is never reached.
- `s57_layer/test/test_tile_edges.cpp` (new, own `ament_add_gtest` entry after `test_rolling_window`'s): fixed 12 x 12 `LayeredCostmap`, `track_unknown` on, 4-cell tiles, origin (-8, 4) cells; chart land / 10 m water / ~20% no data by a hash of the chart index, 3 cells to spare. 13 tests: east, north, both, interior-only (2-cell chart cells offset 3 cells), quarter-cell control, each at 1 m and 0.5 m (value + access); 0.1 m at origin (-12.3, 4.7), chart cells 0.5, 1, 2 cells, starting 4 cells out so each has edges on tile edges (access only). Expectation in integer quarter cells from the chart's own edges, never the layer's quotient (said in the test). Interposed bounds-checking `Costmap2D::getCost(unsigned, unsigned) const` / `setCost(unsigned, unsigned, unsigned char)`; each test asserts zero out-of-range and that `setCost` calls on 4 x 4 costmaps are at least the painted cells (checkpoint finding 2).
- `test_rolling_window.cpp` and `test_initialize.cpp`: comments reworded; offsets kept.

Deviation from the plan: the 0.1 m charts start 4 cells (not 3) out, because with 2-cell chart cells 3 cells out every edge falls on an odd cell edge, never a tile edge, and that case passed on today's source.

### Failing on today's source (scratch build, `ldd` showed the scratch `libs57_layer.so` loaded)

- East 1 m / 0.5 m: `116 of 1084 tile accesses out of range; first getCost(4, 3) in a 4 x 4 costmap`; `19 of 144 cells differ from the expectation; first (0, 1) is 254, expected 0`
- North 1 m / 0.5 m: `150 of 1153 ... first getCost(3, 4)`; `20 of 144 cells differ; first (7, 0) is 0, expected 254` (interior spread on the coincident y edges)
- Both 1 m / 0.5 m: `278 of 1246 ... first getCost(4, 3)`; `44 of 144 cells differ; first (7, 0) is 0, expected 255`
- Interior edges only 1 m / 0.5 m: `20 of 144 cells differ; first (7, 0) is 0, expected 255` (no out-of-range access)
- Control 1 m / 0.5 m: pass
- 0.1 m: `167 of 1710` (0.5-cell chart), `135 of 745` (1-cell), `26 of 365` (2-cell) tile accesses out of range

With the fix: 13 of 13 pass; 30 of 30 separate runs pass. Suite (`colcon test --packages-select s57_layer --parallel-workers 1 --ctest-args -j1`, the freshly built library loaded): 95 results, 0 failures, 9 skipped (all cppcheck's, skipped by ament here). That is 73 gtests (60 existing + 13 new), lint_cmake 1, xmllint 2. pre-commit is not installed here; changed files checked by hand for trailing whitespace and end of file. Build: no new warnings under the package's `-Wall -Wextra -Wpedantic` (GCC 15.2; GCC 13 not installed).

### Equivalence on the final code

Scratch harness: today's loop copied verbatim (checked by `diff`) and the final loop (checked by `diff`), plus the plan's one-sided version, against the real Rolling `Costmap2D` and grid_map `SubmapGeometry`/`SubmapIterator`, with interposed checked accessors.

- **Random, seed 63, 20,000 cases** (19,993 with a successful `SubmapGeometry`): 52,208,206 tile cells, resolutions 1/0.5/0.25/0.1/0.05/0.3, tiles 1/4/20/100, 1-3 charts at ratios 0.03-0.13, 0.2-40, or 2-7 tile widths, some partial, origins 0, ±10, ±1e6, -1e5..-2e5, tile IDs ±20 or down to -3000, 0% or 20% no data. **0 differing cells.** Out-of-range accesses: today 0, new 0. No zero-area write by today's code.
- **Near-aligned, seeds 6364 and 99, 6,000 cases each** (chart edges snapped to tile-cell edges, jitter 0, ±1 ulp, ±1e-12, ±1e-9 m; resolutions 1, 0.5, 0.3, 0.1, 0.05; tiles 4, 20, 100; ratios 0.5, 1, 2, 3; origins at 0.1 m multiples in ±1000 m):
  - 20,462,960 / 20,357,456 cells; 416,756 / 427,644 differing cells in 2,270 / 2,307 cases.
  - Out-of-range accesses: today 198,094 / 200,298, new 0 / 0.
  - Clean cases (today makes no out-of-range access and no zero-area write): 3,059 / 2,997, with **0 differing cells**.
  - Differing cells with no data chart cell within 1e-7 cell of the cell's edge, ahead of the owner: 2,584 / 2,651. In all of them the new value is the long-double positive-area owner, and today's is not.
  - The rest (414,172 / 424,993) are within rounding of an edge.
- **Two-sided vs the plan's one-sided clamps:** 0 differing cells, 0 out-of-range accesses in all three sets.

**Crash reproduction** through the real layer, Release, no interposition, 400 x 400 cells, 100-cell tiles (16 per round), chart cells aligned:

| Library | 1 m | 0.5 m |
|---|---|---|
| Today, 20 rounds per run | 0 of 8 runs died | 2 of 8 died (one `malloc(): invalid next size` abort, one SIGSEGV) |
| Today, 40 rounds per run | 7 of 20 died | 8 of 20 died |
| Today, offset 0.37 cell | 0 of 8 died | 0 of 8 died |
| Fixed, same runs (aligned and offset) | 0 of 36 died | 0 of 36 died |

### Jazzy, by reading

`origin/jazzy` `c7046550` of navigation2:

- `costmap_2d.hpp` declares `unsigned char getCost(unsigned int mx, unsigned int my) const;` (:149) and `void setCost(unsigned int mx, unsigned int my, unsigned char cost);` (:164). Both are only declared there and defined in `costmap_2d.cpp` :264/:274 with the bodies the test copies, exactly the signatures it defines.
- `getIndex` is inline, `my * size_x_ + mx`. `size_x_`, `size_y_` and `costmap_` are protected (:529-535).
- `LayeredCostmap(std::string, bool, bool)`, `resizeMap(..., bool size_locked = false)`, `addPlugin`, `updateMap(double, double, double)`, `getCostmap`, `getCharMap`, `getIndex`, `getOriginX/Y` and the cost constants all exist. The `layered_costmap.cpp` differences from Rolling (`current_`, the `updateMap` early return restructured, setting disabled plugins current) do not touch a fixed, single-layer, track-unknown map.
- Lifecycle node, `Layer::initialize`, `tf2_ros/buffer.h` and every grid_map call are the ones `test_rolling_window`, already run by Jazzy CI, makes. grid_map's `jazzy` branch (`d961499`) has no diff from Rolling's in `GridMap.hpp/.cpp`, `GridMapMath.cpp`, `SubmapGeometry.cpp`, `SubmapIterator.cpp` and `GridMapIterator.cpp`.

### For the pull request

- **Defect and fix:** generateTile took each chart cell's tile-cell range from `Costmap2D::worldToMapEnforceBounds`. That returns the tile size, one past the last cell, for a far corner on or within rounding of the tile's east or north edge. The read and write landed in the next row (east) or past the tile's buffer (north): heap corruption, sometimes a crash. A far corner on an interior tile-cell edge also claimed the next cell, spreading its value into a no-data neighbour. The fix uses half-open ranges, each index clamped on both sides before the cast.
- **Equivalence:** output identical to today's in every tile where today's code makes no out-of-range access and writes no tile cell from a chart cell that only touches that cell or lies outside the tile. Such chart cells arise only where a chart cell edge lies within floating-point rounding of a tile or tile-cell edge. Where the output differs, the new value is the positive-area owner in every cell not itself within rounding of an edge. Figures: 0 differing cells in 52.2 M random non-aligned tile cells; in near-aligned geometry 0 differing cells in 6,056 clean cases, and 5,235 unambiguous differing cells, all now the positive-area owner.
- **Rule after the fix:** a tile cell takes the value of the chart cell with data that overlaps it with positive area and comes first in `SubmapIterator`'s order (east-most, then north-most; finest chart first). A chart cell that only touches a tile cell's edge, lies outside the tile, or has a NaN position paints nothing.
- **For a vessel:** real chart grids are placed from latitude and longitude with nothing snapped, so exact alignment is a floating-point accident. Output changes only there. A rare heap corruption and crash of the costmap node can no longer occur.
- **Tests:** east, north and both tile edges, and interior edges, at 1 m and 0.5 m, every cell checked against an expectation in integer quarter cells; 0.1 m access only. Out-of-range access is detected by bounds-checking `getCost`/`setCost` defined in the test executable, which take Nav2's place by ELF symbol interposition. The call-count assertion fails if they ever stop doing so.
- **Not established:** Jazzy until the PR's Jazzy CI job (first Jazzy compile and run; GCC 13 not tried here). No real NOAA chart was run through the grid code. The straddle rule (owner's question) is untouched.

Build folders and scratch removed; nothing left running.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-24 20:18 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: approved

**Branch**: feature/issue-63 at `48926a6`
**Base**: origin/rolling
**Depth**: Deep (reason: index arithmetic in hazard-marking code deployed on a vehicle; full depth per handoff)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — no must-fix findings; remaining suggestions can be applied or tracked

Code commit `4250a7f` (no `.agent/` file; applies cleanly onto `origin/jazzy` `c8f253e`). Reviewers: Claude adversarial subagent, Gemini and Codex (both completed). #59's `SubmapGeometry` call untouched; files match the plan.

Verified here:
- Build (Rolling, RelWithDebInfo, `-Wall -Wextra -Wpedantic`): no new warning. Suite: 95 results, 0 failures, 9 skipped (cppcheck). `test_tile_edges` 30 of 30 runs pass.
- Old vs new loop, verbatim, against real `Costmap2D` and grid_map `SubmapGeometry`/`SubmapIterator`, with a long-double positive-area truth in chart-index space. Settings: costmap resolutions 1/0.5/0.25/0.1 m; tiles 4 and 100; chart cells 0.3-30 costmap cells (log-uniform plus simple ratios); random offsets; tile IDs ±1000; origins to ±1e5 m; land, shallow, deep and no data.
  - **Non-aligned (seeds 777, 4242, 9001): 27,000 cases, 135.9 M cells.** 0 cells lethal before and not after; 0 the reverse; 0 differing cells; old and new both equal the truth everywhere.
  - **Near-aligned (edges snapped within ±3 ulps, seed 31337): 6,000 cases.** Old made 25,724 out-of-range accesses (193 cases), new 0. Of 67,234 differing cells, the new value matches the truth in 65,450 and the old in 1,607; 25,164 cells lethal before not after (613 of them old-correct), 6,492 the reverse. Cells off the truth: new 239,582, old 303,425. All of this is within rounding.
- Removing the fix makes the named cases fail as described:
  - east: by access `getCost(4, 3)` and by value;
  - north: by access `getCost(3, 4)` and by value;
  - interior edges: by value only;
  - 0.1 m: all three by access;
  - control: passes.
- Mutations, each compiled into the test:
  - far index without `- 1`: x caught by 8 tests, y by 10;
  - `ceil` to `floor` on the far index: x caught by North + control, y by East + control (the axes are caught separately);
  - far upper clamp removed: x 9, y 9;
  - x/y swapped in one line: 10 and 10;
  - near clamps removed: 12;
  - **near clamp lowered to `tile_size_ - 1`: survives for x and y** (see the second suggestion).
- NaN, ±inf, ±1e300 and 1-3 ulps either side of an edge, traced numerically: every range is empty or correctly clamped, and every cast is defined.
- Interposition binding confirmed with `readelf`/`nm`: the layer's calls go through JUMP_SLOTs and the test exports both definitions. The signatures and bodies match Nav2 on Rolling and on `origin/jazzy`. Only the layer's tiles are 4 x 4, so Nav2 cannot satisfy the call count on its own.

### Findings
- [ ] (suggestion) Where a chart cell edge falls on a tile-cell edge within rounding, as happens at decimal resolutions, the quotient lands at k±ε. A chart cell then takes a tile cell it only touches, or skips one it covers, and land can lose a cell to water. This is older than the change, and the old code is worse (see the near-aligned figures). Charts placed from latitude and longitude (`S57Dataset::getGrid`) do not align. Say "in exact arithmetic; within rounding either neighbour may be taken, as before" in the loop comment and in the PR's rule, and offer the owner a follow-up (a tolerance, or lethal wins) beside the straddle question (Codex and the adversarial reviewer) — `s57_layer/src/s57_layer.cpp:628-647`
- [ ] (suggestion) Test gaps:
  - The near-clamp mutation (`min(tile_size_ - 1, ...)`, which paints the last column or row from a chart cell wholly east or north of the tile) passes all 13 tests. Such chart cells are iterated only in near-aligned geometry: 809 east and 377 north in 2,000 near-aligned cases, 0 in random ones.
  - There is no value case with chart cells smaller than a tile cell (`cell_q = 2`) or larger than a tile.
  - The "tile accesses out of range" message counts out-of-range accesses on any costmap (Gemini).
  — `s57_layer/test/test_tile_edges.cpp:79-87,327,395-417`
- [ ] (suggestion) The interposition relies on default ELF semantics: no LTO, `-Bsymbolic` or `-fno-semantic-interposition` on Nav2 or the layer. The call-count check catches a total failure. Name this in the CMakeLists comment — `s57_layer/CMakeLists.txt:149-152`

## Checkpoint
**Status**: complete
**When**: 2026-11-24 20:19 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the review approved with no must-fix; its three suggestions are cheap and two of them concern what a reader of hazard-marking code is told and what the tests cover, so they are taken before publishing)
**After**: publish
**Decision**: address

Read: the Local Review (Pre-Push) round 1 at `4250a7f`, approved, three suggestions, all three outside reviewers run. One pass, then a narrow re-review, then publish. (1) The comment above the loop, and the pull request's statement of the rule, say the rule holds in exact arithmetic and that where a chart cell edge lies within floating-point rounding of a tile-cell edge either neighbour may be taken, as before this change (the review's near-aligned comparison: the new value matches an independent calculation in 65,450 of 67,234 differing cells, the old in 1,607). Two outside reviewers rated that rounding behaviour a must-fix; it is older than this change, does not occur on non-aligned geometry (0 differing cells in 135.9 million), and the old code is worse there; it goes to the owner as a follow-up question beside the straddle rule. (2) Tests: value cases with chart cells smaller than a tile cell and larger than a tile; the out-of-range message counts tile accesses only, or says what it counts; and the mutation that survives every test (the near index clamped to `tile_size_ - 1` instead of `tile_size_`, which paints the last column or row from a chart cell wholly east or north of the tile) gets a test if such a chart cell can be put in front of the loop deterministically, and otherwise a comment saying that the line is exercised only at near-aligned geometry and by which recorded run. (3) The CMake comment names the linker assumptions the test's accessors rely on.
