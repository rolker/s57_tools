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
