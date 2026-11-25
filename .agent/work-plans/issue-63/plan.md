# Plan: s57_layer generateTile reads and writes one cell past a tile row when a chart cell edge lies exactly on the tile edge

## Issue

https://github.com/rolker/s57_tools/issues/63

## Context

`S57Layer::generateTile` (`s57_layer/src/s57_layer.cpp:628-648` on `rolling` at `a1ba225`, the same on `jazzy` at `c8f253e`) takes each chart cell's tile-cell range from `Costmap2D::worldToMapEnforceBounds` of its two corners and loops over it inclusively. That function clamps only a coordinate strictly beyond `resolution*size + origin`, so a far corner on (or within rounding of) the tile's east or north edge gives index `size`. The read at `:643` and write at `:645` then land in the next row (east) or past the buffer (north). The issue review re-established it, with heap corruption and SIGSEGV at 100-cell tiles. The checkpoint chose the review's option (b), on condition that it is shown to leave the output unchanged wherever no chart cell edge coincides with a tile-cell edge.

What the present code computes, per axis, with `q = (x - tile_origin)/res`: near index `0` if `x < origin`, else `trunc(q)`; far index `size - 1` if `x > res*size + origin`, else `trunc(q)`. The loop is inclusive at both ends. So a chart cell partly outside the tile on the west/south gets near `0` (right), and on the east/north gets far `size - 1` only when the comparison says it is beyond. Three cases go wrong: a far corner with `q` an integer (the cell only touches the next tile cell: it claims it; at `q = size` that is out of range); a far corner whose `q` rounds to or past `size` while the comparison says "not beyond" (out of range, no exact coincidence needed); and a chart cell wholly outside the tile that `SubmapIterator` still yields (its corner lookup rounds the other way): wholly east, both corners clamp to `size - 1` and it paints the last column, visited first so it wins; wholly west, both clamp to `0` and it paints column 0 where that cell is otherwise empty.

## Approach

1. **The fix (option (b))**, in the chart-cell loop only, replacing the two `worldToMapEnforceBounds` calls; loop bounds stay inclusive, the read-then-write-if-unknown stays:
   ```cpp
   // The tile cells this chart cell overlaps with positive area, from
   // half-open intervals: a chart cell that only touches a tile cell's edge
   // does not claim it.
   const double last = tile_size_ - 1;
   const int tile_min_x = static_cast<int>(std::max(0.0, std::floor((cell_position.x()-grid_half_resolution-world_min_x)/resolution_)));
   const int tile_min_y = ... same with y / world_min_y;
   const int tile_max_x = static_cast<int>(std::min(last, std::ceil((cell_position.x()+grid_half_resolution-world_min_x)/resolution_) - 1.0));
   const int tile_max_y = ... same with y / world_min_y;
   ```
   `world_min_x/y` and `resolution_` are the values the tile was constructed with, so `q` is the same quotient Nav2 computes. The near index is clamped only up to 0 (never down: a cell just beyond the east edge then has `min > max` and paints nothing); the far index only down to `tile_size - 1`. Clamping in `double` before the cast keeps the cast in range. `<cmath>` is included in the source and `<algorithm>` by `s57_layer.h`.
2. **New test executable `test_tile_edges`** (`s57_layer/test/test_tile_edges.cpp`, CMake entry like `test_tide_offset`'s). Design below.
3. **Reword the comments** at `test_rolling_window.cpp:519-522` and `test_initialize.cpp:414-416`: the quarter-cell offsets stay; the reason becomes "keeps chart cell edges off tile-cell edges, where which chart cell a straddled tile cell takes depends on rounding" (and #59 stays as the other reason). No change to the #57 strip code or to any offset.
4. Verify on Rolling by hand: `build.sh --packages-up-to s57_layer` and `test.sh --packages-select s57_layer` (all 88 existing tests plus the new ones), `ldd` the test to see it loads the freshly built `libs57_layer.so`; show the new tests failing with the fix reverted. Jazzy: the PR's Jazzy CI job.

### Equivalence (the checkpoint's condition), measured

Scratch harness: today's loop copied verbatim and (b), both in one executable against real `grid_map::SubmapGeometry`/`SubmapIterator` and real `Costmap2D`, bounds-checking accessors in the executable, cell-by-cell comparison. "Coincident" = some visited chart cell has an edge whose quotient `q` is an exact integer in `[0, size]`.

| Set | Cases | Coincident cases | Differing cells where not coincident | Out of range, new |
|---|---|---|---|---|
| Random, seed 63: chart/tile cell ratios 0.3, 0.5, 1, 1.7, 2.5, 10; costmap 1, 0.5, 0.1 m; tiles 4 and 100; random non-integer chart offsets; layer origin 0, random in ±1e6 m, or small negative with tiles 1e6 m away (east and west); 0% or 20% no-data, land/shallow/deep values; a quarter of charts partial | 20000 (100.7 M tile cells) | 0 | **0** | 0 |
| Near-aligned: chart edges nominally on tile-cell edges, origins at random 0.1 m values in ±1000 m, resolutions 1, 0.5, 0.3, 0.1, 0.05, chart ratios 0.5-3, jitter 0, ±1e-12, ±1e-9 m | 3000 | 1480 | 303 cells in 52 cases | 0 |

The 303 cells were investigated chart cell by chart cell: every one comes from a chart cell where today's code is wrong, and none from anything else (0 "other"): far index `== size` with a non-integer `q` (out-of-range access within rounding of the edge; 236 index mismatches), a chart cell wholly east of the tile painting the last column (365), a chart cell wholly west/south painting column/row 0 (140). Reproducing those would mean keeping the out-of-range access or the near-corner trap, which the checkpoint rules out; the plain clamp (a) would differ from today in the first kind too. So (b) is identical to today wherever today's code makes no out-of-range access and paints no tile cell from a chart cell that only touches or misses it. **(b) stands; no fallback to (a).**

### What changes

- **East/north tile edges.** Before: the east-edge chart cell's write lands in `(0, j+1)` (next row, first column) and wins whenever that cell's own chart cell disagrees; the north-edge access is `x+1` bytes past the buffer. After: no access outside `[0, size)`; `(0, j+1)` takes its own chart cell's value.
- **Interior coincident edges.** A chart cell whose far edge lies on a tile-cell edge no longer claims the next tile cell. Visible only where that cell's own chart cell has no data. 4x4 tile at 1 m, origin (0,0), 2 m chart cells with edges at 1 and 3, the chart cell over `[1,3)²` no data (rows north to south):
  ```
  before             after = independent expectation
  254   0   0 254    254   0   0 254
    0   0   .   0      0   .   .   0
    0   0   0   0      0   .   .   0
  254   0   0 254    254   0   0 254
  ```
  Before, three of the four no-data cells took a west or south neighbour's value; the fourth survived only because a near edge does not claim. Over 216 binary-exact aligned cases (1 and 0.5 m; 4- and 20-cell tiles; chart ratios 0.5, 1, 2; shifts 0, 1 cell, quarter cell; 0%/20% no-data): today differs from the expectation in 2599 cells, (b) in 0, with 7376 out-of-range accesses before and 0 after.
- **On a vessel:** real chart grids are centred on the chart's lat/lon box and never snapped to the costmap grid, so a chart cell edge lands within rounding of a tile-cell edge only by a floating-point accident (review). Cells change only there; otherwise nothing observable changes except that a rare heap corruption can no longer happen.

### The painting rule after the fix

A tile cell takes the value of the chart cell with data that overlaps it with positive area and comes first in `SubmapIterator`'s order: east-most, then north-most (grid_map's index 0 runs east to west and is the outer loop, index 1 north to south inner; confirmed in `incrementIndexForSubmap`), because the loop writes only cells still `NO_INFORMATION`. Charts are taken finest first, so a coarser chart fills only cells the finer ones left. A chart cell whose edge lies exactly on a tile-cell edge contributes nothing to the cell it only touches. (Unchanged and outside #63: the east/north chart cell wins however small its share; context for the host's separate question.)

### Test design (`test_tile_edges`)

- A fixed `LayeredCostmap` (not rolling, `track_unknown` true so unpainted cells read 255), 12 x 12 cells, tiles of 4, origin (-8, 4) cells; the chart given through a `S57LayerForTest::addChart` as in `test_rolling_window.cpp`; one `updateMap`. Chart: land (elevation 1, cost 254), 10 m water (0), no data (NaN) by a hash of the chart cell index, ~20% no data, covering the window with 3 cells to spare (so #59 plays no part).
- Cases, each at 1 m and 0.5 m: chart cell edges on the tiles' east edges (x aligned, y a quarter cell off); north edges; both; interior edges only (2-cell chart cells offset one cell, so their edges fall on odd cell edges, never on a tile edge); control (quarter cell off both). At 0.1 m with origin (-12.3, 4.7) and chart cells 0.5, 1, 2 tile cells nominally aligned: access check only.
- **Expectation**, computed in the test from the chart grid alone: the rule above (positive-area overlap, data, east-most then north-most; none → 255). The harness confirms the layer follows exactly this rule wherever no edge coincides (quarter-shift aligned cases and the 20000 random ones agree with today's code).
- **Out-of-range detection:** the test file defines `nav2_costmap_2d::Costmap2D::getCost(unsigned int, unsigned int) const` and `setCost(unsigned int, unsigned int, unsigned char)`. Bodies are Nav2's (`costmap_[getIndex(mx, my)]`, identical on Rolling and Jazzy), plus a check: count every call; on `mx >= size_x_ || my >= size_y_`, count it and record the first; make the access only if the linear index is inside the buffer (so the east-edge wrap still shows by value), else skip it (read returns 255). Each test asserts zero out-of-range accesses **and at least one call**; the call count proves the layer's calls reached the test's definitions, so a platform where interposition did not take fails loudly instead of passing silently.
- **Why this is sound, checked:** `getCost`/`setCost` are declared, not defined, in `costmap_2d.hpp` on Rolling and Jazzy (`getIndex` is inline, so it is compiled into each caller; that is harmless because the check sits in `getCost`/`setCost`, which `generateTile` uses for every tile access. `updateCosts` reads tiles through `getCharMap()` and is not checked; the review showed its ranges are in bounds); `libnav2_costmap_2d_core.so` exports them `GLOBAL DEFAULT` with no `DT_SYMBOLIC`/`-Bsymbolic` on Rolling (the workspace underlay) and Jazzy (the `ros-jazzy-nav2-costmap-2d` 1.3.14 noble binary from packages.ros.org, as the Jazzy CI job installs); `libs57_layer.so` calls both through `R_X86_64_JUMP_SLOT` relocations; the test executable's `.dynsym` exports both (GNU ld exports an executable's definition that a linked shared library references), and the dynamic linker searches the executable first. `libnav2_costmap_2d_core.so` has no PLT relocations of its own for them, so Nav2-internal calls are not checked, which does not matter: the layer's are. Calls from other libraries in this process (none use these tiles) see Nav2's behaviour unchanged in range. One executable of its own keeps this away from the other tests. Alternatives weighed: a seam in the layer (free function or accessor template) changes hazard-marking code's structure for a test; compiling the layer source into the test duplicates the build; neither needed since interposition is sound and self-checking.
- **Prototype run on today's source (Rolling)**, each failing at 1 m and 0.5 m alike:
  - east edges: `183 of 1259 accesses out of range; first (4, 3) in a 4 x 4 costmap`; `14 of 144 cells differ from the expectation; first (0, 1) is 0, expected 254`
  - north edges: `212 of 1336 accesses out of range; first (3, 4)`; also `8 of 144 cells differ; first (7, 0) is 254, expected 0` (the interior-edge spread into no-data cells, since y edges coincide everywhere; the north overrun itself shows only by access)
  - both: `423 of 1499 accesses out of range`; `33 of 144 cells differ`
  - interior edges only: `7 of 144 cells differ; first (9, 0) is 254, expected 255` (no out-of-range access)
  - control: passes; 0.1 m: `110 of 727` / `118 of 648` / `335 of 2163 accesses out of range` (chart cell 1, 2, 0.5 tile cells)
  - With (b) applied: all 6 pass, and the existing 88 s57_layer tests pass.

## Files to Change

| File | Change |
|------|--------|
| `s57_layer/src/s57_layer.cpp` | `:628-638`: the index arithmetic above, with its comment |
| `s57_layer/test/test_tile_edges.cpp` | New: the tests and checked accessors above |
| `s57_layer/CMakeLists.txt` | `ament_add_gtest(test_tile_edges ...)`, includes and links as `test_tide_offset`, with a comment saying it defines the checked accessors |
| `s57_layer/test/test_rolling_window.cpp`, `test_initialize.cpp` | Comment wording only (step 3) |

## Prior Art

The issue review's scratch harness (accessor interposition, the three options over 432 scenarios), taken. Nav2's `worldToMap` (fails on the edge) and cell-centre sampling, rejected in the review (fallback or behaviour change everywhere). `updateCosts` and `bathymetry_layer` already use half-open ranges; (b) follows them.

## Serves

The project keeps no goals or design document; in practice: vessels' costmaps show charted hazards where they are charted, and the costmap node does not corrupt its memory.

## Design Picture

No change to the design picture; the project has no design document, and `s57_layer/README.md` does not describe tile compositing.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Test what breaks | Every cell against an independent expectation, plus access checking that fails even when the value is right; the interposition proves itself (call count). |
| Only what's needed | Four index lines and their comment in hazard-marking code; the straddle rule untouched; no restructuring for the test. |
| Verify before claiming | Equivalence measured (20000 + 3000 + 216 cases), every difference traced; Jazzy linkage checked on the released binary; Jazzy run is CI's. |
| A change includes its consequences | The two test comments are reworded in the same commit. |
| Small steps | One code commit, cherry-picked to `jazzy` (`s57_layer` is identical on both branches). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0013 progress entries | Yes | `## Plan Authored` entry citing this commit |
| 0002 worktree isolation | Yes | Work in the issue's worktree |
| 0019 repo_pin | Yes | Base `rolling`; backport by `git cherry-pick -x` to `jazzy` |
| 0017 design document | No | No design change |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Tile-cell range in `generateTile` | Test comments citing #63 | Yes |
| | `s57_layer/README.md` | No: says nothing on compositing |
| | #59 (next, `SubmapGeometry` at `:609-617`) | Separate hunk. (b) makes #59 easier: a chart cell that touches or misses the tile now paints nothing, so #59 can iterate any superset of the overlapping chart cells (a clipped or padded range) without the edge cases above |

## Planned commits

1. (this) the plan, `.agent/` only.
2. `s57_layer: half-open tile-cell ranges in generateTile (#63)`: the fix, the new test and CMake entry, the two comment rewordings. No `.agent/` file. Backport: `git cherry-pick -x` of this commit onto `jazzy`.

## Open Questions

- None blocking. For the owner (from the review, unchanged by this plan): whether to open an issue on the straddle rule (a tile cell cut by a chart cell edge takes the east/north chart cell's value however small its share).

## Estimated Scope

Single PR to `rolling` (one code commit), then a one-commit backport PR to `jazzy`.
