# Plan: s57_layer drops a chart's data for a tile whose centre is outside the chart (gap at chart boundaries)

## Issue

https://github.com/rolker/s57_tools/issues/59

## Context

`S57Layer::generateTile` (`s57_layer/src/s57_layer.cpp:609-617` on `rolling` at `e95d7bd`, after #57 and #63) asks grid_map for each chart's cells under the tile with `SubmapGeometry(grid, tile_center, tile_size)`. The issue review established two ways that request fails and drops the chart from the whole tile: (1) the tile's centre lies outside the chart's grid on either axis (`GridMapMath.cpp:340-342`; on the grid's west/south edge counts as outside), which covers every seam, every chart edge whose part of the tile is the smaller half, and a small chart inside a tile but off its centre; (2) the tile's south-west corner lies on or beyond the chart's south-west edge, `boundPositionToRange` pulls it to the edge plus ε = 10·DBL_EPS·max(1, |corner|), and ε is lost in `positionShifted + mapPosition − L/2`; that happens in a band around map x = 0 and y = 0 that grows with the chart's length. The host's checkpoint chose the review's option (a′), as the only path, on condition that the output is cell-for-cell identical to today's wherever today's request succeeds. **The condition holds (below); (a′) replaces today's request; no fallback.**

## Approach

1. **The fix: option (a′)**, in the per-chart loop of `generateTile`, replacing the `SubmapGeometry` call's arguments; the paint loop, the pre-filter, `complete`, `chart_count` and `needs_update` are unchanged (draft: `harness/aprime.patch`, 27 added lines and 1 changed):
   ```cpp
   // per axis a, from the grid's own geometry (setGeometry rounds the grid to whole cells,
   // so not chart_bounds_):
   grid_lo = getPosition()[a] - getLength()[a]/2;  grid_hi = getPosition()[a] + getLength()[a]/2;
   if(grid_lo >= tile_hi[a] || grid_hi <= tile_lo[a]) -> skip this chart (no positive-area overlap)
   lo = std::clamp(tile_lo[a], grid_lo + r/2, grid_hi - r/2);   // r = grid resolution
   hi = std::clamp(tile_hi[a], grid_lo + r/2, grid_hi - r/2);
   if(lo != tile_lo[a] || hi != tile_hi[a]) { center[a] = lo + (hi - lo)/2; length[a] = hi - lo; }
   // else center[a], length[a] stay today's tile_center[a], tile_size[a]
   SubmapGeometry overlap(grid, center, length, ok);
   ```
   with a comment giving the reason (the two failures above) and why the clip is to cell centres. `tile_lo/hi` are `world_min/max_x/y` as computed today. `std::clamp` needs `<algorithm>` (in `s57_layer.h`); `grid_lo + r/2 <= grid_hi - r/2` holds for any grid of at least one cell.
2. **Tests** in `s57_layer/test/test_tile_edges.cpp` (below; draft: `harness/test_tile_edges.patch`).
3. **Reword two test comments that the fix makes false**, the set-ups unchanged: `test_initialize.cpp:409-413` ("SubmapGeometry, which fails when the tile's centre is outside the grid or a clipped corner falls on a grid edge at 0") and `test_rolling_window.cpp:519-523` ("so #59 ... cannot affect these cases"): the charts reach past the tiles so the test does not depend on chart-edge handling, which `test_tile_edges` covers.
4. **Verify on Rolling by hand**: `build.sh --packages-up-to s57_layer`, `test.sh --packages-select s57_layer`, `ldd` the test binary to see it loads the freshly built `libs57_layer.so` (not the installed one); show the new tests failing with the source hunk reverted. Jazzy: the pull request's Jazzy CI job.

### (a′) exactly

- **The rectangle**: the grid's cell centres span `[grid_lo + r/2, grid_hi − r/2]` on each axis, `grid_lo/hi` from `getPosition() ∓ getLength()/2`. Each tile edge is clamped into it separately, so the clipped rectangle is never inverted; it has zero width on an axis when the tile overlaps only the grid's first or last half cell there (`SubmapGeometry` then returns that one row or column: 261 632 zero-width requests in the comparison, none failed).
- **Cases**: tile wholly inside the grid's cell-centre rectangle: nothing clamps and the request **is today's request**, the same `tile_center` and `tile_size` doubles, by construction (the clamp's result is only compared, never used). Grid wholly inside the tile: the cell-centre rectangle. Overlap by less than one chart cell: a zero-width or sub-cell request inside the edge cell. Touching only, or disjoint: skipped before any request (`grid_lo >= tile_hi || grid_hi <= tile_lo`); today's request fails there too (centre outside), so nothing changes.
- **Why it cannot fail**: no corner is outside or on the grid's edge, so `boundPositionToRange` changes nothing and ε plays no part; the centre lies between the two corners, at least half a chart cell inside the grid, so the final centre check passes.
- **Superset**: grid_map returns the whole cells from the one holding the south-west corner to the one holding the north-east corner, a cell holding the positions in (its west edge, its east edge]. West/south: when `tile_lo >= grid_lo + r/2` the corner is the tile's edge, held by the west-most chart cell with positive overlap, or by the cell that only touches it when the edge lies exactly on a cell edge (extra, painted nothing since #63); otherwise the corner is the first cell's centre, which is the west-most overlapping cell. East/north: the cell holding `tile_hi` is the east-most cell with positive overlap (half-open on that side), and the last cell's centre is in the last cell. So the request is the overlapping cells plus at most one touching row or column, never a strict subset, up to rounding of a tile edge lying within rounding of a chart cell edge (as #63 states for the paint loop). Measured: 0 cells wrong after the fix, below.

### The condition, measured (`harness/`, `measure.sh`)

Both copies of the layer compiled into one executable against the installed grid_map 2.5.0 and Nav2 `Costmap2D` (`ldd`: no `libs57_layer`), `generateTile` called on each for the same tile, every cell compared between them and with an expectation computed from the chart specifications alone (positive-area overlap; finest chart first, then label; east-most then north-most chart cell with data), in integer quarter cells for exact geometry and in `long double` otherwise, where a window cell with a chart edge within 1e-6 m of its edge is counted as ambiguous and not judged. Tile accesses through bounds-checking `getCost`/`setCost` as in `test_tile_edges`.

Set: deterministic families (one chart on each side and at the NE/SW corners with edges at every quarter cell of a 4-cell tile and at 1, 36.25, 49.25, 49.75, 50, 50.25, 50.75, 64.25, 99 m of a 100-cell tile; seams at the same positions, x and y; small charts off and over the centre; fine water over coarse land; the map-axis band and charts whose own edge is near an axis, 0.75 to 5 km charts, tile corners within ±400 m of 0; costmap 1 and 0.5 m, origin 1000.25 cells and 0; chart cells 0.5, 1, 2, 3.25 costmap cells, and 1, 3.25, 7.5 in the band) and 10 000 random cases (4 seeds): 1-3 charts (seam east or north, a coarser chart under, or random overlap); chart lengths 0.4 tile to 100 km; chart cells 0.3 to 30 costmap cells (coarsened to at most 1500 cells a side); costmap 1, 0.5 and 0.1 m; tiles 4 and 100; layer origin 0, ±300 m, ±1e5 m; charts straddling both axes, with their south-west edge within 5% of their length of an axis, near the tiles' origin, or near ±1e5 m; 20% no-data cells; 40% binary-exact, 40% arbitrary, 20% exact then moved 1-3 ulps (edges within rounding of tile-cell edges).

| | Tiles | Cells | Today vs (a′) differ | Wrong today | Wrong after | Out of range |
|---|---|---|---|---|---|---|
| Today's request succeeds for every chart | 346 680 | 1 506 631 296 | **0** | 0 | 0 | 0 |
| Today's request fails for some chart | 212 916 | 954 712 128 | 198 288 563 | 190 777 942 | **0** | 0 |

(14 608 cases, 704 498 chart requests, 220 086 of them failing today; 81.4 M cells ambiguous, nearly all in the ulp-moved cases, excluded from "wrong" only.)

Of the chart requests that succeed today (in either row), (a′) clamps 233 839 (the tile reaches past the grid's cell centres while today's request still succeeds). Its submap equals today's in all but **358**, and in every one of those 358 today's submap runs one row or column past the grid's south or west end: grid_map's corner lookup rounds an index to `size` after its position check passed. Today's code then reads `grid.at()` with that index (1653 indices in all) — **an out-of-bounds read of the chart's Eigen buffer**, through `GridMap::at` in the Release library, so no assertion — and paints nothing from it because `getPosition` then fails. So the outputs are identical, and the only submap differences are today's code being wrong (an out-of-bounds read, painting-neutral; seen only with non-binary geometry, as real charts are). (a′) removes that read.

### What changes for a vessel (cells where today's request failed)

- (i) **Newly charted**: 180 412 193 cells unknown in the tile become charted (free on a local costmap without `track_unknown_space`, unknown on a global one, before).
- (ii) **Where another chart had filled the strip, the finer (or first-ordered) chart's value now wins, so costs go down as well as up**: lethal → free 9 384 463 cells, free → lethal 8 491 907 (charted → uncharted: 0). The fine-water-over-coarse-land family alone: 221 440 lethal → free. Seam cells cut by both charts of one scale take the first chart by label rather than whichever survived (1 971 and 2 022 of the seam family). "Nothing that was marked becomes unmarked" does **not** describe this fix; the pull request says costs can go down where the finer chart shows water. (The harness uses land and 10 m water only; on a vessel depth costs between free and lethal change the same way.)
- (iii) **Map-axis band**, 1:12000 cell (3.75 m cells, 6.1 × 8.3 km), tile corner swept in 0.37 m steps: today's request fails for every tile corner within **±307 m of x = 0** (6.1 km extent) and **−512 to +614 m of y = 0** (8.3 km), at 1 and 0.5 m, wherever the chart's west (south) edge lies inside that tile, whatever its inset (0.25 to 49 cells). There today drops the chart's **whole** part of each such tile along that edge's whole length: a strip up to one tile wide (100 m at 1 m, 50 m at 0.5 m). The band is around the map frame's axes, which on BEN pass through the first position fix, so it matters for chart edges within a few hundred metres of the start position's x or y line.

### Cost per tile (two adjoining 1:12000 cells, 100-cell tiles, Core Ultra 7 265, median of 2000 calls)

| Tile | 1 m: chart cells / µs, today → (a′) | 0.5 m: today → (a′) |
|---|---|---|
| Interior | 756 / 62 → 756 / 63 | 210 / 33 → 210 / 34 |
| Edge, centre inside the chart | 504 / 41 → 504 / 41 | 150 / 25 → 150 / 25 |
| Edge, centre outside | 0 / 0.4 → 280 / 24 | 0 / 0.4 → 75 / 11 |
| Seam | 504 / 42 → 784 / 65 | 150 / 23 → 210 / 33 |
| Chart corner | 0 / 0.4 → 234 / 19 | 0 / 0.4 → 65 / 10 |

Cost rises only on tiles where today drops a chart, to about an interior tile's (at most one extra row or column per chart); tens of microseconds per tile against `update_timeout` 0.5 s.

### Pre-filter (`:577-582`, `chart_bounds_`): listed, not in scope

`S57Dataset::getGrid` (`marine_charts/src/s57_dataset.cpp:161-162`) sets the grid's length to `round(L/r)·r` centred on the dataset's bounds, so each grid edge lies within r/4 of the bound (not r/2, as the review wrote), assuming `llToMap` and the layer's `llToWorld` agree (not checked). The pre-filter can therefore skip a chart only for a tile that overlaps the grid solely in that ≤ r/4 margin **outside the chart's own bounds** (≤ 0.94 m for a 1:12000 cell): no data inside the chart's stated coverage is lost, so it is not this defect. The reverse (grid inside the bounds) lets through a chart the grid does not reach; (a′)'s overlap check skips it before any request. Neither changes with this fix.

### #57 tile reporting and completeness

No interaction: the change is inside the per-chart loop; `complete = all_charts_available`, `chart_count = grids.size()` and `needs_update` are set as before, and the new skip is a `continue` in that loop. No cached state needs invalidating; a vessel gets the change by restarting.

### Tests (`test_tile_edges.cpp`)

Fixture extension, existing tests unchanged: a `Scene` (resolution, window origin in cells, window size, tile size, a list of charts) and a `ChartSpec` per chart (label, cell size and west/south edge in quarter window cells, cells per side, fill: hashed with a seed, all land, all water); `makeChart`, `expectedCost` (charts sorted by cell size then label, as the layer's map orders them), `paint` and `cellsAsCharted` overloads for a `Scene`, and `check(scene)`. `accessesInRange` takes the window and tile size from the costmap and the log instead of `kCells`/`kTileSize`. Every cell is asserted against the integer expectation, every tile access bounds-checked. Window 12 × 12, 4-cell tiles, origin (−8, 4) cells unless stated (tile column 1 spans quarters 16-32, centre 24; tile column 2 starts at map x = 0). Failures on today's source (each passes with the fix; 43/43 pass with it):

| Test | Scene | Today |
|---|---|---|
| `CentreOutsideChartWest1m`, `…HalfMetre` | chart ends 5 quarters into tile column 1 | 22 of 144 cells wrong |
| `CentreOutsideChartEast1m` / `South1m` / `North1m` | chart starts 11 / ends 5 / starts 11 quarters in | 23 / 16 / 21 |
| `CentreOutsideChartCorner1m` | chart's NE corner at (21, 21) quarters | 20 |
| `SeamWestOfCentre1m` / `SeamOnCentre1m` / `SeamEastOfCentre1m` | two charts, seam at 21 / 24 / 27 quarters | 16 / 21 / 16 |
| `SeamOnCentreNorthHalfMetre` | seam on tile row 1's centre | 18 |
| `SmallChartOffCentre1m` | 1.5 × 1.5 cell chart inside tile (1, 1), off its centre | 4 (whole chart dropped) |
| `FineWaterOverCoarseLand1m` | fine water to 5 quarters in, coarse land under all | 24: lethal where water is charted |
| `ChartEdgeInTileAtMapXZero1m`, `…HalfMetre`, `…MapYZero1m` | 70 m chart whose west (south) edge is 0.75 m east (north) of map 0, covering the tile's centre (band, 4-cell tiles) | 46, 46, 45: the chart's whole part dropped |
| `ChartEdgeInTileAtMapOrigin100CellTiles1m` | 200 × 200 window from (−100, −100), 100-cell tiles, chart from (36.25, 36.25) m | 4078 of 40000 |
| `ChartEdgeJustWestOfMapXZero1m` | chart from x = −0.75 m, tile −4..0 m | 11 (also fails under the issue's clip-to-bounds proposal, checked: so do the three band tests above) |

Guards (pass today, labelled so): `SmallChartOverCentreGuard1m`, `TilesInsideChartGuard1m` (every tile inside the chart's cell-centre rectangle: the request is today's, unchanged), `ChartTouchingTileEdgeGuard1m` (a chart edge only touching tile column 1 paints nothing there).

## Files to Change

| File | Change |
|------|--------|
| `s57_layer/src/s57_layer.cpp` | `:609-616`: the clipped request above, with its comment |
| `s57_layer/test/test_tile_edges.cpp` | `Scene` fixture extension and the tests above; header comment mentions #59 |
| `s57_layer/test/test_initialize.cpp`, `test_rolling_window.cpp` | Comment wording only (step 3) |

## Prior Art

The issue review's harness and its option table: (a′) taken; (a) rejected (the band tests fail under it, rechecked here); (b) rejected (grid_map's index order and buffer start are internal conventions; 17 cells differed in one random case); fallback form not needed since the condition holds. #57's and #63's harness pattern (two layer copies in one executable, renamed namespaces) and #63's checked accessors, reused.

## Serves

The project keeps no goals or design document; in practice: charted hazards present in the costmap at every chart edge and seam.

## Design Picture

No change to the design picture; the project has no design document, and `s57_layer/README.md` does not describe tile compositing.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Verify before claiming | The condition measured over 1.5 G cells and 14 608 cases; the 358 submap differences traced to today's out-of-bounds read; costs measured; Jazzy run is CI's. |
| Test what breaks | 17 new tests fail today, each cell asserted against an integer expectation; 3 guards labelled. |
| Only what's needed | One request in one loop; paint loop, pre-filter and reporting untouched. |
| A change includes its consequences | Two test comments made false are reworded in the same commit; the PR states costs can go down. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0013 progress entries | Yes | `## Plan Authored` entry citing this commit |
| 0002 worktree isolation | Yes | Work in the issue's worktree |
| 0019 repo_pin | Yes | Base `rolling`; backport by `git cherry-pick -x` to `jazzy` |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| The submap request | Test comments citing the old failure | Yes |
| | `s57_layer/README.md` | No: says nothing on compositing |
| | Pull request text: before/after counts, costs down and up, the band | Yes (publish phase) |

## Planned commits

1. (this) the plan and `harness/`, `.agent/` only.
2. `s57_layer: request each chart's cells clipped to its cell centres (#59)`: the fix, the tests, the two comment rewordings. No `.agent/` file. Backport: `git cherry-pick -x` of this commit onto `jazzy`.

## Open Questions

- None blocking. For the owner: today's out-of-bounds read of a chart's buffer (above) is removed by this fix and needs no separate change on `rolling`/`jazzy`; whether it deserves a note in the pull request only (planned) or its own record.

## Estimated Scope

Single PR to `rolling` (one code commit), then a one-commit backport PR to `jazzy`.
