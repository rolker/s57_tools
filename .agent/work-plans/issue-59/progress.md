---
issue: 59
---

# Issue #59 — s57_layer drops a chart's data for a tile whose centre is outside the chart (gap at chart boundaries)

## Issue Review
**Status**: complete
**When**: 2026-11-24 21:19 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**Issue**: #59

### Scope Assessment

**Well-scoped?** Yes, with corrections: one defect, one function (`S57Layer::generateTile`), one PR with tests, then a hand backport. But the issue's account of the second cause is wrong in a way that matters to the fix: the fix it proposes (clip the tile to the chart, request the clipped rectangle) **still fails** through that second cause (see "The epsilon cause" below). The plan must use a request that never puts a corner on or outside the chart's edge, or skip grid_map's position lookup.
**Right repo?** Yes: `s57_layer/src/s57_layer.cpp`, rolling first, then a `git cherry-pick -x` to `jazzy`.
**Serves**: none written down (the project keeps no goals or design document). In practice: charted hazards present in the costmap at every chart boundary.
**Design picture?** No change: a change inside one function. The project has no design document.
**Dependencies**: #57 and #63 are merged on `rolling` and `jazzy` (checked against `rolling` at e95d7bd). #63 is what lets the fix hand the loop any superset of the overlapping chart cells. Nothing else is open in this area.

#### Reproduced on this branch (Rolling, grid_map 2.5.0; outside the tree)

A scratch harness compiled `s57_layer.cpp` in, injected charts as the committed tests do, called `generateTile` directly, and compared every tile cell with an expectation worked out in integer quarter cells from the charts alone (positive-area overlap; finest chart first; within a chart, the east-most then north-most cell that has data). A copy of the layer's paint loop matched `generateTile` cell for cell in every case, so the variants below run in the same loop. Setup: costmap at 1 m and 0.5 m, tiles of 4 and 100 cells, origin 1000.25 cells and origin 0, chart cells of 0.5, 1, 2 and 3.25 tile cells, edges at 1 to 15 quarter cells (T 4) and at 1, 36.25, 49.25, 49.75, 50, 50.25, 50.75, 64.25 and 99 cells (T 100). 2818 cases in all; today's code is wrong in 1903 of them. Results were the same at 1 m and 0.5 m. Per family, at T 100 and origin 1000.25:

| Case | Wrong cells (max of 10000) | What `SubmapGeometry` returned |
|---|---|---|
| one chart, west/south part of the tile, centre outside | up to 4999 (e.g. edge at 36.25 m: 3545, all of the chart's cells, 1782 of them land) | FAIL |
| the same with the centre inside | 0 | ok: the chart's cells from the tile's corner to its edge |
| one chart, east/north part, centre outside (edge at 64.25 m: 3422) | up to 5000 | FAIL |
| corner (NE or SW quarter) | up to 2500 | FAIL |
| seam between two charts, at any position inside the tile | 3354 to 4985: always exactly one chart is dropped, the one not holding the centre. A seam exactly at the centre drops the east/north chart (grid_map's range is (min, max]). | one FAIL, one ok |
| chart smaller than a tile, wholly inside it, not over the centre | the whole chart (119 to 172 cells) | FAIL |
| the same over the centre | 0 | ok |
| fine water chart on the west 36.25 m, coarse all-land chart under the whole tile | 3700: **lethal where the fine chart says water** | coarse ok, fine FAIL |

T 4 behaves the same at that scale (up to 8 of 16 cells; up to 16 of 16 at origin 0).

**Axis cases** (origin 0; tiles (0,0), (-1,0), (0,-1), (-1,-1); seam at 1, 36.25, 49.75, 64.25 and 99 m):
- Tile [0,100] with a seam at 36.25 m: both charts are dropped (9603 of 9603 charted cells left empty). This matches the issue.
- Tile [0,100] with the seam at 64.25 m, and the tiles at [-100,0]: only the centre cause applies.
- A single chart whose west edge is 36.25 m into tile [0,100] covers the centre and is still dropped: 6129 cells, the chart's whole part of the tile. With the tile's corner moved to x = 10 m it is still dropped; at 20 m it is used (chart about 364 m long).

#### grid_map, exactly (2.5.0 here; `git diff 2.2.2 2.5.0` and `origin/jazzy..2.5.0` are empty for `GridMapMath.cpp`, `SubmapGeometry.cpp`, `SubmapIterator.cpp`, `GridMap.cpp` and their headers, so Jazzy behaves the same)

`SubmapGeometry` (`SubmapGeometry.cpp:15-25`) calls `getSubmapInformation` (`GridMapMath.cpp:283-343`):
1. Each requested corner goes through `boundPositionToRange` (`:251-276`). A coordinate at or below the chart's min edge is set to min + ε, and one at or above the max edge to max − ε, where ε = 10·DBL_EPS·max(1, |requested corner coordinate|) (`:262-263`).
2. The result is computed as `positionShifted + mapPosition − L/2`. `checkIfPositionWithinMap` (`:162-178`) then accepts a position only in **(min, max]** on each axis. The max corner can never fail: if ε is lost, the position equals max, which the closed side accepts. The **south-west corner fails when ε is lost**. That happens when ε is below about half an ulp of the magnitudes in that arithmetic, which are the chart's length and centre, so when |corner| ≲ L/16 to L/20, L being the chart's extent on that axis.
3. The submap is the whole chart cells from the cell holding one bounded corner to the cell holding the other.
4. The request fails (`:340-342`) unless the **requested centre** lies in (submap min, submap max]. Since the submap reaches at least the chart's edge, this fails exactly when the tile's centre is not in (chart min, chart max] on either axis.

So it fails in these cases and no others:
- (a) The tile's centre is outside the chart's grid on either axis, with the centre on the chart's west or south edge counting as outside. This covers a request wholly outside the chart, a chart touching the tile only at an edge (correct: nothing to paint), a chart wholly inside the tile but off its centre, and every seam.
- (b) The epsilon cause: the tile's west (south) edge lies on or west (south) of the chart's west (south) edge, and the tile's corner coordinate is small relative to the chart's size.

A requested length larger than the chart is not a failure in itself. Wherever the request succeeded, the harness found the region never smaller than the true overlap: every such case was correct cell for cell.

#### The epsilon cause is not about coordinate 0

The issue's "tile corner at exactly 0" is the extreme of a band that grows with the chart's size. A sweep of tile corners from −30 km to 30 km, for a chart whose west edge lies 0.25 to 49 m inside a 100 m tile and which covers the tile's centre, found the request failing for every corner within:

| Chart length on that axis | Corners that fail (either side of the map's x = 0 or y = 0) |
|---|---|
| 500 m | about ±13 to 19 m |
| 5 km | ±307 m |
| 20 km | ±1229 m |
| 100 km | ±4915 m |

It affects only charts whose west or south grid edge crosses a tile in that band. It drops the chart's **whole** part of the tile, up to a full tile and not half. It covers only the south-west corner. The harness's cases with a chart's east or north edge inside a tile at origin 0 showed no failures beyond the centre cause, as the closed max side predicts. A 1:12000 NOAA cell is about 6.1 × 8.3 km, so its band is a few hundred metres either side of the map's axes, and a coarse chart's band is kilometres wide. The axes pass through the map frame's origin; I have not checked where vessels put it.

#### The issue's statements, checked

- Line numbers have moved: `generateTile` is at `s57_layer.cpp:542`, and the request at `:609-616`. The centre check is at `GridMapMath.cpp:340-342`, inside `getSubmapInformation` at `:283-343`. The epsilon is at `:262-268`.
- "up to just under half a tile on the side that does not hold the centre": **confirmed** for the centre cause (max 4999 of 10000 cells). It **does not hold** for the epsilon cause (whole tile), nor for a small chart inside a tile (all of it).
- "keeps the costmap's reset value…; the tile counts as complete, so `allow_uncharted: false` does not mark it unknown": **confirmed** on the current code. `generateTile` sets `complete = all_charts_available` (`:604`) whatever the requests returned. `updateCosts` copies only cells that are not NO_INFORMATION (`:710-712`) and takes the `allow_uncharted` branch only for a tile with no costmap (`:716`). #57 changed which tiles are reported (`needs_update`, strips), not completeness, so nothing differs after it.
- "every chart cell boundary and every edge of coverage": **narrow this** to every edge of a chart's grid rectangle (its bounding box, set in `S57Dataset::getGrid`) and every seam between cells of one scale. Inside a chart's rectangle, outside its coverage, the cells hold NaN; the layer passes over them correctly, and a coarser chart fills them. That is not this defect.
- "unless an overlapping chart of another scale covers the tile's centre": **confirmed and needs adding to**. `GridPublisher::listDatasets` (`grid_publisher.cpp:228`) serves every intersecting chart with scale denominator > 2·resolution/0.0003125 (6400 at 1 m, 3200 at 0.5 m). The layer requests that (`s57_layer.cpp:320`), so approach, coastal and general cells are served with harbour cells wherever the catalogue holds them. `generateTile` paints them finest first, the first writer winning. In the dropped strip **the coarser chart's value shows instead**, which is not always "free": it can be lethal over navigable water. The harness case above gave 3700 cells lethal where the fine chart says water: the Lewes failure that `get_cost_from_grid`'s comment records. It can equally be water over a fine-scale hazard. The strip is left unmarked only where no other chart covers it.
- NOAA cells (cheap look): `US5PSMCC` and `US5PSMCD` in `asv_sim/enc` are both 1:12000. Their M_COVR envelopes abut exactly at lon −70.725 and do not overlap, so every tile straddling that seam drops one of them. That directory holds no coarser cell, so in the simulator nothing fills the gap. **Not established**: which coarser cells a vessel's catalogue holds, and how often a vessel's window holds a seam tile. On a static costmap that holds a seam it is on screen continuously.

#### Fix options, measured on the same harness (2818 exact cases, plus 2491 random cases at 0.05, 0.1 and 0.3 m with 1 to 3 charts, where edges cannot be placed exactly)

| Option | Wrong (exact cases) | Changes output where today's request succeeds | Cost per tile | grid_map dependence |
|---|---|---|---|---|
| (a) as proposed: clip the tile to the grid, request the clipped rectangle | 83 cases, 453593 cells: **the request fails** when the chart's own west/south edge is near an axis (sweep: chart edges within ±307 m for a 5 km chart, ±1229 m for 20 km) | no in the exact cases; in 1 random case yes, where it failed | as today | as today |
| (a′) clip the tile to the grid's **cell centres** (`clamp(edge, gmin + r/2, gmax − r/2)`), request that | 0 | 0 differences (exact and random) | as today | public API only; no corner is ever bounded, so ε plays no part |
| (b) index range from the grid's own geometry (`floor`/`ceil` on `(edge − gmin)/r`, one cell of margin), `SubmapIterator(grid, start, size)` | 0 | 0 in the exact cases. In 1 random case, 17 cells (14 repainted, 3 newly painted): an edge cell grid_map left out, which #63's arithmetic counts as overlapping by rounding, and which is iterated first | +15% cells (1.16 vs 1.00 ms for 20 tiles) | grid_map's index order (index 0 at the max corner) and its buffer start index: unchanged 2.2.2 to 2.5.0, but internal conventions |
| (c) iterate the whole grid when the request fails | correct (#63) | none by construction | 0.14 s per failing chart per tile for a 1:12000 cell at 3.75 m (3.6 M cells, `get_cost_from_grid` only, desktop Core Ultra 7). A seam crossing a 5 km window touches ~50 tiles, about 7 s against `update_timeout` 0.5 s | none |
| (d) move the requested centre into the grid and grow the length | 518 cases wrong: the grown request's south-west corner is still bounded, so the epsilon cause remains | no | up to 4× cells on edge tiles | as today |

Two forms meet "must not change output where today's request succeeds" by construction: (a′) on its own is identical in every case run, but not by construction. Either form costs one extra `SubmapGeometry` only on failure:
- today's request first, with (a′) used only when it fails;
- the same with (b) as the fallback.

**Recommendation**: (a′), either alone or as the fallback. It is a few lines in the loop and uses only grid_map's public API. It cannot meet the epsilon, because no corner is ever bounded, and its centre is always inside the grid. The clip must use the grid's own geometry (`getPosition() ± getLength()/2`), not `chart_bounds_`. `setGeometry` rounds the grid to whole cells, so the grid differs from the dataset's bounds by up to half a chart cell, which is 12.5 m on a 1:80000 chart.

#### What changes on a vessel once fixed (to be shown in the PR, not assumed)
- Where the dropped chart had no other chart under it, cells that are free or unknown today become charted, and nothing that is marked today becomes unmarked.
- Where a coarser chart filled the strip, the cells take the finer chart's value. That **can lower a cost** (lethal to free, as in the coarse-land case above), which is the layer's intended finest-chart-first behaviour, but the PR must state it. "Nothing marked becomes unmarked" is false there.
- No cached state needs invalidating. `chart_count` counts charts passed to the loop, not charts that painted, and a deployed vessel gets the change by restarting. Chart-set changes and tide steps regenerate tiles as before.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Verify before claiming | Action needed | The issue's second cause ("a tile corner at exactly 0") is wrong as stated: it is the tile's south-west corner bounded to a chart's south-west edge, with that corner coordinate within about L/16 to L/20 of the map's x or y axis (L the chart's length; hundreds of metres for a harbour cell). It drops the chart's whole part of the tile. The proposed fix (a) fails the same way when the chart's own edge is in that band. The plan should take this correction, not the issue text. |
| Test what breaks | Action needed | The tests should pin both causes and the coarse-fill case. Each needs a case that fails on today's source, with every cell asserted against an independently computed expectation. Use the `test_tile_edges` pattern; it may be best as new `TEST_F`s in that file, which shares the fixture and the checked accessors. Cases: a chart on each side and a corner with the centre outside; a seam at several positions, including exactly at the centre; a small chart inside the tile off the centre; a tile at origin 0 whose chart's west/south edge is inside it and which covers the centre (fails today at T 4 and T 100); the clipped rectangle with the chart's own edge near 0 (fails under (a)); and a fine chart over a coarse one. Guards that pass today: the centre inside, a chart touching the tile only at an edge. Also a cell-for-cell comparison with today's output on cases where today's request succeeds, as for #63. |
| A change includes its consequences | Watch | The README's "How It Works" says nothing about boundaries, so no document becomes false. The PR should state the vessel-level change, including the lowered costs where a coarse chart filled the gap. |
| Small steps; step back when they stop converging | OK | One loop change. |
| Leave in a project only what it chose to carry | OK | Records in the PR, as for #57 and #63; no `.agent/` file in the code commit. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0013 | Yes | This review is the `## Issue Review` entry. |
| 0002 | Yes | Work in the issue's worktree (in use). |
| Others | No | Project code only. |

### Consequences

- The PR description should give the measured before/after, and state that cells where a coarser chart filled the gap take the finer chart's value (which can lower their cost).
- The backport to `jazzy` (`git cherry-pick -x`) needs no grid_map change: the functions involved are identical in 2.2.2 and 2.5.0.

### Recommendations

- Correct the record of the second cause in the plan: it is a band around the map axes that scales with the chart's length (measured ±307 m for a 5 km chart), it affects only the south-west corner, and it drops up to a whole tile, not half.
- Do not implement (a) as written. Clip to the grid's cell centres (a′) or compute the index range (b), preferably as a fallback after today's request so that output is unchanged by construction wherever that request succeeds. Clip to the grid's own geometry, not `chart_bounds_`.
- Narrow "every edge of coverage" to the edges of each chart's grid rectangle and the seams between cells of one scale.
- Add to "what the costmap shows" that where a coarser chart is served, the strip shows that chart's value (possibly lethal over water, or water over a fine-scale hazard), and that the fix can therefore lower costs there.
- Out of scope, worth a note in the plan: the tile pre-filter (`:577-582`) uses `chart_bounds_` (the dataset's bounds through `llToWorld`), not the grid's extent. They differ by up to half a chart cell, so a tile overlapping only that margin is skipped. Leave it as it is unless the plan finds it matters.
- For the owner: whether "unchanged where today's request succeeds" must hold by construction (the fallback form), or whether matching cell for cell in every case measured is enough (a′ alone, slightly simpler).

### Actions
- [ ] The issue's second cause ("a tile corner at exactly 0") is wrong as stated: it is the tile's south-west corner bounded to a chart's south-west edge, with that corner coordinate within about L/16 to L/20 of the map's x or y axis (L the chart's length; hundreds of metres for a harbour cell). It drops the chart's whole part of the tile. The proposed fix (a) fails the same way when the chart's own edge is in that band. The plan should take this correction, not the issue text.
- [ ] The tests should pin both causes and the coarse-fill case. Each needs a case that fails on today's source, with every cell asserted against an independently computed expectation. Use the `test_tile_edges` pattern; it may be best as new `TEST_F`s in that file, which shares the fixture and the checked accessors. Cases: a chart on each side and a corner with the centre outside; a seam at several positions, including exactly at the centre; a small chart inside the tile off the centre; a tile at origin 0 whose chart's west/south edge is inside it and which covers the centre (fails today at T 4 and T 100); the clipped rectangle with the chart's own edge near 0 (fails under (a)); and a fine chart over a coarse one. Guards that pass today: the centre inside, a chart touching the tile only at an edge. Also a cell-for-cell comparison with today's output on cases where today's request succeeds, as for #63.
- [ ] Correct the record of the second cause in the plan: it is a band around the map axes that scales with the chart's length (measured ±307 m for a 5 km chart), it affects only the south-west corner, and it drops up to a whole tile, not half.
- [ ] Do not implement (a) as written. Clip to the grid's cell centres (a′) or compute the index range (b), preferably as a fallback after today's request so that output is unchanged by construction wherever that request succeeds. Clip to the grid's own geometry, not `chart_bounds_`.
- [ ] Narrow "every edge of coverage" to the edges of each chart's grid rectangle and the seams between cells of one scale.
- [ ] Add to "what the costmap shows" that where a coarser chart is served, the strip shows that chart's value (possibly lethal over water, or water over a fine-scale hazard), and that the fix can therefore lower costs there.
- [ ] Out of scope, worth a note in the plan: the tile pre-filter (`:577-582`) uses `chart_bounds_` (the dataset's bounds through `llToWorld`), not the grid's extent. They differ by up to half a chart cell, so a tile overlapping only that margin is skipped. Leave it as it is unless the plan finds it matters.
- [ ] For the owner: whether "unchanged where today's request succeeds" must hold by construction (the fallback form), or whether matching cell for cell in every case measured is enough (a′ alone, slightly simpler).

## Checkpoint
**Status**: complete
**When**: 2026-11-24 21:20 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the issue review confirmed the defect and the goal; its corrections to the issue's description and proposed fix are in its comment on the issue; the choice of fix is a mechanism choice inside the issue's goal: a chart contributes wherever it overlaps a tile)
**After**: issue-actions
**Decision**: proceed

Read: the Issue Review, status complete. Scope stands and is stated more exactly: `generateTile` takes a chart's data for a tile wherever the chart's grid overlaps the tile, whatever grid_map's submap request does. The review corrected the issue on three points, which the plan and the pull request use: (1) the request fails in two cases: the tile's centre outside the chart's grid; and an epsilon lost when the tile's south-west corner is pulled in to the chart's south-west edge, which is **not about coordinate 0** but a band around the map's x = 0 and y = 0 lines whose width grows with the chart's length (measured: about ±307 m for a 5 km chart), dropping the chart's whole part of the tile; (2) a small chart wholly inside a tile and off its centre is dropped entirely; (3) where a coarser chart is loaded, the dropped strip shows the coarser chart's value, which can be lethal over water the finer chart shows navigable, or water over a hazard only the finer chart shows. **The issue's proposed fix (clip the tile to the chart, request the clipped rectangle) still fails** when the chart's own south-west edge is near an axis. For the planner: **the fix is the review's option (a′): clip the tile's rectangle to the chart grid's cell centres, using the grid's own geometry (position and length, not `chart_bounds_`), and request that**, so the request's centre is always inside the grid and no corner is ever pulled in. Planned **as the only path, replacing today's request, on the condition that the planner shows the output is cell-for-cell identical to today's wherever today's request succeeds**, over a comparison of the size used for rolker/s57_tools#63 (tens of millions of cells, several generators' worth of geometry: chart sizes from smaller than a tile to tens of kilometres, positions on both sides of both map axes and far from them, resolutions 1, 0.5, 0.1 m, tile sizes 4 and 100). If any cell differs where today's request succeeds and the difference cannot be shown to be today's code being wrong, plan it as a fallback used only when today's request fails, which is unchanged by construction there. Tests: new cases in `test_tile_edges` with its bounds-checking accessors, every cell asserted against an expectation computed in integer arithmetic, each failing on today's source: a chart on each side and at a corner with the tile's centre outside it; seams at several positions including exactly on the centre; a small chart inside a tile off its centre; the map-axis band at tile sizes 4 and 100; a chart whose own edge is near an axis (which the issue's proposed fix fails); a fine chart over a coarse one; plus guards. The pull request must say that where a coarser chart filled the strip, costs can go down as well as up (lethal to free where the finer chart shows water): "nothing that was marked becomes unmarked" does not hold for this fix. Out of scope, listed for the host: the tile pre-filter uses `chart_bounds_`, which differs from the grid's extent by up to half a chart cell.

## Plan Authored
**Status**: complete
**When**: 2026-11-24 21:56 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Plan**: `.agent/work-plans/issue-59/plan.md` at `308d1b3`

Option (a′) as the only path, replacing today's request: clip the tile to the chart grid's cell centres (grid geometry, not `chart_bounds_`), passing today's centre and length unchanged when nothing clamps, and skipping charts with no positive-area overlap. The checkpoint's condition holds: over 14 608 cases (1.51 G cells where today's request succeeds) 0 cells differ; the 358 submaps that differ are all today's request running one index past the grid (an out-of-bounds read of the chart buffer, painting nothing). Where today fails (0.95 G cells, 190.8 M wrong) 0 cells are wrong after; costs go down (9.4 M lethal to free) as well as up (8.5 M free to lethal); 17 new tests fail on today's source, 3 guards; pre-filter listed, not in scope.
