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

## Plan Review
**Status**: complete
**When**: 2026-11-24 22:15 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: ready

**Issue**: #59 — s57_layer drops a chart's data for a tile whose centre is outside the chart (gap at chart boundaries)
**Plan**: `.agent/work-plans/issue-59/plan.md` at `308d1b3`
**Branch**: `feature/issue-59`

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | One request in one loop of `generateTile`, 17 tests and 3 guards in `test_tile_edges.cpp`, comment rewordings; one code commit, cherry-picked to `jazzy` (`git diff rolling origin/jazzy -- s57_layer/` is empty, so it applies as is). |
| Issue alignment | Good | Both causes the issue review found (centre outside the grid; the epsilon band) are fixed, and each review action is taken: (a′) on the grid's own geometry, the band recorded with its widths, costs going down stated, the pre-filter listed. Serves, as checked: "The project keeps no goals or design document; in practice: charted hazards present in the costmap at every chart edge and seam." |
| File targeting | Good | `s57_layer.cpp`, `test_tile_edges.cpp`, and the two comments in `test_initialize.cpp:409-413` and `test_rolling_window.cpp:519-523`. One more comment is made conditional by the fix (finding 1). |
| Consequences | Needs work (minor) | Finding 1 (a third test comment); findings 2 and 3 (what the pull request should say about today's out-of-range read and about the pre-filter's remaining sliver). |
| Principle alignment | Good | The host's condition is met by an independent run (below). Each new test fails on today's source with the wrong-cell count the plan lists and passes with the fix; the expectation is integer arithmetic over the chart specs, independent of the layer. |
| ADR compliance | Good | 0013 (this entry), 0002 (issue worktree), 0019 (base `rolling`, `cherry-pick -x` to `jazzy`); no `.agent/` file in the code commit. |
| ROS conventions | N/A | No topic, parameter, QoS or lifecycle change. |

### Independent checks (grid_map 2.5.0 from the Rolling underlay, Release; scratch builds only)

1. **(a′) cannot be rejected, and the superset claim holds.** Arithmetic, from `GridMapMath.cpp`: the request's corners lie in the grid's cell-centre rectangle, so `positionShifted` in `boundPositionToRange` is at least r/2 inside (0, L) and the epsilon never acts; the index truncation `(max − pos)/r` lands at least half a cell from an integer, so no index reaches `size`; the requested centre is between the corners and so inside (submap min, submap max], also for a zero-length request (a cell centre is never on a cell boundary, including a one-cell grid where `grid_lo + r/2 == grid_hi − r/2`). Rounding of `grid_lo + r/2` and of grid_map's own `pos − mapPosition + L/2` is a few ulps of the magnitudes (≈1e-11 m at 1e5 m) against a margin of r/2 (0.15 m at r = 0.3, 1.9 m at r = 3.75). Measured with the real grid_map (own program, 1 000 000 random and near-aligned grid/tile pairs over 4 seeds: chart cells 0.05 to 100 m, 1 to 1500 cells a side, positions near both axes and at ±1e5 m, tile edges on, a few ulps off and a sliver either side of chart cell edges and grid edges, tiles 4 and 100 cells at 1, 0.5 and 0.1 m): (a′) rejected **0** times, **0** indices out of range; chart cells with positive-area overlap of the tile missing from the submap: only cells whose overlap is ≤ 3.0e-11 m (≤ 2 ulps at 1e5 m), none above 1e-6 m. Extra cells: only touching or rounding-level ones, which #63's paint rule paints nothing from.
2. **The host's condition.** `measure.sh` rebuilt and rerun with 8 seeds (the plan's 4 plus 4 new): every random seed and the families give 0 cells differing where today's request succeeds and 0 wrong after (F-wrongA) where it fails. My own comparison (`generateTile` of both layer copies, a different generator: 1-4 charts of arbitrary cell size, same-scale seams, layer origins 0 / ±500 m / ±1e5 m, anchors near the map axes, restricted and elevation land, 15% no data; expectation in long double, a window cell with a chart edge within 1e-6 m of its edge counted ambiguous; 24 000 cases, 8 seeds): where today's request succeeds for every chart, **215 850 tiles, 716 M cells, 0 differing**, 0 wrong against the expectation; where it fails for some chart, 304 249 tiles, 990 M cells, 190.6 M wrong today, **0 wrong after** (9 041 ambiguous); 0 out-of-range tile accesses.
3. **Today's out-of-range read is real (deployed code, `rolling` and `jazzy`).** When today's request succeeds with its south-west corner pulled in by `boundPositionToRange` and only part of epsilon surviving, `getIndexFromPosition` truncates to index `size` after its position check passed, and the submap runs one column (x) or row (y) past the grid's west or south end. Own random search: 163 such requests of 397 759 that succeeded (0.04%), 5 118 indices. Reproduced (`grid 41×1236, r = 0.6`, tile [23,27]×[20,24]: submap y index 1236 = size). On a realistic chart (1630 × 2213 cells of 3.75 m, tile corners random within ±3 km, the chart's south-west edge inside the tile): 3 842 of 291 713 succeeding requests at 100 m tiles (1.3%), all with the tile's south edge 512 to 717 m south of y = 0, just outside the band where the request fails: a vessel can meet it. Chart grids always have start index (0, 0) (`setGeometry`, then `fromMessage`), so no wrapping: `GridMap::at` (compiled in `libgrid_map_core`, Release, no Eigen assertion) addresses element `i0 + i1·rows` of a column-major float matrix: **x past the west end** reads the next column's first element, a real chart cell inside the buffer, or, for the last column, 4 bytes past its end; **y past the south end** reads `4·(i0 + 1)` bytes past the end of each layer's buffer (`get_cost_from_grid` reads up to 6 layers), i0 being the x index, up to `nx` (≈ 6.5 KB for a 1:12000 cell). The value is never used: `getPosition` rejects the index and nothing is painted. In every reproduction the address was in mapped memory; a crash needs the read to cross into an unmapped page, which a large (mmapped) buffer ending near a page boundary with no mapping after it allows, so it cannot be excluded; a `grid_map_core` built without `NDEBUG` would abort on Eigen's range assertion. (a′) removes it (0 out-of-range indices above).
4. **Costs going down** (own comparison, tiles where today's request fails, non-ambiguous cells): lethal → free 8 797 379, free → lethal 8 957 751, charted → uncharted 0. Every cell whose value changed from a charted one took the value of a chart whose request failed today, which the expectation names as the winner; today's value equals the expectation computed over the charts that succeeded today (0 mismatches). No cell lethal today becomes free where the expectation says lethal. The plan's account (finer, or same scale and first by label, now wins) holds.
5. **The band** (own sweep, real grid_map, chart 1630 × 2213 cells of 3.75 m, tile corner in 0.173 m steps from −2 to +2 km, chart edge 0.3 to 49.9 m inside 50, 100 and 400 m tiles): today fails for every corner from **−307.2 to +307.1 m** of x = 0 and from **−511.9 to +614.4 m** of y = 0, the same whatever the tile size and inset; (a′) fails 0. On a vessel: the map frame's origin is the first position fix (BEN), so this bites tiles straddling a chart's **west or south** edge that lies within about 300 m east or west of the north-south line through that fix, or about 500-600 m north or south of the east-west line through it, for a harbour cell; wider for coarser cells (±1.2 km for a 20 km chart). There today drops the chart's whole part of each such tile (up to one tile wide) along the edge.
6. **Tests.** `test_tile_edges.patch` applied to a scratch copy of `rolling`: on today's source 17 fail and 26 pass, the wrong-cell counts exactly the plan's (22, 22, 23, 16, 21, 20, 16, 21, 16, 18, 4, 24, 46, 46, 45, 4078 of 40000, 11); with `aprime.patch` 43/43 pass, and the whole `s57_layer` suite (7 gtests, cppcheck) passes. The 23 existing cases are unchanged: `accessesInRange` now reads the window and tile size from the costmap and the log, which equal `kCells`/`kTileSize` for them; their tiles lie inside the chart's cell-centre rectangle (chart from ≥ 2.75 cells before the window to 3 cells past it), so their requests are today's, except `LargeCellChartInteriorEdgesGuard1m` (a guard). The 100-cell case runs in 6 ms. Nothing in the new fixture differs on Jazzy beyond what the existing fixture already uses.
7. **Cost per tile**: `measure.sh`'s timing reproduces the plan's table (interior 61.4 → 63.0 µs, seam 40.8 → 63.8 µs at 1 m).

### Findings

1. **[Consequences]** (minor) The fix also makes the comment on `chartCellsBeyondTile` (`test_tile_edges.cpp:291-294`, "requested as generateTile requests it") conditional: after (a′) the layer requests the tile unchanged only where nothing clamps. It still holds for the four one-ulp cases that use it (tiles (2, 0) and (0, 0) lie inside the chart's cell-centre rectangle; checked), so the precondition keeps its teeth, but the comment should say so. Add it to step 3's list of rewordings.
2. **[Consequences / PR text]** (minor) State today's out-of-range read exactly in the pull request: as in check 3 above (x past the west end: another chart cell, or 4 bytes past the buffer; y past the south end: up to 4·(nx + 1) bytes past the end of each layer's buffer; value discarded, nothing painted; Release has no check; a crash cannot be excluded though none was seen; a grid_map built without NDEBUG would abort). The plan's "an out-of-bounds read of the chart's Eigen buffer … painting-neutral" is right but does not say which reads leave the buffer. No separate change is needed: the fix removes it on both branches. Whether it gets its own record is the owner's call (the plan's open question).
3. **[Scope / PR text]** (minor) The pre-filter claim holds: `getGrid` sets the length to `round(L/r)·r` centred on the bounds, so each grid edge is within r/4 of its bound; `llToMap` (GDAL WGS84 → ECEF at altitude 0, then the context's earth→map transform) and `llToWorld` (geodesy, `GeoPoint` altitude 0, then tf earth→global frame at the time of the dataset reply) agree when earth→map is static and s57_grids' `map_frame` is the costmap's global frame; not checked on vessel data. Leaving it leaves a sliver of the same kind: where a tile meets the grid only in that ≤ r/4 margin outside the stated bounds, the outermost chart cells' overlap there is dropped (≤ 0.94 m for 1:12000, ≤ 6.25 m for a 25 m cell). It is outside stated coverage, so I agree with leaving it, but the pull request should not say every chart edge is now complete; one line naming it is enough.
4. **[Tests]** (nit) On today's source 5 of the new tests (`SmallChartOffCentre1m`, the three `ChartEdgeInTileAtMap*` 4-cell cases and the 100-cell case) also fail `accessesInRange` with "the layer's tile accesses are not reaching them" because nothing is painted in the window; the real failure is the cell count beside it. Optional: skip the painted-cells check when the scene expects nothing, or leave it. The patch also adds stray blank lines (top hunk, before the #59 tests).

### Summary

The plan is ready. Its central claims check out independently: (a′) is never rejected and returns every positively-overlapping chart cell up to ulp-level rounding; output is identical to today's wherever today's request succeeds (716 M cells, own generator; plus the planner's harness rerun on 8 seeds); after the fix 0 cells are wrong; costs go down only where a chart today dropped is the expected winner. The findings are wording for one test comment and for the pull request.

### Recommended Actions

- [ ] Reword the `chartCellsBeyondTile` comment (`test_tile_edges.cpp:291-294`) to say the layer's request is today's only where nothing clamps, as for the four one-ulp cases (finding 1).
- [ ] In the pull request, state today's out-of-range read as finding 2 gives it, and that the fix removes it.
- [ ] In the pull request, name the pre-filter's ≤ r/4 sliver as left in place (finding 3).
- [ ] Optional: the misleading second failure message and the stray blank lines (finding 4).

## Checkpoint
**Status**: complete
**When**: 2026-11-24 22:16 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the plan review's verdict is ready; its independent runs confirmed the condition the host set; its findings are wording and one comment)
**After**: plan
**Decision**: proceed

Read: the Plan Review at `308d1b3`, verdict ready, three minor findings and a nit. The fix replaces today's request outright: the review's own comparison (24,000 cases, 716 million cells where today's request succeeds for every chart, a different generator) found 0 differing cells, and where today's request fails 0 cells wrong after the fix; no cell lethal today becomes free where the expectation says lethal. Findings applied: (1) the comment on `chartCellsBeyondTile` in `test_tile_edges.cpp` is reworded with the other two (the layer sends the tile unchanged only where nothing is clamped, which holds for the four one-ulp cases). (2) **The pull request states today's out-of-range read of a chart's buffer exactly**: where today's request succeeds after grid_map pulls in the south-west corner and part of its epsilon is lost, the submap runs one index past the grid's west or south end; the layer then reads a neighbouring chart cell (west) or past the end of each layer's buffer (south, up to several kilobytes), discards the value and paints nothing; about 1.3% of succeeding requests on a realistic 1:12000 chart, all just outside the map-axis band; in a Release grid_map no check fires, and a crash cannot be excluded; this fix removes it. It is recorded in the pull request and in this issue's records; whether it gets a record of its own is put to the owner. (3) The pull request does not claim every chart edge is now complete: the tile pre-filter, which is left as it is, can still skip a tile that overlaps a chart's grid only within a quarter chart cell outside the chart's stated bounds (at most 0.94 m for a 1:12000 chart); no data inside stated coverage is lost. (4) The misleading "tile accesses are not reaching them" message on today's source, where nothing is painted, is reworded or ordered after the wrong-cell count; stray blank lines removed.

## Implementation
**Status**: complete
**When**: 2026-11-24 22:32 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Branch**: feature/issue-59 at `69e800e`

### Commits
- `edd754b` s57_layer: request each chart's cells clipped to its cell centres (#59): the code commit (no `.agent/` file; message ends "Backport to jazzy."). `git merge-tree` of it onto `origin/jazzy` is clean (`s57_layer/` is identical on the two branches).
- `69e800e` records: the harness builds `HEAD` ("after") against `origin/rolling` e95d7bd ("today"); `instrument.py` counts each copy's requests, rejections and chart indices outside the grid (counted and skipped before `grid.at()`); new `oorreal` mode; changed cells classified by the chart that now wins.

### What changed
- `generateTile` (`s57_layer.cpp:614-664`): per axis, from the grid's own `getPosition()`/`getLength()`, skip a chart with no positive-extent overlap (`grid_lo >= tile_hi || grid_hi <= tile_lo`); clamp each tile edge to `[grid_lo + r/2, grid_hi - r/2]`; only if an edge moved, rebuild that axis's centre and length from the clamped edges, otherwise today's `tile_center`/`tile_size` are passed unchanged. Comment gives the two rejection causes, why a cell-centre request cannot be rejected, the superset property, and that the unclipped request is today's. A failed request keeps today's handling (paints nothing), commented as now defensive. Paint loop and the `chart_bounds_` pre-filter untouched.
- `test_tile_edges.cpp`: the plan's `Scene`/`ChartSpec` fixture, 17 cases and 3 guards as listed in the plan. Departures from the planner's patch: `check(Scene)` asserts the cells first and only checks out-of-range accesses when the cells are wrong (`accessesInRange(require_reach)`), so on today's source each case reports the wrong-cell count alone; the scene's window origin is asserted; stray blank lines dropped; header comment names #59.
- Comment rewordings: `chartCellsBeyondTile` now says the layer sends the tile unchanged only where the tile lies within the grid's cell centres, and **returns -1 otherwise, which `checkOneUlpBeyond` asserts against** (it holds for all four one-ulp cases: they pass); `test_initialize.cpp:409-412` and `test_rolling_window.cpp:519-523` say the charts reach past the tiles so the tests do not depend on chart-edge handling. Geometry unchanged.

### Tests (Rolling, grid_map 2.5.0)
- **Today's source** (scratch colcon build of `s57_layer` at e95d7bd's `s57_layer.cpp` with the new tests; `LD_LIBRARY_PATH` put first and `ldd` showed the scratch `libs57_layer.so` loaded; without it `ldd` showed the worktree's installed one, the trap): 17 failed, 26 passed (23 existing + 3 guards). Each failure printed one message only, the cell count: CentreOutsideChartWest1m 22 of 144 (first (4, 0) is 255, expected 254), …WestHalfMetre 22, …East1m 23, …South1m 16, …North1m 21, …Corner1m 20, SeamWestOfCentre1m 16, SeamOnCentre1m 21, SeamEastOfCentre1m 16, SeamOnCentreNorthHalfMetre 18, SmallChartOffCentre1m 4, FineWaterOverCoarseLand1m 24 (first (4, 0) is 254, expected 0), ChartEdgeInTileAtMapXZero1m 46, …XZeroHalfMetre 46, …YZero1m 45, ChartEdgeInTileAtMapOrigin100CellTiles1m 4078 of 40000, ChartEdgeJustWestOfMapXZero1m 11: the plan review's counts exactly.
- **With the fix**: 43/43 (`ldd`: the worktree's freshly built library); `test_tile_edges` 30 times as separate processes: 30 pass. `test.sh --packages-select s57_layer --parallel-workers 1`: 10/10 ctest (7 gtests, cppcheck, lint_cmake, xmllint), 125 tests, 0 failures, 9 skipped: all 9 are cppcheck's own skip (cppcheck 2.19 is a version ament_cppcheck skips). Run by hand with `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1`: `s57_layer.cpp` clean; `test_tile_edges.cpp` reports a syntaxError at the first `TEST_F`, the same as the file at `HEAD~` (it cannot parse gtest macros; it was already there). No new compiler warning under `-Wall -Wextra -Wpedantic` (only the existing `BUFFER_HEADER_DEPRECATION` from tf2_ros).

### Comparison on the final code (`measure.sh`, JOBS=8: families + 8 random seeds 101-108 of 2500 cases; today = origin/rolling e95d7bd, after = edd754b, both compiled into one executable)
24 608 cases, 1 094 602 tiles, 1 379 083 chart requests today.

| | Tiles | Cells | Today vs after differ | Wrong today | Wrong after | Out-of-range tile accesses |
|---|---|---|---|---|---|---|
| Today's request succeeds for every chart | 678 713 | 2 972 593 040 | **0** | 0 | 0 | 0 |
| Today's request fails for some chart | 415 889 | 1 863 887 888 | 382 616 600 | 366 496 022 | **0** | 0 |

(169 M cells ambiguous, a chart edge within 1e-6 m of a window-cell edge, almost all in the ulp-nudged family: excluded from "wrong" only; "differ" counts every cell.)
- The fix's own counters: 1 367 917 requests, **0 rejected** (11 166 fewer requests than today: charts only touching a tile, now skipped before the request; today rejected all of them).
- Changes where today fails: newly charted 346 746 590; lethal → free 18 159 324, free → lethal 17 710 686, charted → uncharted 0, other 0. Classified (non-ambiguous): lethal → free 17 187 121, of which 16 767 223 a finer chart's value replacing a coarser one and 419 898 a same-scale chart first by label (seam cells), **0 otherwise and 0 where the expectation says lethal**; free → lethal 16 722 027 (finer 16 349 220, by label 372 807, 0 otherwise, 0 where the expectation says free). Today's value equals the expectation over the charts today's request succeeded for in all 1 799 887 781 non-ambiguous changed-tile cells (599 893 more ambiguous there, not classified). Family `fine-water-over-coarse-land` alone: 221 440 lethal → free.
- **Today's out-of-range chart read**, counted inside the layer copy: of 949 112 succeeding requests today, **769 yielded 3 436 chart indices outside the grid; the fix: 0 requests, 0 indices.** The harness's own count of submaps that differ between today's and the clipped request where today's succeeds and the request is clipped: 769 of 457 838, the same 769. So every submap difference is today's run past the grid's end.
- `oorreal` (1:12000-size chart, 1630 × 2213 cells of 3.75 m, its west edge, south edge or both inside a random tile within ±3 km, no data): 100 m tiles, 300 000 tiles: today 197 415 rejected, 102 585 succeeded, **759 (0.74%) yielded 19 492 indices outside the grid**, all y past the south end, tile south edges -700 to -600 m; 50 m tiles: 825 of 103 462 (0.80%), 10 819 indices, south edges -700 to -550 m. After: 0 rejected, 0 out of range. (The plan review's 1.3% came from its own distribution, which differed; none past the west end in this set: the x band ends inside the grid's x arithmetic differently. The random set above had both.)
- **Band** (`band`, chart 1627 × 2213 cells of 3.75 m, tile corner -2 to +2 km in 0.173 m steps, insets 0.25 / 10 / 36.25 / 49 cells, 1 and 0.5 m): today fails for every corner from **-307.2 to +307.1 m of x = 0** and **-511.9 to +614.4 m of y = 0**, whatever the inset and resolution; after: 0 failures. (Uses the harness's replica of the clipped request, which is the committed code's arithmetic.)
- **Cost per `generateTile`** (two adjoining 1:12000 charts, 100-cell tiles, median of 2000 calls, chart cells / µs, today → after): 1 m interior 756 / 62.9 → 756 / 64.4; edge, centre inside 504 / 41.6 → 504 / 42.3; edge, centre outside 0 / 0.4 → 280 / 24.2; seam 504 / 42.2 → 784 / 66.1; SW corner 0 / 0.4 → 234 / 19.4. 0.5 m: interior 210 / 33.9 → 210 / 34.5; edge, centre inside 150 / 25.1 → 150 / 25.1; edge, centre outside 0 / 0.4 → 75 / 11.0; seam 150 / 23.6 → 210 / 33.8; SW corner 0 / 0.4 → 65 / 9.9. An interior tile costs about 1-2% more (the clamp); a tile only rises where today drops a chart, to at most an interior tile's cost; tens of µs against `update_timeout` 0.5 s.

### Jazzy, by reading only
`git diff 2.2.2 2.5.0` in grid_map (local clone at `~/p11-rolling-test/src/grid_map`, tags 2.2.2 and 2.5.0) is empty for `GridMap.cpp/.hpp`, `GridMapMath.cpp/.hpp`, `SubmapGeometry.cpp/.hpp`, `SubmapIterator.cpp/.hpp` and `TypeDefs.hpp`; `getPosition`, `getLength`, `getResolution` (plain accessors) and `SubmapGeometry` (`getSubmapInformation`) are the same code in 2.2.2. `std::clamp` needs C++17, which the package sets. The new test code uses only what the existing fixture already uses (grid_map `setGeometry`/`GridMapIterator`, `LayeredCostmap`, the lifecycle-node alias). The first Jazzy compile and run is the pull request's Jazzy CI job.

### Statements for the pull request (each tied to the run above)
- **Defect.** `generateTile` asked grid_map for a chart's cells with the tile's own centre and size. grid_map rejects that (1) when the tile's centre is outside the chart's grid: every tile where a chart edge or a seam lies between the centre and part of the tile, and a chart smaller than a tile and off its centre (dropped entirely); and (2) in the map-axis band: when the tile's south-west corner is on or beyond the chart's south-west edge, grid_map pulls it in by an epsilon that rounding loses near the map's x = 0 and y = 0 lines, in a band whose width grows with the chart's length: about -307 to +307 m of x = 0 and -512 to +614 m of y = 0 for a 1:12000 chart (`band`). There the chart's whole part of each tile along its west or south edge is dropped, up to a tile wide. The dropped cells stay unknown, or show a coarser chart's value. The tests reproduce each (today's failing counts above).
- **Fix.** The request is the tile clipped per axis to the chart grid's cell centres (the grid's own geometry, not `chart_bounds_`). No corner is on or beyond the grid's edge, so grid_map never pulls one in, and the centre lies at least half a cell inside the grid, so the request cannot be rejected: 0 of 1 367 917 rejected in the comparison, 0 in the band and `oorreal`. Where nothing is clipped the request is today's, the same doubles.
- **Equivalence.** Where today's request succeeds for every chart: 0 differing cells over 2 972 593 040 cells (this run), as in the plan's run (1.5 billion cells) and the plan review's independent one (716 million cells, a different generator).
- **What changes for a vessel: costs can go down as well as up.** Where no other chart covered a dropped strip, free or unknown cells become charted (346.7 M newly charted cells here). Where a coarser chart had filled the strip, the finer chart's value now wins: lethal → free where the finer chart shows navigable water (17.19 M non-ambiguous cells), free → lethal where only the finer chart shows a hazard (16.72 M). Every such change is the chart the expectation names replacing a coarser chart (or, at a seam of one scale, the chart first by label), none where the expectation says lethal (resp. free). These counts come from a constructed test set (hashed land/water, coarse all-land underlays), not from real chart data.
- **Today's out-of-range read of a chart's buffer.** Where today's request succeeds after grid_map pulls in the south-west corner and part of its epsilon is lost, the submap runs one index past the grid's west or south end; the layer reads a neighbouring chart cell or past the end of each layer's buffer, discards the value and paints nothing. About 1.3% of succeeding requests on a 1:12000-size chart in the plan review's distribution (0.74-0.80% in this run's `oorreal`, all past the south end, tile south edges 550-700 m south of y = 0), just outside the band; no check fires in a Release grid_map and a crash cannot be excluded. This change removes it: today 769 requests (3 436 indices) in the comparison and 1 584 in `oorreal`; after: 0.
- **Not fixed.** The tile pre-filter (`chart_bounds_`) can still skip a tile that overlaps a chart's grid only within a quarter chart cell outside the chart's stated bounds (at most 0.94 m for a 1:12000 chart); no data inside stated coverage. As in #63's pull request: a chart edge within floating-point rounding of a tile-cell edge may give either neighbour, and a straddled tile cell takes the east-most then north-most chart cell with data.
- **What the tests pin.** Every window cell against an integer expectation (finest chart first, then label; east-most then north-most cell with data), all tile accesses bounds-checked: the centre-outside cases on each side and a corner, seams west of, on and east of the centre and on it north-south, a small chart off a tile's centre, fine water over coarse land, chart edges at x = 0, y = 0 and the origin (4- and 100-cell tiles), a chart edge just west of x = 0; guards for a small chart over the centre, tiles wholly inside a chart (the unclipped request), a chart only touching a tile.
- **Not established.** Jazzy until its CI job; no real chart run through `s57_grids`; which coarser charts a vessel's catalogue holds; where vessels' map origins fall relative to chart edges.

### Housekeeping
Scratch work in a folder of my own in the session scratchpad, removed; `core_ws/build`, `install`, `log` removed; no process left running; nothing pushed, nothing posted.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-24 22:48 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: changes-requested

**Branch**: feature/issue-59 at `d565fd4`
**Base**: origin/rolling
**Depth**: Deep (reason: hazard-marking code deployed on a vehicle; changes which chart data reaches the costmap and can lower costs; host asked for full depth)
**Must-fix**: 1 | **Suggestions**: 5
**Round**: 1 | **Ship**: continue — one must-fix (std::clamp precondition); mechanical, but round 1 needs a re-review

Reviewed code commit `edd754b` (diff `origin/rolling...HEAD -- . ':!.agent'`). Reviewers: Claude adversarial subagent (fresh context), Gemini and Codex via `cross_model_review.sh` (both completed). All three raised the std::clamp finding independently. A fourth claim (missing `<algorithm>`) is a false positive: `s57_layer.h:4` includes it.

### Findings
- [x] (must-fix) `std::clamp(v, lo, hi)` is called with `lo > hi` for a chart grid one cell long on an axis: `grid_lo + r/2` rounds one ulp above `grid_hi - r/2` (my search: 71 of 1.64 M requests, every one a 1-cell axis, at ordinary coordinates such as P = 2039.7 m, r = 67.7 m; Codex: P = -7.97, r = 3.75; Gemini: P = 4.0, r = 0.2). That breaks std::clamp's precondition (undefined behaviour). libstdc++ without assertions returns `hi` for both edges, which gives a zero-length request at the cell centre, so it does no harm today: 0 rejected, 0 cells missed. Built with `-D_GLIBCXX_ASSERTIONS`, it aborts: reproduced with the case above, "stl_algo.h:3638 ... Assertion !(__hi < __lo) failed", exit 134. Mechanical fix: order the bounds first (e.g. `hi_b = std::max(lo_b, hi_b)`, or both bounds = `grid_position[a]` when the axis has one cell), or use `std::min(std::max(v, lo_b), hi_b)` explicitly. Add a 1-cell-axis test case with a non-binary centre. A zero-length grid cannot reach this code (`mapGridCallback` rejects length <= 0, `s57_layer.cpp:1027`). NaN geometry does not trigger it (every comparison is false, so the precondition holds and the request is today's). — `s57_layer/src/s57_layer.cpp:650-651`
- [x] (suggestion) PR text, equivalence: "0 differing cells wherever today's request succeeds" holds in the implementer's set but not in mine. Over 8 seeds of my own generator, 476 cells in 5.2 G differ where today's request succeeds for every chart. All 476 are cells where a chart cell edge lies within 1e-7 cell of a tile-cell edge: either neighbour is acceptable under #63's rounding rule, and 0 non-ambiguous cells differ. They include lethal to free. State the equivalence as "identical except where a chart edge lies within floating-point rounding of a tile-cell edge" (the adversarial reviewer saw the same ±1 touching row/column at snapped edges). — PR description / `progress.md` Implementation "Equivalence"
- [x] (suggestion) The tests cannot tell the grid's geometry from `chart_bounds_`: the fixture sets `chart_bounds_` equal to the grid's extent, and the `bounds` mutation (clip to `chart_bounds_`) passes all 43 tests. The comment at `:630-631` says the distinction matters. A chart whose stated bounds are not whole cells would separate them. — `s57_layer/test/test_tile_edges.cpp:145-160` (fixture `addChart`)
- [x] (suggestion) `TilesInsideChartGuard1m`'s comment says it pins "the request is today's, unchanged". Rebuilding centre and length on every axis (mutations `alwaysx`, `alwaysy`) passes all tests, so it pins the cells only, not the request. Reword the comment, or assert the request where the tile lies inside the cell-centre range. — `s57_layer/test/test_tile_edges.cpp:770-772`
- [x] (suggestion) The comment "By the argument above the request cannot fail" assumes finite geometry. With NaN geometry the request is today's and grid_map rejects it, as before. Say so in a few words. — `s57_layer/src/s57_layer.cpp:661-662`
- [x] (suggestion) Owner question for the PR: at a seam of one scale, the winner is the chart whose label sorts first (`std::map<std::pair<resolution, label>>`, `:566`, first writer wins). That order is pre-existing and undocumented. On real charts, `getGrid` rounds each grid to whole cells centred on its bounds, so abutting same-scale grids can overlap by up to r/2 (≤ 1.9 m for 3.75 m cells). Their edge cells hold data, so in that strip this fix can turn lethal to free (or free to lethal) where today the label-first chart was dropped. This is the rule the rest of the seam already follows. Whether lethal should win at a same-scale tie is for the owner. — PR description

### Evidence (own runs; grid_map 2.5.0, Release; scratch builds only)
- **Superset and no rejection** (own program, real grid_map, 2 M random and near-aligned grid/tile pairs: chart cells 0.05-100 m, 1 cell to 100 km, positions to ±1e6 m, tile edges on, one ulp off, 1e-12 to 1e-3 m off, cell edges and centres, plus 500 k one-cell grids). The committed request: 1 642 264 requests, **0 rejected**, 0 indices out of range. Cells overlapping the tile by more than 1e-6 m missing from the submap: 0 (largest missed overlap 1.9e-10 m). No extra cell beyond one that only touches. Skipped charts that overlapped: 0.
- **Old vs new through `generateTile`** (both layer copies in one executable; own generator: 1-3 charts, including two adjoining same-scale charts built as getGrid builds them, over a coarse chart; scales aligned and not; positions near both map axes and at ±9e4 m; layer origins 0, ±50 m and ±1e5 m; resolutions 1, 0.5, 0.25 and 0.1 m; tiles of 4, 10, 25 and 100 cells; 15% no data; expectation in long double, a cell ambiguous when an edge lies within 1e-7 cell; 24 000 cases, 8 seeds):
  - Today's request succeeds for every chart: 51.5 M tiles with requests, 5.22 G cells, **476 differ, all ambiguous**; 0 wrong after.
  - Today's request fails for some chart: 1.91 M tiles, 635 M cells, 110.6 M wrong today, **0 wrong after** (13.8 M ambiguous). Today's value equals the expectation over today's succeeding charts: 0 mismatches.
  - Lethal to free: 7 200 332. **0 that are not the expectation's value, 0 where the expectation says lethal.** Of these, 6 908 908 are a finer chart's value; 291 396 are a same-scale chart first by label (111 754 of them in the adjoining-seam scenario); 28 are cells ambiguous for today's expectation.
  - Free to lethal: 3 899 691. Newly charted: 99.5 M. Charted to uncharted: 0.
  - Today rejected 1 951 973 of 60.9 M requests; the fix rejected 0 of 60.87 M.
- **Out-of-range read** (instrumented): today, 582 requests yielded chart indices past the grid: 1 023 past the **west** end (x index = size) and 440 past the **south** end (y index = size), none east or north. The search program found both too (west 209, south 217). The fix: 0. So the implementer's unverified sentence (both ends) holds. Today reads the next column's first element (x), or past the end of each layer's buffer (y), discards the value and paints nothing (`getPosition` rejects the index).
- **Tests**: with only the source reverted (scratch build, `ldd` confirmed the scratch library), the 17 fail with exactly the stated wrong-cell counts. Each message is the cell count. For example, FineWaterOverCoarseLand1m gives "first (4, 0) is 254, expected 0". The 23 existing cases and 3 guards pass. Mutations of HEAD (`test_tile_edges`, 43 tests):
  - far index without `- 1` (#63): x fails 31, y fails 32 (existing cases included).
  - today's request restored: fails exactly the 17.
  - no half-cell inset: x fails the three XZero / JustWest cases; y fails YZero.
  - skip test strict (each comparison, on x and on y), skip removed, centre as `(lo+hi)/2`: caught by nothing. Each paints the same cells (a touching chart's request yields only a touching column, which paints nothing).
  - rebuild always, `chart_bounds_` for the grid: caught by nothing (findings above).
- Suite (scratch workspace at HEAD, `--parallel-workers 1`, ctest `-j1`): 125 tests, 0 failures, 9 skipped. `test_tile_edges` passed 30 of 30 runs. No new compiler warning.
- **Jazzy, by reading**: `git diff 2.2.2 2.5.0` is empty for GridMap, GridMapMath, SubmapGeometry, SubmapIterator, GridMapIterator and TypeDefs. `s57_layer/` is identical on `origin/rolling` and `origin/jazzy`. A cherry-pick merge of `edd754b` onto `origin/jazzy` is clean. The code commit touches no `.agent/` file. `std::clamp` is C++17, set in CMakeLists.
- **Comments**: the code comment's account of both rejection causes matches `GridMapMath.cpp` (`boundPositionToRange` `<= 0`, the final `getIndexFromPosition`). Its superset claim matches the search. The three reworded test comments are accurate.

## Implementation
**Status**: complete
**When**: 2026-11-24 23:02 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**Branch**: feature/issue-59 at `4657084`
**Addressed**: Local Review (Pre-Push) at `d565fd4` (2026-11-24 22:48 -05:00)
**Commits**: `b39ec5b`, `21d9838`, `4dbfa81`, `f2a0230` (code; each ends "Backport to jazzy.", none touches `.agent/`); `4657084` (records: boxes ticked)

### Actions
- [x] (must-fix) std::clamp with crossed bounds for a one-cell grid axis — `s57_layer/src/s57_layer.cpp:650-660` (`b39ec5b`). `centre_lo = grid_lo + r/2`, `centre_hi = grid_hi - r/2`; `if(centre_hi < centre_lo)` both become `grid_position[a]` (comment: "One cell on this axis: the two can round an ulp apart in either order, and std::clamp needs them ordered. Use the cell's centre."); then `std::clamp(tile_lo[a], centre_lo, centre_hi)` and the same for `tile_hi`. Unchanged wherever the bounds were ordered. Tests `OneCellLongOnX` / `OneCellLongOnY`: one cell of 3.75 m centred at -7.97 m on the short axis, four cells from -12.6 m on the other, 12 x 12 window of 1 m from (-12, -12), 4-cell tiles; each first `ASSERT_GT(grid_lo + half, grid_hi - half)` with the layer's own arithmetic on the grid's `getPosition/getLength/getResolution` (holds: 8.9e-16 apart), then every window cell against an expectation in integer 5 mm units, and the accesses.
- [x] (suggestion) equivalence wording for the pull request — statement below.
- [x] (suggestion) tests could not tell grid geometry from `chart_bounds_` — `s57_layer/test/test_tile_edges.cpp` (`21d9838`): `ChartSpec::bounds_inset_q` (stated bounds = grid extent inset on every side, outset if negative, as `S57Dataset::getGrid` + `GridMap::setGeometry` make them: `round(L / r)` cells centred on the stated bounds; `check(Scene)` asserts `4 |inset| < cell_q`); `addChart` takes the inset; `expectedCost(Scene)` counts a chart only in tiles the `chart_bounds_` pre-filter passes (no change for existing cases). New: `GridLargerThanBoundsWest1m`, `GridLargerThanBoundsNorth1m`, `GridSmallerThanBounds1m`, `GridLargerThanBoundsPreFilterSkips1m`.
- [x] (suggestion) `TilesInsideChartGuard1m` comment — `test_tile_edges.cpp` (`4dbfa81`): reworded to say it pins the cells only. No deterministic case exists: the rebuilt request differs by at most an ulp, which moves the submap's edge only where it lies within rounding of a chart cell edge, gaining or losing a cell that only touches the tile (paints nothing).
- [x] (suggestion) "request cannot fail" assumes finite geometry — `s57_layer.cpp:670-674` (`f2a0230`): says so; with NaN every comparison is false, the request is the tile's own and grid_map rejects it, as before.
- [x] (suggestion) same-scale tie, owner question — statement below.

### Evidence (Rolling, GCC 15.2, Ubuntu 26.04, grid_map 2.5.0)
- **Finding on the toolchain**: GCC 15's libstdc++ defines `_GLIBCXX_ASSERTIONS` itself in any unoptimized build (`c++config.h:597-604`, `! defined(__OPTIMIZE__)`), and this package's colcon build has no `CMAKE_BUILD_TYPE`. So on this machine the unfixed source (`edd754b`) **aborts the new test in the package's own build**, without any flag: `stl_algo.h:3638 ... Assertion '!(__hi < __lo)' failed`, exit 134. A deployment built that way on a GCC 15 machine would have aborted the costmap node on a one-cell-axis chart. Jazzy's GCC 13 has no such default (from GCC 15's release notes, not run here); there, only `-D_GLIBCXX_ASSERTIONS` shows it. The test comment says this. The flag is not added to the package.
- **Before/after, scratch builds** (layer source compiled into the test executable, nothing installed linked): unfixed `edd754b` + final test, `-O2 -D_GLIBCXX_ASSERTIONS`: aborts in `OneCellLongOnX` and `OneCellLongOnY` (each run alone: exit 134); unfixed, `-O0` (the default build): aborts the same; unfixed, `-O2` without the flag: 49/49 pass (libstdc++ returns `hi`, same cells, so the committed test cannot fail on the unfixed source in an optimized build without assertions). Fixed (`f2a0230`), `-O2 -D_GLIBCXX_ASSERTIONS`: 49/49 pass.
- **One-cell sweep** (scratch gtest, final layer source, `-O2 -D_GLIBCXX_ASSERTIONS`): 6 000 charts one cell long on x or y, cells 0.2-70.2 m in 0.01 m steps, centres to ±1e5 m in 0.01 m steps, 1-4 cells on the other axis, resolutions 1/0.5/0.25/0.1 m, tiles 4/10/100: **433 with crossed centres, no abort**, 0 cells wrong against the chart's rectangle (1e-6 cell margin), 0 out-of-range tile accesses, 28.1 M painted cells. The unfixed source aborts in the same sweep.
- **Stated bounds vs grid, mutations** (scratch, `-O2`): clip to `chart_bounds_` in the new block (grid_lo/hi from `chart_bounds_`): **fails `GridLargerThanBoundsWest1m` (12 of 144 cells, first (7, 0) 255 vs 254) and `GridLargerThanBoundsNorth1m` (11 of 144)**, 47 pass. Pre-filter using the grid's extent: fails `GridLargerThanBoundsPreFilterSkips1m` (12 of 144), so that case does pin the pre-filter. `GridSmallerThanBounds1m` passes under the `chart_bounds_` mutation: where the grid is smaller, a clip to the stated bounds yields only the grid's edge column or row, wholly outside the tile, which paints nothing; I found no geometry with `|inset| < r/4` where the mutation changes a cell when the grid is smaller (its corners always stay inside the edge cell). The difference decides the outcome only where the grid reaches past the stated bounds and a tile edge lies exactly on the stated bound (the pre-filter compares with `>`/`<`, so it passes that tile); strictly between, the pre-filter skips the tile: the known quarter-cell limit, pinned as current behaviour, not wanted behaviour.
- **Suite**: `test.sh --packages-select s57_layer --parallel-workers 1 --ctest-args -j1`: 10/10 ctest, **131 tests, 0 errors, 0 failures, 9 skipped** (cppcheck's own skip, as before). `test_tile_edges`: 49 tests, **30 of 30 runs pass** (`ldd`: the freshly built library).
- **Harness comparison** (`.agent/work-plans/issue-59/harness/measure.sh`, JOBS=8, SEED0=100; today = origin/rolling `e95d7bd`, after = HEAD `f2a0230`): 24 608 cases, 1 094 602 tiles. Today's request succeeds for every chart: 678 713 tiles, **2 972 593 040 cells, 0 differ, 0 wrong after**. Today's request fails for some chart: 415 889 tiles, 1 863 887 888 cells, 366 496 022 wrong today, **0 wrong after**. After: 1 367 917 requests, **0 rejected, 0 yielding chart indices outside the grid** (today: 769 requests, 3 436 indices). Lethal → free / free → lethal / newly charted 18 159 324 / 17 710 686 / 346 746 590, 0 charted → uncharted. Band and `oorreal` unchanged (band -307.2 to 307.1 m on x, -511.9 to 614.4 m on y today, 0 after; `oorreal` 759 and 825 today, 0 after). **Identical figures to the previous run**: the clamp change altered nothing there (the harness has no one-cell-axis crossings that change a cell).

### Statements for the pull request (corrected)
- **Equivalence, qualified.** Where today's request succeeds for every chart, output is identical, except in cells where a chart cell edge lies within floating-point rounding (1e-7 cell) of a tile-cell edge, where either neighbour may be taken, as rolker/s57_tools#63's rule states. The code review's own generator: 476 such cells in 5.2 billion, 0 others; the two earlier runs: 0 in 1.5 billion and 0 in 716 million; this run: 0 in 2.97 billion.
- **Today's out-of-range read** reaches past the **west** end of a chart grid (the next column's first element, a real chart cell) and past the **south** end (past the end of each layer's buffer), never east or north (the review's instrumented run: 1 023 and 440 indices). The layer discards the value and paints nothing; no check fires in a Release grid_map. This change removes it (0 after).
- **The same-scale tie.** Charts are ordered by resolution and then by label, and the first writer wins (pre-existing, `s57_layer.cpp:566`). Abutting same-scale grids overlap by up to half a chart cell because `getGrid` rounds each grid to whole cells, so in that strip the chart whose label sorts first wins. This fix extends that existing rule to tiles where today's request dropped a chart, which can turn a cell from lethal to free there. Whether lethal should win such a tie is a question for the owner, not changed here.
- **One-cell axis.** The first version of this fix passed crossed bounds to `std::clamp` for a chart grid one cell long on an axis (undefined behaviour); with GCC 15 an unoptimized build aborts there. Fixed before publishing; the bounds are now ordered.
- Other statements of the previous Implementation entry stand (defect, fix, costs can go down as well as up, the pre-filter's quarter-cell limit not fixed, what is not established).

### Checks
Package suite and `test_tile_edges` ×30 as above; no new compiler warning (only the existing `BUFFER_HEADER_DEPRECATION`). Pre-commit hooks ran on every commit. Scratch work was in a folder of my own in the session scratchpad, removed; `core_ws/build`, `install`, `log` removed; no process left running; nothing pushed, nothing posted.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-24 23:09 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: approved

**Branch**: feature/issue-59 at `7944007`
**Base**: rolling
**Depth**: Standard (reason: round 2 narrow check of round 1's findings and the fix diff `edd754b..f2a0230`, per handoff; no cross-model reviewers and no adversarial subagent, per handoff)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 2 | **Ship**: recommended — no must-fix findings; remaining suggestions can be applied or tracked

### Findings
- [x] (suggestion) The one-cell tests exercise only one crossing direction: at -7.97 m / 3.75 m, `grid_lo + r/2` is an ulp above the centre and `grid_hi - r/2` equals it, so a fix that resets only `centre_lo` (mutation run here: `centre_lo = grid_position[a]` alone) passes all 49 tests, while one that resets only `centre_hi` aborts. A case where `grid_hi - r/2` rounds below the centre and `grid_lo + r/2` equals it (for example a 3.75 m cell centred at 0.47 m, or 0.2 m at 0.08 m: found by evaluating the layer's expressions in doubles) would cover the other order. — `s57_layer/test/test_tile_edges.cpp` (`checkOneCellAxis`)
- [x] (suggestion) `b39ec5b`'s message says the tests "abort in std::clamp in an unoptimized or -D_GLIBCXX_ASSERTIONS build" on the unfixed source; for an unoptimized build that holds with GCC 15 only (the paragraph above says so, this one does not), and the commit is to be cherry-picked to jazzy, where GCC 13's unoptimized build passes them. Reword before pushing ("an unoptimized GCC 15 build"), or leave it: the test's comment is right. — commit `b39ec5b` message (deferred: not changed: history is not rewritten; the test's comment and the pull request state it correctly (an unoptimized GCC 15 build, or -D_GLIBCXX_ASSERTIONS))

### Evidence (own runs; this machine: GCC 15.2, grid_map 2.5.0)
- **Toolchain**: `/usr/include/x86_64-linux-gnu/c++/15/bits/c++config.h:597-604` defines `_GLIBCXX_ASSERTIONS` when `!__OPTIMIZE__` (GCC 15 is the only libstdc++ installed here). The package's `CMakeLists.txt` sets no build type; the package worktree's `build.sh` and `test.sh` pass only `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`; the build here had `CMAKE_BUILD_TYPE` empty, no `-O`. The CI's jazzy job runs plain `colcon build` too, but on GCC 13 (no such default), and the rolling CI job does not build `s57_layer`; so on CI the one-cell tests are guards, as their comment says. The project's own `~/project11/rolling/core` build also has `CMAKE_BUILD_TYPE` empty for 29 packages, among them `bathymetry_layer` (a costmap plugin), so workspace-built plugins on this machine run unoptimized with libstdc++ assertions on; the session's harness measurements were stated as Release builds.
- **`b39ec5b`**: unfixed `edd754b` in the default build: `OneCellLongOnX` and `OneCellLongOnY` each abort (`stl_algo.h:3638 ... '!(__hi < __lo)'`, exit 134); the other 47 pass. Fixed: identical to the old code wherever the bounds are ordered (same expressions); for two or more cells the bounds are a cell apart, far beyond rounding. The cell's centre is strictly inside the grid whenever r exceeds an ulp of the coordinate (any chart). NaN: `centre_hi < centre_lo` is false, `std::clamp(v, NaN, NaN)` passes its assertion and returns `v`, so the request is the tile's own, as the new comment says. Mutations of the clamp lines: only `centre_hi` reset: aborts; only `centre_lo` reset: passes (first finding); `std::swap` of the two: passes (also correct behaviour).
- **One-cell precondition on Jazzy**: grid_map 2.2.2's `setGeometry` is identical to 2.5.0's (`git show 2.2.2:grid_map_core/src/GridMap.cpp`): size `round(L/r)` = 1 and 4, length `size * r` = 3.75 and 15 exactly, position stored as given; `getPosition`/`getLength`/`getResolution` are plain accessors. The arithmetic is additions of exact halves to the literal -7.97 in IEEE double (`(p - 1.875) + 1.875` = p + 8.9e-16, `(p + 1.875) - 1.875` = p); no product to contract into an FMA (and `L*0.5` would be exact anyway), no reassociation without `-ffast-math`. The crossing is the same bit for bit on Jazzy's x86_64 GCC 13: the `ASSERT_GT` cannot fail there.
- **`21d9838`**: `getGrid` calls `setGeometry(L, r)` then `setPosition` at the stated bounds' centre, so the grid is `round(L/r) * r` long centred on the bounds: each side differs by at most r/4, larger or smaller; the fixture's `4 |inset_q| < cell_q` is that bound with the tie excluded. `expectedCost`'s pre-filter is its own statement of the rule in integer quarter units (skip when west > tile end or east < tile start, and so on), matching the layer's `>`/`<` by its own statement, not by calling layer code; its comment and the `PreFilterSkips` case's comment label that as pinning current, not wanted, behaviour. The comments' quarter positions for all four cases are right. `chart_bounds_` mutation in the new block: fails `GridLargerThanBoundsWest1m` (12 of 144, first (7, 0) 255 vs 254) and `GridLargerThanBoundsNorth1m` (11 of 144), 47 pass. `s57_layer.cpp` at `origin/rolling` (`e95d7bd`) with the final test: the 17 earlier cases fail with the stated counts exactly (22, 22, 23, 16, 21, 20, 16, 21, 16, 18, 4, 24, 46, 46, 45, 4078 of 40000, 11), plus the two `GridLargerThanBounds*` cases (12, 11) and both one-cell cases (36 cells each); the 23 existing cases, 3 guards, `GridSmallerThanBounds1m` and `PreFilterSkips` pass.
- **`4dbfa81`, `f2a0230`**: comments true (above).
- **Suite** (package worktree, default build type, i.e. unoptimized): `build.sh --packages-up-to s57_layer --parallel-workers 4`; `test.sh --packages-select s57_layer --parallel-workers 1 --ctest-args -j1`: 131 tests, 0 errors, 0 failures, 9 skipped (cppcheck's own skip). `test_tile_edges` 30 of 30 runs pass (49 tests; `ldd`: the worktree's freshly built library). Scratch mutation builds: `LD_LIBRARY_PATH` first, `ldd` showed the scratch library. No new compiler warning.
- **Hygiene**: the four commits touch only `s57_layer/`, one change each, subjects name #59, bodies end "Backport to jazzy." before the trailers. The five code commits `edd754b`, `b39ec5b`, `21d9838`, `4dbfa81`, `f2a0230` cherry-pick onto `origin/jazzy` in that order without conflict (scratch worktree, removed). Branch contains `origin/rolling`.
- **PR statements** (newest Implementation entry): the qualified equivalence, the west/south out-of-range read, the same-scale tie (`std::map<std::pair<resolution, label>>`, `s57_layer.cpp:566`), the one-cell clamp (verified above) and the pre-filter limit (pinned by `PreFilterSkips`) are supported. Harness figures not rerun.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 23:10 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (round 2 approved with no must-fix; one of its two suggestions closes a gap in what the tests pin about the fix made in round 1, and is cheap)
**After**: publish
**Decision**: address

Read: the Local Review (Pre-Push) round 2 at `f2a0230`, approved, two suggestions. One short pass, then a narrow re-review, then publish. (1) The one-cell tests exercise one crossing direction only (`grid_lo + r/2` an ulp above the centre): a fix that reset only `centre_lo` passes all 49 tests. Add the mirrored case, where `grid_hi - r/2` rounds below the centre and `grid_lo + r/2` equals it (the review suggests a 3.75 m cell centred at 0.47 m, or 0.2 m at 0.08 m: verify by evaluating the layer's expressions), on x and on y, with the precondition asserted, and show that resetting only `centre_lo`, and resetting only `centre_hi`, each now fails a test in an assertions-on build. (2) The message of `b39ec5b` says the tests abort on the unfixed source "in an unoptimized or -D_GLIBCXX_ASSERTIONS build"; for an unoptimized build that holds with GCC 15 only. History is not rewritten for it; the test's comment is right, and the pull request states it correctly.

## Implementation
**Status**: complete
**When**: 2026-11-24 23:16 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**Branch**: feature/issue-59 at `279ddef`
**Addressed**: Local Review (Pre-Push) at `7944007` (2026-11-24 23:09 -05:00)
**Commits**: `8c650af` (test only, `s57_layer/test/test_tile_edges.cpp`, ends "Backport to jazzy."); `279ddef` (records: boxes ticked)

### Actions
- [x] (suggestion) one-cell tests covered one crossing direction only — `s57_layer/test/test_tile_edges.cpp` (`checkOneCellAxis`, `8c650af`). The helper now takes the centre (and the same in 5 mm units) and a `Crossing` (`low_above`, `high_below`); it asserts `centre_hi < centre_lo` with the layer's own expressions on the grid's `getPosition/getLength/getResolution`, then for `low_above` `centre_lo > position` and `centre_hi == position`, for `high_below` `centre_hi < position` and `centre_lo == position`; then every window cell against the integer expectation (cell span `centre_q ± 375`) and the accesses. Existing pair renamed `OneCellLongOn{X,Y}LowCentreAbove` (3.75 m cell at -7.97 m: `lo - p` = +8.9e-16, `hi - p` = 0); new pair `OneCellLongOn{X,Y}HighCentreBelow` (3.75 m cell at 0.47 m: `lo - p` = 0, `hi - p` = -2.2e-16; the cell spans -1.405 to 2.345 m, so it reaches window columns/rows 10 and 11). The comment saying where these can fail without the fix (assertions-on: GCC 15 unoptimized or `-D_GLIBCXX_ASSERTIONS`; otherwise cells only) is kept and now names both directions.
- [x] (suggestion) `b39ec5b`'s message on "unoptimized" — not changed (deferred: history is not rewritten; the test's comment and the pull request state it correctly).

### Evidence (Rolling, GCC 15.2)
- **Values**: found by evaluating `(p - L/2.0) + r/2.0` and `(p + L/2.0) - r/2.0` in doubles for L = r = 3.75: p = 0.47 gives lo = 0.46999999999999997 (= p), hi = 0.46999999999999975; p = -7.97 gives lo = p + 8.9e-16, hi = p. (0.2 m at 0.08 m also crosses the high-below way; 3.75 m was kept so one fixture shape serves both.)
- **Jazzy (GCC 13, x86-64)**: for 0.47 as for -7.97, the arithmetic is only additions and subtractions of exact values (L = 1 × 3.75 and the halves 1.875 are exact; the division by 2.0 is exact) to the literal; no product that could be contracted (x86-64 without `-march` has no FMA, and a contracted `L*0.5` is exact anyway). The crossing is the same bit for bit; grid_map 2.2.2's `setGeometry` is as 2.5.0's (review round 2).
- **Mutations** (scratch builds, `-O2 -D_GLIBCXX_ASSERTIONS`, layer source compiled into the test, `ldd`: no `libs57_layer`):
  - full fix: `[  PASSED  ] 51 tests.`, each of the four one-cell tests passes alone.
  - only `centre_lo` reset: `OneCellLongOnXHighCentreBelow` and `OneCellLongOnYHighCentreBelow` each exit 134, `stl_algo.h:3638: ... std::clamp ... Assertion '!(__hi < __lo)' failed.`; the two `LowCentreAbove` pass; the whole run aborts (exit 134).
  - only `centre_hi` reset: `OneCellLongOnXLowCentreAbove` and `OneCellLongOnYLowCentreAbove` each exit 134 with the same assertion line; the two `HighCentreBelow` pass; the whole run aborts.
- **Suite**: `build.sh --packages-up-to s57_layer --parallel-workers 4`; `test.sh --packages-select s57_layer --parallel-workers 1 --ctest-args -j1`: **133 tests, 0 errors, 0 failures, 9 skipped** (cppcheck's own skip). `test_tile_edges`: 51 tests, **30 of 30 runs pass** (`ldd`: the worktree's freshly built library).

### Statement for the pull request (corrected, about `b39ec5b`)
On the unfixed code the one-cell tests abort in an assertions-on build (GCC 15 unoptimized, which is this workspace's default build on Ubuntu 26.04, or any build with `-D_GLIBCXX_ASSERTIONS`); on GCC 13 without that flag (Jazzy CI) they check the cells only. They now cover both directions in which a one-cell axis's two centres can cross.

### Checks
Pre-commit hooks ran on both commits. Scratch work in a folder of my own in the session scratchpad, removed; `core_ws/build`, `install`, `log` removed; no process left running; nothing pushed, nothing posted.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-24 23:17 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)
**Verdict**: approved

**Branch**: feature/issue-59 at `8c650af`
**Base**: rolling
**Depth**: Standard (reason: round 3 convergence check per handoff; one test-only commit, no cross-model reviewers, cross-model skipped)
**Must-fix**: 0 | **Suggestions**: 0
**Round**: 3 | **Ship**: recommended — no must-fix; the diff since round 2 is test-only, source unchanged

### Findings
- [ ] No issues found. LGTM.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 23:18 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (pre-push review round 3 approved with no findings)
**After**: publish
**Decision**: publish

Read: the Local Review (Pre-Push) round 3 at `8c650af`, approved, nothing open. Six code commits: `edd754b`, `b39ec5b`, `21d9838`, `4dbfa81`, `f2a0230`, `8c650af`. The pull request's Jazzy CI job is the first Jazzy compile and run.

## Integrated Review
**Status**: complete
**When**: 2026-11-24 23:23 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)
**Reviews-read**: none

**PR**: #68 at `ce41b10`
**Sources**: 2 (Local Review (Pre-Push) rounds 1-3, last at `8c650af`, which covers the head through bookkeeping-only commits; CI rollup). No pull-request review, inline comment or conversation comment exists (no human, Copilot or bot).
**Cross-source confirmations**: 0
**CI**: all-pass

### Findings
- [x] No open findings. The local timeline holds nothing open: round 1's must-fix and five suggestions were taken, round 2's second suggestion was ruled not changed by the host, round 3 approved with none. Known and not findings: the pre-filter's quarter-cell limit, the within-rounding behaviour, the straddle rule, the same-scale tie.

### False positives
- (none: no bot or human comment to dismiss)

### CI evidence (run 45545835177, both jobs SUCCESS)
- Jazzy job 138975109814 built the merge commit `28d9bbd` ("Merge ce41b10a... into e95d7bd"), where `e95d7bd` is the current tip of `rolling`, so it builds the head's tree. `s57_layer` was built there ("Finished <<< s57_layer [49.5s]") from `src/s57_tools/s57_layer`.
- `7: [==========] Running 51 tests from 1 test suite.`, `7: [  PASSED  ] 51 tests.`, `7/10 Test #7: test_tile_edges ... Passed 0.52 sec`, `100% tests passed, 0 tests failed out of 10`, `Summary: 316 tests, 0 errors, 0 failures, 33 skipped`. 51 cases are listed OK; no FAILED, DISABLED or SKIPPED line in the test_tile_edges output.
- Cases named: one-cell OneCellLongOnXLowCentreAbove, OneCellLongOnYLowCentreAbove, OneCellLongOnXHighCentreBelow, OneCellLongOnYHighCentreBelow; map-axis ChartEdgeInTileAtMapXZero1m, ChartEdgeInTileAtMapXZeroHalfMetre, ChartEdgeInTileAtMapYZero1m, ChartEdgeInTileAtMapOrigin100CellTiles1m (and ChartEdgeJustWestOfMapXZero1m); FineWaterOverCoarseLand1m; grid versus bounds GridLargerThanBoundsWest1m, GridLargerThanBoundsNorth1m, GridSmallerThanBounds1m, GridLargerThanBoundsPreFilterSkips1m. All OK.
- Compiler warnings: none from `s57_layer.cpp` or `test_tile_edges.cpp` (GCC 13.3.0 on Ubuntu 24.04). The only build warnings are `-Wlogical-not-parentheses` in `unh_marine_autonomy/.../gz4d_geo.h`, not this change.
- Installed: `ros-jazzy-grid-map-core 2.2.2-2noble.20261025.230136`; `ros-jazzy-nav2-costmap-2d 1.3.14-1noble.20261105.140901`.
- The Rolling job (`build-and-test (rolling)`, 138975109522) does not build `s57_layer` (no mention of it in its log).

### Pull request body
Replace "This pull request's Jazzy CI job is the first Jazzy compile and run." with the result (see the final message). No unsupported count found: 23 before and 51 after match `TEST` counts in `rolling` and the branch; the 17 failures were of the first batch (17 fail, 26 pass of 43) per progress.md; the six code commits match the commit list.
