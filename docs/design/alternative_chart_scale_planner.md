# Alternative: Chart-Scale Collection Planner — Starting Point

A second alternative design for ENC-driven path planning, captured here
as a starting point for a later discussion. **This is not a worked-out
architecture.** It is the user's idea recorded faithfully, ready to be
picked up and developed.

The intent is to have it on file alongside
[`alternative_costmap_architecture.md`](alternative_costmap_architecture.md)
and [`prior_art_reed_2018.md`](prior_art_reed_2018.md) so the eventual
comparison exercise has three meaningfully distinct designs to weigh
against each other (this one, the clean-room costmap design, and Reed's
work).

## The mental model

Plan the way a human mariner plans a voyage between two harbors —
e.g., Portsmouth, NH to Boston, MA:

1. Pull the largest-scale (most detailed) chart for the **start**
   (Portsmouth Harbor).
2. Notice the start chart doesn't reach the destination.
3. Find a chart at a coarser scale that **covers both** Portsmouth
   and Boston. Plan most of the voyage on that.
4. Pull the largest-scale chart for the **destination** (Boston
   Harbor). Plan the close-quarters approach there.
5. The composite plan: large-scale departure → coarse-scale transit
   → large-scale arrival.

This is how navigators actually do it. The collection of charts each
person uses isn't a single data structure at one resolution — it's a
working set assembled per-voyage, with scale chosen to match the
operational phase.

## Algorithm sketch (chart collection assembly)

```
1. For start point and end point, find the largest-scale chart
   containing each.
2. Iteratively add the next largest scale to the collection from
   each end (working outward in scale).
3. Continue until a chart is found at a coarse-enough scale that
   contains BOTH start and end.
4. If no single chart spans both, find the smallest-scale charts
   that can be joined together to cover the gap between the
   two end-collections.
```

The output is a *collection* of charts at varying scales, not a
single map at one scale. Together they cover the full voyage with
appropriate resolution everywhere along it: fine near departure
and arrival, coarse in the middle.

## Planner integration

With the chart collection assembled:

- The planner starts on the largest-scale start chart.
- When the plan would drive off that chart, the planner jumps to the
  next chart up in the collection.
- Continues across charts in scale order until reaching the
  largest-scale destination chart.

Two ways to feed cost into the planner:

### Per-chart rasterization

Each chart is converted into a costmap at a resolution appropriate
for that chart's scale. This implies the planner expects a
**fixed-resolution underlying grid** but the resolution is per-chart,
not global.

To fit nav2, check whether the cost calculation step can be supplied
as a plugin — if so, each chart in the collection contributes its
own costmap and the planner navigates across the collection.

### Vector-direct (skip rasterization)

If the planner can call out to a cost function rather than reading a
grid, the chart never needs to be rasterized at all. Vector features
are queried directly. This avoids:

- Choosing a rasterization resolution for each chart.
- Loss of feature precision during rasterization.
- The memory cost of large rasters at fine resolution.

## Operational behavior

The planner runs continuously:

- As the boat progresses, the relevant charts in the collection
  update (charts behind the boat can be dropped; charts ahead may
  be added if the route changes).
- The plan adjusts as conditions change (tide, vessel state, observed
  hazards).

## Open questions for the next discussion

A non-exhaustive list of things to work out when this gets picked up:

- **How do "scale" and "largest/smallest" map to ENC metadata?** S-57
  cells carry compilation scale (`CSCL`) and a usage band; the
  algorithm needs to be precise about which of these drives chart
  selection.
- **What does "next largest scale" mean algorithmically?** Is it the
  next chart in `CSCL` order that contains the current point, or
  something more nuanced (e.g., the next chart whose footprint
  extends *toward* the destination)?
- **Chart boundaries.** Overlap zones, gaps between cells at the same
  scale, scale mismatches at the boundary — how does the planner
  handle the seam?
- **Where does "next chart up" decision happen?** Per-step in the
  search, per-segment after a partial plan, or globally as a
  pre-decomposition into legs?
- **nav2 plugin point.** Is the right plugin point a costmap layer, a
  global planner, or a custom planner that operates on the chart
  collection directly? Stock nav2 global planners assume a single
  costmap; this approach probably wants something bespoke at the
  planner level.
- **Vector-direct planning.** Stock nav2 planners read grids; a
  vector-cost approach would likely require a custom planner. Worth
  knowing what that costs in development effort vs. what we gain.
- **What about the "no chart covers this gap" case?** The algorithm
  has a fallback (smallest-scale charts joined together) — what
  happens if no joinable set exists? Does that just fail, or is there
  a different recovery?
- **Tide and dynamic state across scales.** A single tide value is
  fine on a harbor-scale chart but not over a 100-km transit chart
  (covered in the costmap-architecture doc's section 4). How does
  this interact with the per-chart cost generation?

## What this is *not*

- Not a critique or alternative to the costmap-architecture doc — a
  parallel design.
- Not analyzed for feasibility or performance yet.
- Not committed to as the path forward.

This file gets picked up next time we discuss alternatives.
