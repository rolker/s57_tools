# Alternative ENC → Costmap Architecture

A clean-room design for generating nav2-compatible costmaps from S-57 Electronic
Navigational Chart (ENC) data, produced **without consulting the existing
implementation in this repo**. This document records the design exercise that
produced the architecture: not just the conclusions, but the questions asked,
the options considered, and the reasoning behind each decision.

The intent is comparison. Once this design exists, it can be compared against
the implementation in `s57_grids` / `s57_layer` / `marine_charts` to surface
shared assumptions, divergent choices, and places where either design is
stronger.

Tracked by [#23](https://github.com/rolker/s57_tools/issues/23).

## Reading guide

The document is structured chronologically by the conversation that produced
it, lightly edited for flow:

1. [Inputs and constraints](#1-inputs-and-constraints) — the framing pass,
   before any architecture discussion.
2. [Resolution and tiering](#2-resolution-and-tiering) — what resolution is
   actually needed, and why nav2's standard global+local pattern fits.
3. [Four prep questions](#3-four-prep-questions) — design choices that needed
   to be settled before the architecture made sense.
4. [The multi-leg mission scenario](#4-the-multi-leg-mission-scenario) — a
   late-breaking case (regional missions covering hundreds of km) and how it
   shaped the design.
5. [Architecture](#5-architecture) — the actual pipeline, layer breakdown,
   composition rule, modules, and update model.
6. [v1 scope and deferrals](#6-v1-scope-and-deferrals) — what should ship,
   and what is explicitly deferred.
7. [Where confidence is low](#7-where-confidence-is-low) — places where the
   answer is a best guess rather than a settled decision.

## 1. Inputs and constraints

### Inputs

**Chart data**

- S-57 ENC cells (`.000` base + `.00N` updates), or a pre-converted form
  (GeoPackage, OGR-readable). Per cell: header metadata (scale, datum,
  edition), and vector features with geometry (point/line/area) plus IHO-coded
  attributes.
- The feature classes that actually drive a marine costmap are a small subset
  of S-57: `LNDARE`, `COALNE`, `DEPARE`, `DRGARE`, `DEPCNT`, `SOUNDG`,
  `UWTROC`, `OBSTRN`, `WRECKS`, `RESARE`/`PRCARE`, `BRIDGE`, `CBLSUB`/`PIPSOL`,
  `FAIRWY`, `TSSLPT`/`TSEZNE`, plus aids (`BOYxxx`, `BCNxxx`, `LIGHTS`) — the
  latter mainly as keep-clear points, not as obstacles.
- Multiple cells of differing scale (`CSCL`) usually overlap; the "best"
  feature at a point is the one from the largest-scale cell that contains it.

**Vehicle / mission state** (these turn raw chart features into costs)

- Static vessel params: draft, beam, length, air-draft, turning radius,
  safety margin.
- Dynamic state: current tidal offset relative to chart datum (LAT/MLLW),
  and the planning horizon (now vs. when the boat would actually be there).
- Optionally: vessel class / rules profile (commercial vs. small craft —
  changes how `RESARE` / TSS are treated).

**Map / runtime config**

- Target frame (typically a local ENU/UTM map frame), origin geodetic anchor,
  grid resolution and extent, update rate.
- Cost policy: which feature → which cost, inflation radii, hard-vs-soft
  thresholds, unknown-cell policy.

### Constraints

**Semantic / correctness**

- Depth is *relative*: a `DEPARE` with `DRVAL1 = 2 m` is lethal at 3 m draft
  and benign at 1 m. Cost must be derived from
  `(depth − tide) − draft − safety_margin`, not baked in.
- Feature priority is non-trivial: a `WRECKS` point lies *inside* a `DEPARE`
  polygon; the wreck must win locally. Rasterization order alone is fragile —
  an explicit priority/merge rule is needed.
- Polygons have holes (an island inside a depth area, a depth area inside
  land reclamation). The rasterizer has to honor them or land becomes
  navigable.
- Geographic → planar projection introduces distortion; for a few-km map it's
  negligible, for a large operating area it isn't.

**Operational**

- ENC cells are large; pre-rasterizing every cell at high resolution is
  wasteful. The robot only needs the local window, but tide updates
  invalidate any cached depth-derived costs globally.
- Updates: tide changes are slow (minutes), vessel params change rarely, the
  chart itself is effectively static per mission. That asymmetry suggests
  separating "what's there" (cacheable, vessel-independent) from "what it
  costs me" (recomputed when tide/draft change).
- nav2 expects a `nav_msgs/OccupancyGrid` or layered `Costmap2D`. Layered is
  the natural fit because each S-57 concern (land, depth, point obstacles,
  regulatory) maps cleanly to one layer with its own update rule.
- Provenance / debuggability: when a planner refuses to cross a cell, the
  operator needs to know *which feature* did it. That's a real constraint on
  the pipeline, not a nice-to-have.

**Edge cases worth naming up front**

- Cell boundaries (no double-cost in overlap zones, no gaps).
- Unsurveyed areas (`UNSARE`) — neither free nor blocked; "unknown" is a real
  value, not a failure mode.
- Soundings (`SOUNDG`) are point depths, often denser than `DEPARE` resolution
  — do we use them, or trust the polygons?
- Time-varying features (lights, seasonal buoys) — probably out of scope, but
  worth deciding explicitly.

### Scope decisions made up front

- **Scope**: just the static-environment costmap layer that nav2's local /
  global planner consumes. Dynamic AIS / radar tracks are a separate concern.
- **Tide handling**: the boat publishes a scalar tide-offset estimate at its
  current location. For short-term planning this is fine; spatially varying
  tide is deferred.

## 2. Resolution and tiering

The driving constraint on resolution is the UNH pier: piling diameter ~0.3–0.5 m,
gaps between pilings on the order of 4–8 m, and a BizzyBoat-class hull at ~2 m
beam wanting maybe 1–2 m of clearance per side.

The rasterization rule of thumb: a point obstacle occupies at minimum 1 cell,
and once it straddles a cell boundary, 2 cells (so its effective width is
`r` to `2r`). Then inflation is added on top.

| Resolution | Piling footprint | + 1.5 m inflation each side | Usable in a 6 m gap |
|------------|------------------|----------------------------|---------------------|
| 1.0 m      | 1–2 m            | ~5 m total                 | ~1 m — too tight    |
| 0.5 m      | 0.5–1 m          | ~3.5–4 m total             | ~2 m — workable     |
| 0.25 m     | 0.25–0.5 m       | ~3.25 m                    | comfortable         |

**Decision**: 0.5 m for the close-quarters layer. 0.25 m would be safer but
quadruples memory and rasterization cost for a marginal gain on a 2 m hull.

That choice forces the **coverage** question, because 0.5 m everywhere does not
scale: Portsmouth Harbor + Great Bay (~5 km × 5 km) at 0.5 m is 100 M cells per
layer, ~100 MB just for one layer.

### Coverage options considered

**(a) Single full-area static map** at one resolution.
Simple, cacheable, but pays full-area cost at whatever the worst-case
resolution is. At 0.5 m over a real operating area this hurts; at 2 m the pier
doesn't work.

**(b) Two-tier: coarse global + fine local window.**
Global costmap at ~2–4 m covers the operating area for route planning. Local
costmap at 0.5 m covers a window (e.g., 200 m × 200 m, 160 k cells) sliding
with the vessel for obstacle-aware control. This matches nav2's existing
pattern — global and local planners read separate costmaps.

**(c) Single fine-resolution sliding window only.**
Rules itself out for survey work because the global planner needs a route
across the whole operating area.

**Decision**: (b). Matches nav2 architecture, keeps memory bounded, same cost
model in both — just rasterized at different resolutions from the same source.

A consequence flagged early: a single boat-position tide estimate is fine for
the local window (within a few hundred metres of the boat) but applies that
same scalar to the far corner of the global costmap. For *route planning* (not
control) this is acceptable — the route gets re-evaluated as the boat moves
and tide updates — but the global map's depth layer is "approximate, refreshed
continuously" rather than "accurate everywhere right now."

## 3. Four prep questions

Before sketching architecture, four questions had to be settled. Each is
recorded with the options considered, the decision, and the reasoning.

### 3.1 Soundings (`SOUNDG`) versus depth areas (`DEPARE`)

**The semantic asymmetry that drives the question:**

- `DEPARE` is a polygon with `DRVAL1` (shoalest depth in the area) and `DRVAL2`
  (deepest). A cartographer drew it by tracing depth contours and applying
  generalization — it's a *judgment* about what the seafloor looks like in
  that band.
- `SOUNDG` is a point with a discrete depth from the underlying hydrographic
  survey. The chart shows a curated subset, not all sampled depths. Compilers
  deliberately keep shoal soundings visible even when they fall inside a
  deeper-band polygon, as warnings.
- A `SOUNDG` of 1.5 m sitting inside a "2–5 m" `DEPARE` is **not a contradiction
  or a chart error** — it's a flag saying "this area is mostly 2–5 m but
  there's a shallow spot here, watch out."

You cannot just pick one.

**Options:**

**(a) DEPARE only.** Clean coverage but loses every shoal-spot warning the
cartographer encoded as a sounding. The boat would happily drive over a 1.5 m
least-depth point sitting inside a "2–5 m" polygon.

**(b) DEPARE as base, SOUNDG as override-when-shoaler.** Rasterize DEPARE for
full coverage. For each SOUNDG, look up the cell's current depth — if the
sounding is shoaler, replace it (and apply a small inflation, since the
point's actual position has uncertainty). SOUNDG only ever makes things
shoaler, never deeper.

**(c) Interpolate SOUNDG into a continuous depth raster, drop DEPARE.** This
is what hydrographic gridding software does (CUBE, IDW, splines). Wrong tool:
SOUNDG density on an ENC is uneven and *biased* — the cartographer picked
points for navigational prominence, not statistical sampling. Interpolating
between two soundings 200 m apart gives a smooth ramp that may have nothing to
do with the actual bottom (could be a steep bank). And it throws away the
cartographer's expert generalization.

**Decision: (b).** DEPARE is the authoritative "what does the chart claim
here" because the cartographer's generalization is signal, not noise. SOUNDG
is the authoritative "warning point" mechanism.

**Mechanical points that fall out:**

- For DEPARE cost, use `DRVAL1` (shoalest), not the average. We are a
  navigator, not a statistician.
- DEPCNT (depth contour) lines are redundant with DEPARE if the data is
  consistent — the polygon's `DRVAL1` equals the shoaler bounding contour. We
  can ignore DEPCNT and just trust the polygons.
- When two SOUNDGs land in the same cell at high resolution, take the shoaler
  — same rule.
- The "small inflation" around a SOUNDG matters: positional uncertainty
  (`QUAPOS`) on a sounding can be metres. A SOUNDG point with no inflation
  under-represents what it's warning about. Half a cell to one cell feels
  right at 0.5 m resolution; this should be configurable.

### 3.2 UNSARE (unsurveyed area) handling

**What UNSARE is**: a polygon marking water that hasn't been hydrographically
surveyed (or not to modern standards). Important distinction: it's still
**water**, not land. The boat physically can be there — we just don't know
what's under it.

**Options:**

1. **Lethal (cost 254).** "Assume the worst." Conservative, but claims a
   certainty we don't have — there's no obstacle there, we just don't know.
2. **Unknown (cost 255 / `NO_INFORMATION`).** Honest. nav2's default planner
   behavior treats unknown as non-traversable (`allow_unknown=false`), so
   practically the boat still won't enter, but the layered costmap preserves
   the "absence of data" semantic for downstream consumers.
3. **Free space.** No.
4. **Per-mission configurable.** Operator chooses 1 or 2 at mission start.

**The wrinkle: this is a survey boat.** The end goal is autonomous surveys —
UNSARE is exactly the kind of place a survey boat might be tasked to go, to
fill it in. That changes the framing.

**The clean separation: the costmap reports facts, the planner makes the policy.**

- Costmap layer: UNSARE → 255 (unknown). Always. The truthful representation.
- Planner config: `allow_unknown` is mission-policy. Operational transit →
  false (boat refuses to enter). Survey mission targeting an UNSARE polygon →
  true (boat may enter, with the implicit assumption that downstream sensors
  will clear ahead).

This keeps the costmap purely descriptive and pushes the risk decision to the
layer that should own it. It also avoids a category error where the
*cost mapping* becomes mission config instead of just the planner.

**Decision: option 2**, with `allow_unknown` exposed as planner config rather
than baked into the costmap.

**Related cells, same rule:**

- Outside any loaded ENC cell: 255. No data is no data.
- Inside the operating area but no DEPARE / UNSARE / LNDARE covers it: 255.
  Treat absence of feature as absence of information, not as free water.

**Explicitly not done in v1**: ZOC (Zone of Confidence). S-57 charts encode
survey quality via `M_QUAL` polygons with `CATZOC` values (A1/A2/B/C/D/U). We
could go further and use ZOC to *scale our confidence* in DEPARE depths —
e.g., add a depth uncertainty to `DRVAL1` based on ZOC class. That's a real
improvement but it's gold-plating for v1. Flagged as a future layer.

### 3.3 Regulatory features

**The taxonomy**: "regulatory" lumps together genuinely different things.

| Feature             | What it actually is                                      | Costmap treatment                       |
|---------------------|----------------------------------------------------------|----------------------------------------|
| `RESARE`            | Could be *anything* — depends on `CATREA` / `RESTRN`     | Depends entirely on attributes         |
| `FAIRWY`            | Designated/dredged channel for big ships                 | Soft — avoid as small craft            |
| `TSSLPT`/`TSEZNE`   | Traffic separation scheme lanes/zones                    | Soft — stay out, ideally direction-aware |
| `ACHARE`            | Anchorage area                                           | Soft — boats may be anchored here      |
| `MARCUL`            | Aquaculture (oyster farms, mussel rafts)                 | Should be near-lethal — physical       |
| `CTNARE`            | Caution area                                             | Informational, no costmap effect       |
| `CBLSUB`/`PIPSOL`   | Submarine cables / pipelines                             | Lethal for *anchoring*, irrelevant for surface |
| `DMPGRD`            | Dumping ground                                           | Usually historical, low cost           |
| `MIPARE`            | Military practice area                                   | Time-varying; usually soft, sometimes lethal |

**The big trap**: `RESARE` is the most regulatory-looking class and the
**most attribute-dependent**. The same feature class covers:

- `RESTRN=7` entry prohibited → genuinely lethal (security zones, minefields)
- `RESTRN=1` anchoring prohibited → completely irrelevant to a
  surface-navigating costmap
- `CATREA=2` nature reserve → soft preference
- `CATREA=12` no-wake area → speed limit, not a costmap concern at all
- `CATREA=15` environmentally sensitive → soft preference

A v1 that treats `RESARE → lethal` will brick the entire harbor on day one.
The attributes are not optional — they're load-bearing.

**Scope options:**

**(a) Hard exclusions only.**
- `RESARE` where `RESTRN ∈ {7, 14}` (entry prohibited / contact prohibited) → lethal.
- `MARCUL` → lethal (or near-lethal — it's a physical obstruction more than a regulation).
- Everything else regulatory → ignored.

Defensible v1, very small spec, almost certainly correct as far as it goes.

**(b) Hard exclusions + soft preferences.**
- All of (a), plus:
- `RESARE` with environmental / nature attributes → high soft cost (~200).
- `TSSLPT` / `TSEZNE` → high soft cost. Direction-of-travel ignored for v1.
- `FAIRWY` → small soft cost (~50) for a small autonomous craft, since we
  mostly want to *avoid* big-ship lanes when there's an alternative.
  Optionally zero.
- `ACHARE` → small soft cost; might have anchored vessels.

More useful, but commits to a cost-policy table that needs maintenance and
operator review.

**(c) Full regulatory awareness.** TSS direction enforcement, vessel-class
profiles, time-varying military zones, seasonal rules. Out of scope for v1.

**Decision: (b), structured as a config-driven mapping table.**

**Two architectural commitments fall out:**

1. **Regulatory should be a separate costmap layer (or set of layers) from the
   depth layer.** They have totally different update properties (depth
   changes with tide, regulatory effectively never) and different failure
   modes (a depth bug is a safety issue, a fairway-cost bug is a quality
   issue). Mixing them couples invariants that shouldn't be coupled.
2. **No hardcoded class → cost mappings.** Ship a defaults YAML
   (`RESARE+RESTRN → cost`, `FAIRWY → cost`, etc.) that the operator can
   review and override. The mapping is policy, not code.

**Subtleties:**

- **RESARE polygons can be huge** (a no-anchoring zone may cover an entire
  harbor). Even at "soft" cost, painting a large area at cost 200 will distort
  planning over miles. The cost magnitude needs to scale with the *kind* of
  restriction, not the polygon size.
- **MARCUL is misclassified as "regulatory."** Aquaculture infrastructure —
  buoys, longlines, rafts, oyster cages — is physical and real. It belongs in
  a "static obstacle" layer with WRECKS and OBSTRN, not regulatory. But it's
  encoded as an area with regulatory-ish framing in S-57, so it's flagged
  here.
- **FAIRWY soft-cost direction is debatable.** Some operators want the boat
  to *prefer* fairways (predictable bottom, marked, well-charted). Others
  want it to *avoid* them (commercial traffic). The right answer depends on
  operating posture; lean avoid for an autonomous small craft, but it's a
  config knob, not a hardcoded rule.

### 3.4 Operating-area definition

**Options considered:**

**(a) Per-mission config.** Operator sets a bounding box at mission start.

**(b) Union of loaded ENC cells.** Killed by scale: a single Maine coastal
cell can cover hundreds of square km. At 2 m global resolution that's
billions of cells.

**(c) Auto-fit to waypoints + margin.** Compute extent from start pose +
waypoints + return-home + some margin.

**(d) Deployment-level operating area.** Operator defines an area once at
deployment ("Great Bay", "Portsmouth Harbor near Judd Gregg pier"); all
subsequent missions for that deployment use it. Persists across missions.

**Decision: (d) primary, (c) as fallback when (d) isn't set.**

The operating area is genuinely a *deployment* concept, not a *mission*
concept. The boat goes to a location, sits there for a deployment, runs many
missions. Re-defining the costmap extent before every mission would be
ceremony with no information content.

(c) is the right fallback for the case where someone fires up the boat
without setting anything. Just don't make it the primary mechanism, because
mid-mission replans should not silently resize the costmap.

(a) is what you get when the operator explicitly overrides for a one-off —
keep the path open, don't make it the default.

**Architectural consequence**: this unifies cleanly with the UNSARE decision
— **outside the operating area = unknown (255)**, same as outside-loaded-ENC
and feature-gaps. The operating area is just "where the global costmap exists
at all." Beyond it, the planner sees out-of-bounds and refuses to plan there,
which is the safe behavior.

That gives one concept driving two things:
- Global costmap rasterization extent.
- Which ENC cells actually need to be loaded and rasterized.

**Memory / resolution interaction:**

| Area                          | At 2 m  | At 4 m | At 8 m |
|-------------------------------|---------|--------|--------|
| 2 km × 2 km (small bay)       | 1 M     | tiny   | tiny   |
| 5 km × 5 km (Great Bay)       | 6 M, ~6 MB  | 1.5 M  | 0.4 M  |
| 10 km × 10 km (Portsmouth + approaches) | 25 M, ~25 MB | 6 M | 1.5 M |
| 50 km × 50 km (regional transit) | 625 M — no | 156 M | 39 M |

Global resolution is a config knob with a sane default for the typical
operating-area size, not a hardcoded constant.

**Edge cases settled:**

- Boat leaves operating area: planner refuses to plan into out-of-bounds.
  Ops gets a clear failure ("destination outside operating area") rather
  than weird routing through unknown space.
- Operating area straddles ENC cell boundaries: feature-gap cells inside the
  area are 255. Same rule as the rest.

## 4. The multi-leg mission scenario

A scenario surfaced after the four prep questions and re-shaped the operating-area
discussion: a larger boat departs Portsmouth, NH, transits to a survey area near
Cape Cod, stops at Provincetown, MA for refuel, returns to survey, then returns
home to Portsmouth. Total mission scope: ~200 km × 100 km bounding box.

### The size problem

| Resolution | Cells | Memory      |
|-----------|-------|-------------|
| 2 m       | 5 G   | impossible  |
| 4 m       | 1.25 G | impossible |
| 8 m       | 312 M | very rough  |
| 16 m      | 78 M  | ~78 MB, feasible |
| 32 m      | 20 M  | ~20 MB, comfortable |

A single global costmap covering the whole mission is feasible, but only at
16–32 m. At that resolution coastlines and major shoals resolve fine, but a
piling cannot. The instinctive reaction is "we need multi-resolution /
multi-zone." That instinct deserves stress-testing.

### What is the global costmap actually for?

The nav2 global planner's job is *route planning* — find a safe path from
where the boat is now to where it needs to go, avoiding land and major
hazards. For a 175 km transit, it has to know about the New Hampshire and
Massachusetts coastlines and shoals like Stellwagen Bank, but it does not
need to know about pilings at the Provincetown harbor entrance — the boat
won't be near them until it arrives.

What about the *survey area*? Does nav2's global planner need fine resolution
there to plan the survey pattern? **No** — survey patterns (lawnmower lines)
are generated by a *mission planner*, not by nav2's grid planner. The
autonomy executes pre-computed lines and uses the local costmap to avoid
trouble. The global costmap's job is only to route the boat *to* the survey
area, not to plan the survey itself.

What about the *harbor approach* (Portsmouth pier, Provincetown harbor)? The
local 0.5 m sliding window is exactly the right tool. It moves with the
boat. By the time the boat is approaching the pier, the local costmap has
the pier at 0.5 m. The global costmap's 32 m is fine for "go through the
harbor entrance roughly here."

### Two architectures considered

**(α) Single global, mission-scoped resolution and extent.**
- Harbor mission: global at 2 m, 5 km × 5 km.
- Regional mission with transits: global at 16–32 m, 250 km × 150 km.
- Local always at 0.5 m, sliding 200 m × 200 m.
- Stock nav2 global+local pattern. One operating area concept, just larger
  and coarser when needed.
- Limitation: no fine-resolution awareness anywhere outside the local window.

**(β) Multi-zone: union of operational areas + transit corridors.**
- Operational zones (Portsmouth, Cape Cod survey area, Provincetown) at fine
  global resolution.
- Transit corridors between them at coarse resolution (or even just
  route-graph waypoints, not a grid at all).
- Boat "knows what mode it's in" — operational zone vs. transit — and the
  active global costmap reflects that.
- Cost: substantially more architecture. Zone transitions, costmap loading /
  unloading, potentially custom planner logic. Not stock nav2.

**Decision: (α) for v1.** The role argument convinces: nav2's global
costmap is for *route planning*, not survey-pattern execution. The gap (β)
closes is mostly hypothetical for how marine autonomy actually plans.

**(β) becomes necessary if/when:**

- Grid-based planning is wanted over a survey area at sub-metre resolution
  *simultaneously* with grid-based long-range transit.
- Missions cover geography too large for any single resolution to span.

Both are genuine future possibilities for the larger boats, but neither is a
v1 requirement.

**Implementation note**: lazy / tile-based rasterization becomes important
at 200 km × 100 km. The global costmap should be tiled internally, with
tiles invalidated and rebuilt as needed — but that's an implementation
detail under the (α) architecture, not a separate architecture.

## 5. Architecture

### 5.1 Pipeline overview

```
ENC files
   │
   ▼
[Parse S-57] ──► vector features (geom + attrs) per cell
   │
   ▼
[Spatial index] ──► R-tree keyed on map-frame coords
   │
   ▼
[Layer rasterizers] ──► one grid per logical layer
   │
   ▼
[Composer] ──► final 2D costmap
   │
   ▼
nav2 (Costmap2D / OccupancyGrid)
```

Single ENC ingest, multi-layer rasterization, max-cost composition. The
interesting design choices live in the layer breakdown and the update strategy.

### 5.2 Layer breakdown

Five logical layers, organized by **update trigger** rather than by feature
type — that's the cut that actually matters operationally.

| Layer             | Sources                                                  | Cost behavior        | Update trigger                       |
|-------------------|----------------------------------------------------------|----------------------|--------------------------------------|
| **Land**          | `LNDARE`, closed `COALNE`, `SLCONS`                      | Lethal               | Chart load (rare)                    |
| **Hard regulatory** | `RESARE` with `RESTRN ∈ {entry / contact prohibited}`, `MARCUL`, surface-piercing `OBSTRN` / `UWTROC` / `WRECKS` | Lethal | Chart load |
| **Soft regulatory** | `RESARE` (env / nature / fish), `FAIRWY`, `TSSLPT` / `TSEZNE`, `ACHARE` | Soft (config-driven) | Chart load + vessel-class change |
| **Coverage**      | `UNSARE`, area outside any feature polygon, area outside loaded ENC cells | 255 unknown | Chart load |
| **Depth**         | `DEPARE` (`DRVAL1`) + `SOUNDG` override + tide/draft-conditional `UWTROC` / `WRECKS` / `OBSTRN` | Continuous (depth → cost function) | Chart load + tide + draft |

The cut: 4 of the 5 layers are static after chart load. Only **Depth** needs
continuous refresh. That asymmetry drives the implementation — static layers
can be pre-rasterized and held; the depth layer is the only one with
hot-path complexity.

Each layer is a `nav2_costmap_2d::Layer` plugin. Layered costmaps are
exactly what nav2's plugin architecture is for; reusing it avoids reinventing
composition, transforms, and ROS-side plumbing.

### 5.3 Composition rule

Cell-by-cell, applied in this order:

1. Start every cell at 255 (unknown).
2. Coverage layer keeps unsurveyed / out-of-data cells at 255.
3. Land, Hard Regulatory, Soft Regulatory, Depth each report a cost (or
   skip the cell).
4. **Combination**: any real cost (0–254) overrides 255. Among real costs,
   **max wins**.

The "max wins" rule is the right default because every layer is a
hazard-or-preference cost. Notes:

- A SOUNDG override inside a DEPARE polygon is handled *inside* the Depth
  layer, not by composition — same reasoning as 3.1.
- Deliberately not using nav2's `addExtra` semantics; max-only is cleaner
  and the regulatory-vs-physical priorities work out.

### 5.4 The Depth layer in detail

It's the only complex layer.

**Inputs:**

- `DEPARE` polygons with `DRVAL1` (shoalest depth in band).
- `SOUNDG` points with depth values.
- Conditional point obstacles (`UWTROC`, `WRECKS`, `OBSTRN`) where the
  attribute `VALSOU` (or feature flag for "always dangerous") indicates
  depth-conditional shoaling.
- Tide offset (scalar).
- Vessel draft + safety margin.

**Per cell:**

```
charted_depth   = DEPARE.DRVAL1 at this cell
                  (or SOUNDG depth if shoaler than DEPARE within search radius)
effective_depth = charted_depth - tide_offset
clearance       = effective_depth - draft - safety_margin
```

Then a **clearance → cost function** maps clearance to 0–254:

- `clearance < 0`              → lethal (254)
- `0 ≤ clearance < margin_warn` → high cost, gradient
- `clearance ≥ margin_safe`     → free (0)
- Linear or smoothstep between thresholds; configurable.

This function is *config*, not code. Different vessels / missions tolerate
different clearance margins.

For depth-conditional point obstacles: same clearance computation, but
stamped at the point with a small inflation (1 cell at 0.5 m local; possibly
larger at coarse global to avoid the point being lost to rasterization).

**Update model**: when tide changes, recompute the entire depth layer. At
16 m / 80 M cells, this is a few hundred ms — fast enough for tide-update
rates (minutes). At 0.5 m / 160 k cells (local window), trivially fast.

**Optimization (deferred)**: cache the *clearance* values rather than the
cost. A tide update then just adds an offset to clearance and re-runs the
cost function. Faster than re-rasterizing from vectors. v2 work.

### 5.5 Static-layer rasterization

Land, Hard Regulatory, Soft Regulatory, Coverage are computed once when the
chart loads, into raster form, and held.

For the **global costmap** at 16–32 m / mission-scoped extent: full rasters
fit easily in memory. Compute and hold them.

For the **local 0.5 m sliding window**: full pre-rasterization at 0.5 m over
the operating area is impossible. Two options:

- **Tile-based**: pre-rasterize fixed tiles (e.g., 200 m × 200 m at 0.5 m →
  160 k cells per tile, ~160 KB) on demand. Cache recently-used tiles. Drop
  tiles far from the boat.
- **Vector-on-demand**: at each local-costmap update, query the spatial
  index for features inside the window, rasterize directly into the local
  grid.

**Decision**: start with vector-on-demand for the local window — the window
is tiny (~160 k cells), the spatial index makes the feature query cheap, and
the result is always fresh. Tile caching is an optimization if profiling
shows it's needed.

### 5.6 Module breakdown

```
enc_chart_loader        Library — parses S-57, produces feature stream.
                        Uses GDAL/OGR (mature S-57 reader).

enc_feature_index       Library — R-tree index over features, in map-frame coords.
                        Projection happens at load.

enc_costmap_layers      ROS package — five nav2 costmap layer plugins.
                        Each plugin holds a reference to the shared feature index
                        and queries it during update.

enc_costmap_config      YAML schemas for:
                          - operating area polygon
                          - mission profile (resolution, extent)
                          - vessel params
                          - cost-mapping table (feature class + attrs → cost)
                          - inflation per layer

enc_costmap_diagnostics ROS topics for per-layer grids + a "provenance" overlay
                        showing which feature dominated each cell.
```

Plugin model means nav2's existing `costmap_2d` node loads and runs everything
— no separate node tree, no separate publisher logic.

### 5.7 Coordinate handling

- ENC in WGS84 (geographic).
- Map frame: UTM, zone defaults to operating-area centroid's zone, configurable.
- Projection at chart load time, into the spatial index. Re-projection per
  query is wasteful and not needed.
- Multi-zone operating areas (e.g., very large transit crossing UTM zone
  boundaries): out of scope for v1, flagged for future. Portsmouth–Cape Cod
  fits in Zone 19.
- Local ENU origin: not needed — UTM with a sensible local origin offset
  works, and TF chains from `map` are unaffected.

### 5.8 Update model summary

| Trigger                              | What recomputes                                     |
|--------------------------------------|----------------------------------------------------|
| Chart load                           | All five layers, full extent                        |
| Tide change                          | Depth layer only, full extent (cached-clearance fast path deferred) |
| Draft / safety-margin change         | Depth layer only                                    |
| Vessel-class change                  | Soft Regulatory only                                |
| Boat moves                           | Local window slides, vector-on-demand re-rasterization for local |
| Operating area change                | Reload (treat as new mission)                       |

## 6. v1 scope and deferrals

### Ship in v1

- The five layers above, with the feature classes listed.
- DEPARE + SOUNDG (option b) depth handling.
- UNSARE → 255.
- Hard exclusions + soft preferences regulatory, config-driven attribute
  mapping.
- Single-area mission-scoped global, 0.5 m local sliding window.
- Tide via scalar topic.
- nav2 layer plugin integration.
- Per-layer debug topics + provenance overlay.

### Explicitly deferred (named, not ignored)

- ZOC-based depth uncertainty.
- Multi-zone operating area (β architecture).
- TSS direction-of-travel awareness.
- Vessel-class profiles beyond a single config.
- Cached-clearance fast path for tide updates.
- Multi-UTM-zone operating areas.
- Spatially varying tide.
- Time-varying restrictions (military activity periods, seasonal zones).

## 7. Where confidence is low

A few places where the answer is a best guess rather than a settled decision:

- **Whether to treat `OBSTRN` as Hard Regulatory or Depth-conditional.** S-57
  splits obstructions into surface-piercing and submerged via attributes
  (`VALSOU`, `WATLEV`); the mapping above is the "right" interpretation, but
  `OBSTRN` in real charts is messy and the attribute coverage is
  inconsistent. Worth empirical inspection of actual charts before
  committing.
- **Whether MARCUL belongs in "Hard Regulatory" or a new "Physical Static
  Obstacle" layer.** Argued for the former, but it's a physical thing, not
  a regulation. Either is defensible; the cost is the same; the layer name
  is cosmetic.
- **The clearance-to-cost function shape.** Linear vs. smoothstep vs.
  discrete bands — all defensible, none obviously right. Worth deciding by
  what the operator's mental model is, not by mathematical elegance.
- **Vector-on-demand for local vs. tile cache.** Committed to
  vector-on-demand; tile cache could be better if the spatial index queries
  turn out slower than expected. Worth profiling early.
- **Whether SOUNDG inflation should be a fixed radius or read from `QUAPOS` /
  `QUASOU` attributes when present.** Probably start with fixed and make it
  data-driven later if `QUAPOS` is reliably populated in the charts that
  actually matter.
- **Whether SOUNDGs *deeper* than their containing DEPARE should be ignored
  entirely (the recommendation here) or allowed to make a "1 m" band locally
  read as 1.5 m.** Recommended ignore — the navigationally conservative
  direction is shoaler-only.

---

**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.7 (1M context)`
