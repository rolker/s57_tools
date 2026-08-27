# Prior Art: Reed 2018 — "Providing Nautical Chart Awareness to Autonomous Surface Vehicles"

A neutral, descriptive summary of Samuel Reed's 2018 MS thesis at UNH. The
work in this repository (`s57_tools`) is the spiritual successor to Reed's
research — it inherits the goal of giving autonomous surface vessels an
ENC-derived sense of the marine environment, and many of the same problems
(depth-area generalization, qualitative-vs-quantitative depth attributes,
tide handling, cartographic scale) recur here in different form.

This document captures Reed's approach for reference. It does not contrast
that approach with the current implementation or with proposals in this
repo — those comparisons live separately.

## Citation

> Reed, Samuel John. *Providing Nautical Chart Awareness to Autonomous
> Surface Vehicles*. Master's Thesis, University of New Hampshire,
> Department of Electrical and Computer Engineering, December 2018.
> Thesis Director: Dr. Brian Calder (CCOM Associate Director).
> Available at https://scholars.unh.edu/thesis/1256/

Funded by NOAA Grant NA15NOS4000200. Co-advised by Val Schmidt (CCOM Research
Project Engineer) along with Dr. Se Young Yoon and Dr. Kent Chamberlin (UNH ECE).

## Goal

Move ASV autonomy from ALFUS Level 3 (executes pre-planned missions, no
environmental awareness) toward Level 4–5 (knowledge of local and planned
path environment; hazard avoidance) using only Electronic Nautical Charts —
no real-time bathymetric sensors required for the autonomy itself.

The thesis argues ASVs need *both* an a-priori intelligent mission planner
*and* a real-time reactive obstacle avoidance system, both informed by the
chart, because:

> Vehicles cannot unconditionally follow a pre-programmed list of waypoints
> nor react to other vessels in real time without regard to these hazards.

## Stack and architecture

Built on **MOOS-IvP**, not ROS. Specifically:

- **MOOS** (Mission Oriented Operating System): publish-subscribe middleware
  with a centralized `MOOSDB` for inter-application communication.
- **IvP Helm** (`pHelmIvP`): combines behavior modules into ship-driving
  decisions. Each behavior produces an **IvP Function** — a polar utility
  surface where azimuthal angle = heading, radial distance = speed, and
  height = utility. The `IvP_Solver` finds the optimal (heading, speed) pair
  by combining behaviors' surfaces.

Reed adds four custom MOOS components alongside the pre-existing waypoint
behavior, `pMarineViewer`, `uSimMarine`, and `pMarinePID`:

| Component       | Role |
|-----------------|------|
| **`MOOSTides`** | Real-time tide prediction from NOAA harmonic constituents (uses Sam Cox's `pytides`). Single tide value per region. |
| **`ENC_Contact`** | Pre-processes ENC into a queryable spatial database (`ENC_DB`); monitors vessel position; publishes per-heading utility values. |
| **`ENC_DB`**    | Single shapefile of polygons with threat-level and buffered geometry, derived from ENC layers. |
| **`ENC_WPT_Check`** | Validates user-provided waypoints against the chart; skips unreachable or unsafe waypoints mid-mission. |
| **`BHV_ENC_OA`** | IvP behavior subscribing to `ENC_Contact`'s utility report; produces an avoidance heading utility surface combined with the waypoint behavior in the IvP Solver. |

Below the autonomy stack, the planner (offline, separately) generates the
mission's waypoints from the ENC.

## Two distinct ENC representations

Reed uses two derivations of the same ENC for two purposes:

1. **Interpolated depth grid** — used by the offline mission planner.
   Single-resolution rectilinear grid; resolution = 0.25 mm × ENC compilation
   scale (so a 1:20,000 chart → 5 m grid). Built once per chart.
2. **`ENC_DB`** — used by the real-time reactive OA. Polygons-only spatial
   database, threat-level annotated, ASV-buffered. Built once per chart, queried
   in a sliding 20·L × 20·L window around the vessel (where L is vessel length).

The two representations are independent and serve different consumers.

---

## Part 1 — ENC Derived Mission Planner

### Building the depth grid (Section 2.1.2)

The planner's grid is built once per chart, offline:

1. **Project**: ENC features WGS84 → UTM via GDAL.
2. **Buffer obstacles**: every obstacle polygon expanded by
   `B = (SM/2) · √(L² + W²)` where `SM` = safety margin (default 2),
   `L` = vessel length, `W` = vessel width. After this, the ASV can be
   treated as a point during planning.
3. **Buffer point features**: chart points (rocks, buoys, beacons, wrecks)
   buffered by 2 mm at chart scale (a chart point isn't a real point — it's
   an icon with positional uncertainty).
4. **Linear interpolation** of buffered polygon vertices to grid cells using GDAL.
5. **Conflict resolution** via four raster masks:
   - `M_COVR` — outer chart bounds.
   - `DA_shoal` — depth-area minimum bounds.
   - `DA_deep` — depth-area maximum bounds.
   - `Points` raster — point obstacles (rocks, wrecks, buoys).
   - `Polygons` raster — land, pontoons, shoreline construction, etc.
6. **Algorithm 1: Water Level → Depth.** Converts qualitative `WATLEV` codes
   to quantitative depths, scaled per ocean (Atlantic vs. Pacific):

   ```
   if Type == Land:                  depth = -(MHW + 2)
   else if WATLEV == 2 (always dry): depth = -(MHW + 0.3048 · OceanScalar)
   else if WATLEV == 3 (submerged):  depth =  0.3048 · OceanScalar
   else if WATLEV == 4 (covers/uncovers): depth = -(MHW + 0.3048 · OceanScalar)
   else if WATLEV == 5 (awash):      depth = -0.3048 · OceanScalar
   ```

   Atlantic uses scalar 1, Pacific uses scalar 2. MHW is the closest NOAA
   tidal station's MHW value, assumed constant across the ENC. (Reed flags
   this as inadequate for spatially varying tide and large-scale charts.)

7. **Algorithm 2: Depth Resolution.** Per cell:
   - If outside ENC bounds: cell = -10 (sentinel for unknown / no data).
   - Else: clamp to depth-area band, then apply Points and Polygons rasters,
     each only if shoaler than the current value. "Shoaler wins."

The result is an interpolated 2D depth grid in metres below MLLW, with land
encoded as negative depth.

### Depth-Based A* (Section 2.1.3)

Custom A* graph search on the depth grid. Cost function:

```
G(n) = G(n-1) + (DP + w · Cost_depth)

Cost_depth = DP · (15 - AD)    if AD < 15 m
           = 0                  if AD ≥ 15 m
```

Where:
- `DP` = distance from cell `n` to its parent.
- `AD` = average depth between the cells (integrating depth between the
  endpoints, divided by their distance).
- `w` = depth-cost tuning weight. `w = 0.15` was used for the C-Worker 4.

The 15 m cutoff means depth penalties are zero in deep water — the planner
behaves like nominal A* there, minimizing path length.

**Branching factor 8** (as adopted from Yang's FAA*): each node connects to
180 neighbors instead of the 8-Moore-neighbor classic A*. Heuristic is
Euclidean distance to goal. Result: heading changes can be < 2°,
producing shipboard-realistic paths.

**Prohibited cells**: any cell where charted depth is less than `3 × draft`.
Hardcoded multiplier on draft, not a configurable cost-to-clearance function.

The cost function balances "go through the channel like a human mariner"
(high `w` → conservative deep-water paths) against "take the shortest route"
(low `w` → cuts corners through shallow but non-dangerous water). Reed
demonstrates this with five weightings (0, 0.1, 0.15, 0.25, 0.5) on
Portsmouth Harbor missions.

### Mission planner integration

The Depth-Based A* outputs waypoints. These feed the standard MOOS waypoint
behavior, whose IvP Function defines preferred heading + speed. ENC_Contact
adjusts the waypoint behavior's `lead` parameter dynamically (see below) so
the vessel relaxes track-following near hazards.

---

## Part 2 — ENC Derived Obstacle Avoidance

### `ENC_Contact` and `ENC_DB` (Section 2.2.4)

`ENC_Contact` is a custom MOOS app that:

1. Pre-processes ENC: filters out non-obstacle features, reorganizes the
   ~20+ S-57 layers into a single shapefile of polygons.
   ("Significantly more efficient... only needs to open, filter, and react
   to a single layer instead of over 20.")
2. Assigns each obstacle a **threat level** (see below).
3. Buffers each obstacle by:

   ```
   B = (1 + 0.4 · TL) / 2 · √(L² + W²)
   ```

   So buffering scales from 1× the absolute minimum (TL = 0) to 3× (TL = 5).
4. Optionally rasterizes a depth grid to polygonize areas shoaler than a
   user-set minimum MLLW depth, treating those polygons as obstacles. Uses
   only depth-relevant features (soundings, piles, shoreline construction,
   depth areas, land, pontoons, docks) — distinct from the planner's grid.

At runtime, `ENC_Contact`:
- Identifies obstacles within a square search area of side `20·L` centered
  on the ASV.
- Runs an angular-sweep at 8° resolution over 360°.
- Applies a low-pass filter (window 5) to smooth utility transitions at
  polygon edges.
- Publishes a heading-utility report at 5 Hz.

### Threat level (Section 2.2.3)

A new attribute Reed adds to every charted obstacle, range 0–5:

| TL | Description       | pMarineViewer color |
|----|-------------------|---------------------|
| 0  | No threat         | (not shown)         |
| 1  | Small threat      | Yellow-Green        |
| 2  | Medium threat     | Gold                |
| 3  | Large threat      | Orange              |
| 4  | Great threat      | Red                 |
| 5  | Land              | Black               |

Determined by a flowchart (Figure 2.11):

1. **Type-based** (Algorithm 3): Land/Dyke/Pontoon → 5; Shoreline construction
   /Dock → 5; Buoy/Weed/Beacon → 3; TopMark/DayMark → 3.
2. **Depth-based** (Algorithm 4) when `VALSOU` is given:

   ```
   Depth = MLLW_Depth + Current_Tide
   if Depth ≤ 2·draft: TL = 4
   else if Depth ≤ 3·draft: TL = 3
   else if Depth ≤ 4·draft: TL = 2
   else if Depth ≤ 5·draft: TL = 1
   else: TL = 0
   ```

3. **Qualitative depth fallback**: use Algorithm 1 (WATLEV → depth) then
   Algorithm 4.
4. **Unresolvable**: post warning, set TL = 4 (worst-case treatment).

### Distance-vs-utility curves (Section 2.2.5)

For each polygon intersected by an angular-sweep ray, utility is computed:

```
Utility = min(2.5 · 2^D / TL², 100)
```

where `D` = distance to the obstacle in vessel lengths, `TL` = threat level.
Higher utility = safer heading. Multiple polygons → keep minimum
(most-conservative). Lower-threat-level obstacles only penalize headings
when very close; higher-threat obstacles penalize from much further out
(see Figure 2.15).

### Reactive obstacle-avoidance behavior (`BHV_ENC_OA`)

A new IvP behavior subscribing to `ENC_Contact`'s heading utility report:
- Linearly interpolates between 8° increments to produce a smooth
  per-heading IvP utility surface.
- If the ASV is within 3 boat lengths of an obstacle, biases utility toward
  the current heading: `U = PrevU · (1 - |H-Z|/360)` — penalizing radical
  course changes near hazards.
- Combined with the waypoint behavior in the IvP Solver to produce final
  (heading, speed) commands.

### Adjusting the waypoint-behavior lead parameter

When obstacles are near, `ENC_Contact` increases the MOOS waypoint
behavior's `lead` parameter, which controls how aggressively the vessel
returns to the planned trackline:

```
SD = 3·L + 0.5·MTL                     (safety distance threshold)
lead = 8                if (ΔD > 0 and D > SD)    (clear, default aggressive)
     = MTL · 20         otherwise                  (near threat, relaxed)
```

Where `D` = distance to closest obstacle, `ΔD` = its change since last
control iteration, `MTL` = max threat level in the search area. This makes
the ASV deviate from the trackline when threats are nearby and resume
aggressive trackline-following only after passing them.

### Waypoint validity (`ENC_WPT_Check`, Section 2.2.6)

Independent MOOS app that checks user-provided waypoints against the chart.
Skips waypoints that are unreachable (e.g. inside an extended obstacle)
once the ASV gets within a user-defined distance of the obstacle's boundary.
This lets operators plan missions "with little regard to obstacles" and
still complete safely.

---

## Tide handling (`MOOSTides`, Section 2.2.2)

Tide prediction uses NOAA harmonic constituents from a user-specified tide
station, evaluated at the current time using the `pytides` Python library.
Reed publishes the predicted real-time tide and MHW offsets to MOOSDB; both
the threat-level computation and the obstacle-buffer process consume them.

Acknowledged limitations:
- Single-region scalar — no spatial variation across an ENC.
- Doesn't account for meteorological effects.
- Doesn't account for tidal currents.

Reed argues a more sophisticated tidal model "might not be worth the cost"
unless tidal range is large or spatially variable.

---

## Field tests

Two platforms used:
- **Seafloor Systems EchoBoat** — small ASV, 1.68 m × 0.79 m × 0.28 m
  (L × W × draft). Used for in-water field tests at UNH Pier.
- **ASV Global C-Worker 4** — larger commercial ASV, 4.0 m × 1.58 m × 0.4 m.
  Simulation only.

### Mission-planner results (3.1)

Three planned missions on real ENCs:
1. **UNH Pier → Prescott Park** up the Piscataqua River. Path stayed in the
   deep channel for almost the entire mission.
2. **UNH Pier → Isle of Shoals**. Path mostly in the channel but
   occasionally cut through shallower (but non-dangerous) water when that
   was less costly than detouring to the deepest line.
3. **Boston Harbor**. Path stayed near the channel; near the destination,
   it cut a corner through shallower water. Drove ~2.5 m from a lateral
   buoy — Reed flags this as a planner limitation: when point obstacles sit
   inside relatively deep water, the depth-cost function doesn't penalize
   the surrounding cells enough to push the path away. The reactive
   component would mitigate this in execution.

### Reactive OA results (3.2)

**Synthetic ellipse (8 m × 5 m)**, simulation:

| TL | EchoBoat min distance (sim) | C-Worker 4 min distance (sim) |
|----|----------------------------|-------------------------------|
| 1  | 1.8 m                      | 4.4 m                         |
| 2  | 2.9 m                      | 5.8 m                         |
| 3  | 3.4 m                      | 7.3 m                         |
| 4  | 4.0 m                      | 7.9 m                         |
| 5  | 4.2 m                      | 8.7 m                         |

EchoBoat field test, same ellipse: TL=1 → 2.6 m, TL=3 → 4.8 m, TL=5 → 5.3 m.

**Real charted obstacles**:
- UNH Pier breakwater (entering cove) — EchoBoat avoided at 3.0 m.
- UNH Pier breakwater (leaving cove) — EchoBoat squeezed between breakwater
  and shallow water polygon, 1.1 m and 3.6 m closest approaches respectively.
- UNH Pier itself, line-following — avoided at 3.4 m closest approach.

### Combined planner + reactive (3.3)

Boston Harbor mission with both Depth-Based A* and ENC_OA active. ENC_OA
adjusted the planned mission near a buoy to give it more clearance.

---

## Limitations called out by Reed (Section 4)

### 4.1.1 — Depth-Based A* mission planner

Reed's stated limitations:
- **Point obstacles inside deep water can be approached too closely.** When
  a buoy/wreck sits in deep water, the surrounding cells aren't depth-penalized,
  so the depth-cost function offers little disincentive to skim the buffered
  obstacle. (Demonstrated in the Boston Harbor mission.)
- **Single tide value across the chart** — inadequate for large or
  spatially-variable tidal regions.
- **Fixed branching factor / fixed depth threshold** — no adaptation to
  vessel maneuvering or to mission scope.

### 4.1.2 — Reactive obstacle avoidance

- **Local minima.** The most consequential limitation. When the avoidance
  utility function has multiple peaks (e.g. ASV between two obstacles), the
  combined IvP function can oscillate between heading choices, leaving the
  ASV stuck. Demonstrated in a UNH-Pier mission where the EchoBoat
  oscillated and the mission had to be canceled (came no closer than 8.3 m,
  but never made progress). Reed proposes invoking Depth-Based A* as a
  fallback when stuck — left for future work.
- **Behavior-only navigation may be insufficient** for missions requiring
  field-of-view larger than the angular-sweep search area.

### 4.2 — ENC scale and uncertainty

This is the most generally applicable critique in the thesis. Several
specific issues:

- **Cartographic scale mismatch.** A typical Portsmouth Harbor chart at
  1:20,000 is rendered at chart scale for human navigation, but
  autonomous vessels need higher effective resolution to safely approach
  the coast. Over-scaling the ENC for display does not uniformly
  improve feature resolution — different feature types are optimized for
  different display scales.
- **Vector representation at the proper scale.** Path planning needs
  features rendered at a navigation-appropriate scale (Figure 4.3).
- **Buffering objects to original chart-scale extent.** A pier shown as
  a thin line at chart scale is in reality a wider object (Figure 4.4 —
  Little Harbor).
- **Missing features.** Floating piers next to fixed piers may be omitted
  entirely; the breakwater near UNH Pier was installed in 2006 but absent
  from the ENC until summer 2018.
- **Misplaced features.** Reed observed a "Breakers" hazard label located
  88 m from the actual hazardous water, with the actual hazard visible in
  satellite imagery (white water) and confirmed by underlying bathymetry.
  This is "due to the cartographic depiction" — chart symbols are placed
  for human readability, not autonomy precision.

The takeaway: ENCs are designed for human chart-readers, not autonomous
robots. They embed cartographic generalization and human-readability
choices that introduce spatial errors at the scale autonomous vessels need.
This is a lasting structural limitation, not something an algorithm can
fully compensate for.

---

## Summary table

| Concern             | Reed's approach                                                      |
|--------------------|----------------------------------------------------------------------|
| Stack              | MOOS-IvP                                                             |
| Planning           | Custom Depth-Based A* on interpolated depth grid                     |
| Reactive control   | IvP behaviors with utility surfaces, combined in IvP Solver          |
| Chart representation | Two: depth-grid for planner, polygon `ENC_DB` for reactive         |
| Resolution         | Single, scaled by ENC compilation scale (0.25 mm × scale)            |
| Vessel handling    | Buffering at chart-time (planner) and threat-scaled buffering (`ENC_DB`); ASV treated as point |
| Threat encoding    | 5-level discrete + special "Land" level                              |
| Cost function      | Continuous: `G(n) = G(n-1) + DP + w·Cost_depth`                      |
| Tide              | Real-time scalar offset from harmonic constituents (`pytides`)       |
| Out-of-data       | -10 m sentinel                                                       |
| WATLEV handling   | Algorithm 1: qualitative → quantitative with Atlantic/Pacific scalar |
| DEPARE/SOUNDG conflict | Multi-raster (DA_shoal, DA_deep, Points, Polygons); shoaler wins |
| Test platforms    | Seafloor Systems EchoBoat (field), ASV Global C-Worker 4 (sim)       |
| Demonstrated      | Path planning, synthetic-ellipse OA, breakwater OA, pier OA          |
| Failure mode      | Reactive: local minima at multi-obstacle scenarios. Planner: close approaches to point obstacles in deep water |

## Code availability

The thesis itself doesn't link to a code repository, but Reed's GitHub
account [`sji367`](https://github.com/sji367) (registered as "Sam Reed,
UNH CCOM") hosts the original source for the work, plus his advisor Val
Schmidt published a C++ reimplementation of the gridding pipeline.
Most of the code dates from 2016–2017 (the thesis is December 2018) —
Reed evidently developed the system, then wrote the thesis describing it.

### Reed's own repositories

| Repo | Description | Last code push |
|------|-------------|----------------|
| [`sji367/MOOS_ENC`](https://github.com/sji367/MOOS_ENC) | The ENC obstacle-avoidance stack: `BHV_OA.cpp` (point-geometry OA behavior), `BHV_OA_poly.cpp` (polygon-geometry OA behavior — *split into two behaviors* in the published code, where the thesis describes one), `ENC_converter.py` (ENC→shapefile with threat-level annotation), `ENC_Search.py` (sliding-window search posting obstacles for OA), `ENC_Print.py` (pMarineViewer rendering), `ENC_WPT_check.py`, `AOF_Gauss.cpp` (custom IvP utility function with Gaussian falloff), plus example `alpha.moos`/`alpha.bhv` and a launcher. | 2016-09-27 |
| [`sji367/moos-ivp-reed`](https://github.com/sji367/moos-ivp-reed) | MOOS-IvP `extend`-style scaffolding for "Sam Reed's Nautical Chart Awareness MOOS work" — `bin/`, `lib/`, `missions/`, `scripts/`, `src/` directory layout. | 2022-12-22 |
| [`sji367/ENC_Mission_Planner`](https://github.com/sji367/ENC_Mission_Planner) | Qt5/QGraphics offline mission planner with chart loading, gridding (`griddingthread`), background raster rendering, and a vehicle-project model (`autonomousvehicleproject`). | 2017-06-23 |

The published code is broadly consistent with the thesis description but
contains evolutions: the single `BHV_ENC_OA` described in the thesis appears
as two distinct behaviors in `MOOS_ENC` (`BHV_OA` for point obstacles,
`BHV_OA_poly` for polygons), and `AOF_Gauss` (a Gaussian objective function)
exists in code but isn't called out by name in the thesis.

### Val Schmidt's reimplementation

| Repo | Description | Last code push |
|------|-------------|----------------|
| [`valschmidt/encgrid`](https://github.com/valschmidt/encgrid) | Standalone C++ tool that converts an ENC into a raster depth GeoTIFF, "based on Sam Reed's Master's Thesis." Built on GDAL+BOOST. CLI: `encgrid -f ENC.000 -b buffer -r resolution`. Produces intermediate per-class shapefiles (polygon, point, depth_area, outline) then `gdal_rasterize`s and grids. Includes a candid `notes_on_encgrid.txt` design log. | 2020-01-24 |
| [`valschmidt/enc_dump`](https://github.com/valschmidt/enc_dump) | Python tool to dump Layer/Feature/Attribute data from US ENCs (uses GDAL/OGR). Useful as a generic ENC inspection utility. | 2024-05 |
| [`valschmidt/ReadingENCswithGeopandas`](https://github.com/valschmidt/ReadingENCswithGeopandas) | Tutorial / Colab notebook for reading ENCs with Geopandas. | 2022-01 |

`encgrid` is Val Schmidt's standalone C++ port of Reed's planner-side
gridding. The included `notes_on_encgrid.txt` is a candid maintainer's
log — flags that geotiffs are written upside-down, that "MOOS_path was
hardcoded to Sam's HD," that the A* implementation was bundled but should
be split out, that internal scaling uses cm (multiply by 100), and walks
through the per-feature-class handling rules in `layer2XYZ()`.

The notes also surface several implementation choices not explicit in the
thesis:

- **MULTIPOINT features** (likely `SOUNDG` clusters) are extracted only as
  X/Y/depth and not pushed into a shapefile — they bypass the buffering
  pipeline.
- **`DEPCNT` (depth contour)** handling differs from other linestrings:
  segmented without buffering, with an unexplained UTM-origin subtraction
  that the maintainer flags with "WHY?".
- **`OBSTRN`, `PONTON`, `FLODOC`, `DYKCON`** linestrings are extracted,
  buffered into polygons, depth-checked (with `WATLEV` fallback) and stored
  in the polygon shapefile — *not* in the X/Y/depth point set.
- **`POINT` features** (rocks, wrecks, obstructions, land) are added to
  *both* the point shapefile and the X/Y/depth point set.

These are exactly the kind of details a re-implementer would want to know
and that the thesis description glosses over.

## Connections to this workspace

There are two separately documented points of contact between Reed's
work and this workspace, plus a broader topical overlap that doesn't
reduce to direct code re-use:

### Reed's `astar.h` is incorporated into `camp`

The file `camp/src/camp/astar.h` (and `astar.cpp`) carries the original
header:

```
/*
 * astar.h
 *  Created on: Mar 21, 2017
 *      Author: Sam Reed
 */
```

This was added to CAMP on 2020-02-20 in commit
`f464452 — "Add initial AStar support for Sam Reid's work"`
(`git log src/camp/astar.h`), about 14 months after Reed's thesis was
filed. The rest of CAMP is independently authored — CAMP's own first
commit is 2016-10-25, predating Reed's `ENC_Mission_Planner` (created
2017-05-16) by seven months. Shared filenames between the two Qt
projects (`autonomousvehicleproject`, `backgroundraster`,
`griddingthread`, etc.) exist; the direction of any influence on
those files is not established here and shouldn't be inferred from
filename overlap alone.

### `s57_tools` and Reed's gridding share a problem statement

`s57_tools` was first committed on 2021-06-25 by Roland Arsenault, two
and a half years after Reed's thesis. The history (`git log --reverse
--pretty=format:'%h %ad %s' --date=short`) shows it is original code
authored against the ROS 2 / nav2 stack, not a port of `encgrid` or
`moos-ivp-reed`. There is no commit message indicating an import from
those repos, and no Reed authorship in the history.

The conceptual overlap is genuine — both `s57_tools` and Reed's
gridding work convert ENC vector features into a 2D depth surface
suitable for navigation — but the implementations are independent.

### Topical overlap regardless of code

Many of the problems Reed worked through (depth-area conflict
resolution, qualitative-vs-quantitative depth via WATLEV,
tide-corrected charted depth, vessel-size buffering, threat encoding,
ENC scale and uncertainty) recur in any ENC-driven autonomy system,
including this one. The thesis remains a useful reference point even
where the code lineage is independent.
