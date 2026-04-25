# Design Evolution Log

Companion to [`alternative_costmap_architecture.md`](alternative_costmap_architecture.md).
Records what was added or changed to the design document over time, what
prompted each change, and what was wrong with the previous version. Use
this when reading the design doc to understand which sections were original
and which came from later questions / reviews — `git log` shows when, but
not why.

This is a living document. Append-only by convention; don't edit prior
entries except for typo fixes.

## 2026-04-25 — Initial draft (#23 / PR #24)

First pass at the clean-room design. Walked through inputs and constraints,
resolution and tiering, four prep questions (`SOUNDG`/`DEPARE`, `UNSARE`,
regulatory features, operating area), the multi-leg mission scenario, and
the resulting layered architecture with five layers, an explicit "out of
scope" list, and a "where confidence is low" section.

749 lines. Architecture had **5 layers** (Land, Hard Regulatory, Soft
Regulatory, Coverage, Depth). nav2 plugin model. Single global + sliding
local at 0.5 m.

## 2026-04-25 — Round 2: caching, restart cost, overhead clearance

**Triggered by follow-up questions** after the initial draft was committed:

> "Can you estimate how long this could take to load up and be ready for
> nav2 to plan? If a nav stack restart happens, what might be the down
> time? Also, are you accounting for overhead clearance?"

### What changed in `alternative_costmap_architecture.md`

- **Added a sixth layer: Overhead.** Sources are `BRIDGE`, `CBLOHD`,
  `CONVYR` (with `VERCLR`). Tide-conditional lethal cost, mirroring Depth
  with **opposite sign** on the tide dependency (high tide → less air
  clearance). New section 5.5 details the cost computation.
- **Added section 5.10: Persistence and lifecycle.** Covers persistent
  on-disk cache for parsed feature index, cold-start time estimates
  (~2–4 s harbor / ~8–20 s regional with cache; ~30–90 s regional
  without), restart downtime implications, and the deferred
  out-of-process ENC service alternative.
- **Updated the v1 scope** to include the Overhead layer + air-draft and
  the parsed-feature cache.
- **Updated the deferrals list** with drawbridge state, overhead-cable
  EMI hazards, and the out-of-process ENC service architecture.
- **Updated the update-model summary** (5.9, was 5.8) with rows for
  Overhead, air-draft, and nav2 restart.
- Renumbered subsections 5.6–5.9 (was 5.5–5.8) to make room for the new
  Overhead section.

### What was wrong in the initial draft

- **Overhead clearance was a real gap.** The initial draft listed `BRIDGE`
  in section 1's feature inventory but never created a layer for it.
  `CBLOHD` (overhead cable) was not even mentioned — only `CBLSUB`
  (submarine cable), which was correctly dismissed as "irrelevant for
  surface navigation," but that dismissal silently dropped the overhead
  case too. Vessels with significant air-draft (mast, antennas, sensors —
  larger boats run 5–10 m+) would have hit a charted bridge with no cost
  signal.
- **Caching was implicit, not designed.** Cold-start time was never
  analyzed. The design as written would force a 30–90 s cold start on
  regional missions, which has direct implications for nav2 restart
  behavior that should have been explicit. The doc now states the cache
  contract (per-cell, mtime-keyed) and quantifies the cost trade.
- **Vessel air-draft** was listed as a static vessel parameter in section
  1's input list but was never used in any cost computation. With the
  Overhead layer added, it now is.

### What's still open after Round 2

- **Whether to fold Overhead into Depth** for a single tide-driven
  recompute, or keep them as separate layers. Kept separate in the
  current design for cleaner debug provenance and operator understanding,
  but the implementation could share a recompute pass internally.
- **Whether the in-process plugin architecture needs revisiting** if
  field experience shows ~10–20 s regional restart is too long. The
  out-of-process ENC service alternative is sketched but deferred.
- **`warn` and `safe` thresholds for the Overhead clearance-to-cost
  function.** Stated to be smaller than the Depth equivalents but
  specific values not chosen.

## 2026-04-25 — Round 3: prior-art summary added

**Triggered by** the comparison-exercise framing being expanded to include
an explicit prior-art reference point. Specifically the request to "dig up
Sam Reed's thesis and prior work to use as an extra point of comparison"
and to "acknowledge this is the spiritual successor to that work."

### What was added

- **New file**: [`prior_art_reed_2018.md`](prior_art_reed_2018.md), a
  standalone neutral summary of Reed's 2018 MS thesis "Providing Nautical
  Chart Awareness to Autonomous Surface Vehicles" (UNH ECE,
  thesis-director Brian Calder, co-advisor Val Schmidt). Documents Reed's
  MOOS-IvP-based architecture, Depth-Based A* mission planner with cost
  function `G(n) = G(n-1) + DP + w·Cost_depth`, threat-level encoding,
  ENC_DB / ENC_Contact / BHV_ENC_OA reactive stack, MOOSTides tide
  handling, field test results, and Reed's stated limitations including
  the "ENC scale and uncertainty" critique (breakwater installed 2006 but
  omitted from ENC until 2018; "Breakers" label 88 m from actual hazard).

### What this is *not*

- Not a comparison to the alternative architecture in
  `alternative_costmap_architecture.md` or to the existing
  implementation in `s57_grids` / `s57_layer` / `marine_charts`. The
  comparison exercise is a separate planned step.
- Not a reproduction of the thesis. It captures the architectural
  decisions and limitations sufficient for using as a reference point;
  for full detail consult the thesis directly.

### Acknowledgment

The work in this repository is the spiritual successor to Reed's
research. Many of the problems Reed identified (depth-area conflict
resolution, qualitative-vs-quantitative depth, tide-corrected charted
depth, vessel-size buffering, ENC scale and uncertainty) recur in any
ENC-driven autonomy system, and Reed's solutions remain a useful
reference point.

## 2026-04-25 — Round 4: Reed's code repositories located

**Triggered by** the question "did you find a code repository for Reed's
work? Can we find it and see if it contains more insight we should know
about?"

### What changed in `prior_art_reed_2018.md`

- **New "Code availability" section** with three Reed-authored repositories
  ([`sji367/MOOS_ENC`](https://github.com/sji367/MOOS_ENC),
  [`sji367/moos-ivp-reed`](https://github.com/sji367/moos-ivp-reed),
  [`sji367/ENC_Mission_Planner`](https://github.com/sji367/ENC_Mission_Planner))
  and three Val-Schmidt reimplementation/tooling repos
  ([`valschmidt/encgrid`](https://github.com/valschmidt/encgrid),
  [`valschmidt/enc_dump`](https://github.com/valschmidt/enc_dump),
  [`valschmidt/ReadingENCswithGeopandas`](https://github.com/valschmidt/ReadingENCswithGeopandas)).
- **Implementation details** lifted from `encgrid`'s `notes_on_encgrid.txt`
  (MULTIPOINT/SOUNDG bypass, `DEPCNT` linestring oddities, `OBSTRN`/`PONTON`/
  `FLODOC`/`DYKCON` polygon-conversion path, internal cm-scaling, geotiff
  upside-down quirk).
- **Connections to this workspace** documented (verified against git
  history, not inferred from filename overlap):
  - `camp/src/camp/astar.h` and `astar.cpp` were imported from Reed on
    2020-02-20 (commit `f464452 — "Add initial AStar support for Sam
    Reid's work"`), about 14 months after the thesis. The rest of CAMP
    is Roland Arsenault's independent work since 2016-10-25. Reed's
    `ENC_Mission_Planner` was created 2017-05-16 — *after* CAMP — and
    they share several filenames; the direction of any influence on
    those files is not established here.
  - `s57_tools` was first committed 2021-06-25 by Roland, ~2.5 years
    after Reed's thesis. Its git history shows independent
    authorship — no port commit from `encgrid` or `moos-ivp-reed`.
    The conceptual overlap is real (both produce an ENC-derived 2D
    depth surface), but the implementations are independent.

### What was wrong / missing before

- The Round-3 prior-art summary correctly cited the thesis but assumed
  the code was unfindable. In fact Reed publishes under `sji367` and
  his repos are public. Without the code, the prior-art doc had no way
  to surface implementation choices that the thesis prose glosses
  over — notably the SOUNDG/MULTIPOINT bypass, the per-feature-class
  linestring rules, and the existence of an `AOF_Gauss` IvP utility
  function not named in the thesis.
- An earlier draft of this Round-4 entry asserted code lineages
  ("CAMP ← Reed's `ENC_Mission_Planner`", "s57_tools ← encgrid ←
  Reed's gridding") inferred from shared filenames. Git log
  contradicted that: CAMP predates Reed's Qt planner by seven months,
  and `s57_tools` has no import history from `encgrid`. The corrected
  text states only what git history confirms (specific imported
  files; independent first-commit dates) and explicitly does not
  narrate direction of influence beyond what is documented in commits.
  The user flagged the inference; existing memory feedback
  ("No causal stories from similarity") was reinforced to add the
  code-lineage-specific guidance "git log first."

### Reed's repository activity

The "last update" timestamps from GitHub search results (e.g.
2025-07 on `MOOS_ENC` and `ENC_Mission_Planner`) are repository
metadata changes, not code commits. Actual final code commits per
repo:

- `MOOS_ENC` — 2016-09-27 ("Updated the comments")
- `ENC_Mission_Planner` — 2017-06-23 ("Cleaned up repo")
- `moos-ivp-reed` — 2018-07-07 (uploading thesis-fieldwork log files
  from "Pier Ops Day 2")

Reed's code is essentially frozen since his thesis defense.

### What's still open

- The published `MOOS_ENC` code shows a `BHV_OA` / `BHV_OA_poly` split
  (point vs polygon obstacles) that the thesis describes as a single
  `BHV_ENC_OA`. Reading the actual code might surface other
  thesis-vs-code drift worth knowing for the comparison phase. Not done
  yet — flag for the comparison step if it becomes load-bearing.
- The relationship between Reed's `moos-ivp-reed` (the moos-ivp-extend
  scaffolding) and `MOOS_ENC` (the thesis-component code) isn't fully
  mapped. They're both his work but the directory layouts differ; one
  may be a refactor of the other or they may have served different
  demos.

## How to add a new entry

When the design doc gets meaningful changes (new section, changed
recommendation, dropped feature, scope shift), add an entry here:

```markdown
## YYYY-MM-DD — Short title

**Triggered by**: <comment, review, field experience, related issue, etc.>

### What changed

- Specific section / decision changed.
- ...

### What was wrong / missing before

- Honest accounting of the gap.
- ...

### What's still open

- ...
```

Don't use this log for typo fixes, formatting, or clarifying rewrites that
don't change the design. Use it when a future reader of the doc would
benefit from knowing "this section was added in response to X."
