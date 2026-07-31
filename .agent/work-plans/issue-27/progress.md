---
issue: 27
---

# Issue #27 — s57_to_geotiff: export ENC bathymetry as two-band GeoTIFFs for the chart layer

## Issue Review
**Status**: complete
**When**: 2026-07-31 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #27
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [x] Clarify CATZOC wiring scope: acceptance criteria require a CATZOC-varied fixture test, which means `marine_charts` `s57_dataset.cpp` case 308 must be wired. Either include this fix explicitly in this issue's scope, or open a prerequisite issue so it doesn't land silently at implementation time.
  — **Resolved (operator, 2026-07-31)**: wire CATZOC/M_QUAL in `marine_charts` **as part of this issue** (same repo, small change, acceptance requires it). The plan must carry it as an explicit work item.
- [x] Acknowledge cost-model gating: ADR-0010 D7 gates chart cell ingestion on the cost-model rework ("Chart ingestion is gated on the cost-model rework"). The round-trip acceptance demo (`export → import_geotiff → store query`) will hit this gate. Note the sequencing constraint explicitly so the acceptance test isn't blocked unexpectedly.
  — **Resolved (host)**: the gate applies to chart data feeding a *live costmap* deployment (uma#276, tracked as an Aug deploy precondition), not to the offline round-trip bench test (export → import → store query), which touches no costmap. The plan should note the sequencing constraint in its consequences section; it does not block this issue's acceptance.
- [x] Confirm uma#274 (`marine_vertical_datum`) is available before starting implementation — the datum conversion is load-bearing for the tool's correctness, and implementing the exporter without it would require a significant rework at wire-up time.
  — **Resolved (host)**: uma#274 merged 2026-07-24 (unh_marine_autonomy PR#279, `3578292`); uma#275 chart layer merged 2026-07-30 (PR#280, `6d3ca5c`). Both dependencies are satisfied.

## Plan Authored
**Status**: complete
**When**: 2026-07-31 15:26 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-27/plan.md` at `07aa8c3`
**Branch**: feature/issue-27 at `07aa8c3`
**Phases**: single

### Open questions
- [ ] Thread-safety scope for `make_vdatum_query()`: one factory call per cell or per corpus? (per-thread in v1 single-threaded loop = one call total)
- [ ] Output file naming: confirm `{S57Dataset::label()}.tif` is unambiguous across the New Castle ENC corpus before committing to it.

## Plan Review
**Status**: complete
**When**: 2026-07-31 15:30 +00:00
**By**: Claude Code Agent (Claude Opus)  <!-- independent: shares agent name with the Sonnet plan author but is a distinct model in a separate dispatched context; not an author self-review -->

**Plan**: `.agent/work-plans/issue-27/plan.md` at `07aa8c3`
**PR**: PR-less (reviewed from local plan + progress.md; `gh` unauthenticated in this context)
**Verdict**: approve-with-suggestions

### Findings
- [x] (must-fix) `s57_to_geotiff/package.xml` omits `marine_autonomy`, which provides `gggs::Level::fromCellSize`; `marine_vertical_datum` does not pull it in transitively — build blocker — `plan.md:29` — **folded at `8aeb7a4`** (dep added to Approach step 2 + Files table row)
- [x] (suggestion) M_COVR footprint source for the finer-scale clip is unspecified; `S57Dataset` exposes only a bbox — state the exporter reads M_COVR (OBJL 302) geometry directly — `plan.md:36` — **folded at `8aeb7a4`** (Approach step 3 "Footprint clip" bullet)
- [x] (suggestion) scale→level constant `0.0003125` (0.3125 mm-at-scale) diverges from ADR-0010 D7's "≈0.5 mm-at-scale" (0.0005); confirm or document — `plan.md:37` — **folded at `8aeb7a4`** (adopted 0.0005 per ADR + operator guidance; documented that 0.3125 mm is the S52 display-pixel size, a distinct quantity)
- [x] (suggestion) ADR table mixes project (uma ADR-0010 D7, ADR-0002 D2) and workspace (0008/0009/0018) ADRs under one unqualified namespace with number collisions; qualify the project ones — `plan.md:80` — **folded at `8aeb7a4`** ([uma]/[ws] tags added)
- [x] (suggestion) cost-model gate sequencing noted in Context but not the Consequences table; getGrid() case 308 stays a no-op — add a consequence row for completeness — `plan.md:89` — **folded at `8aeb7a4`** (cost-model-gate row added to Consequences)

## Implementation
**Status**: complete
**When**: 2026-07-31 16:05 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-27
**Plan**: `.agent/work-plans/issue-27/plan.md` (re-synced this branch; step-0 fold at `8aeb7a4`, open-question/overload re-sync at `18e9793`)

### Step 0 — Plan Review findings folded
All five approve-with-suggestions findings folded into plan.md at `8aeb7a4`
and checked off (annotations above, committed `510e991`). Constant reconciled to
`0.0005` (0.5 mm-at-scale per ADR-0010 D7); documented that
`recommendedResolution()`'s 0.3125 mm is the S52 display-pixel size, a distinct
quantity, so it is not used for level selection.

### Commits (Step 1)
- `aadf134` feat(marine_charts): `readCatzocZones()` M_QUAL reader (+ `GDALDataset*`
  overload; case 129/308 no-ops annotated to point at the new consumer)
- `6122f44` feat(s57_to_geotiff): the exporter package (CLI + `exporter.{hpp,cpp}`,
  no store dependency)
- `1980865` test(s57_to_geotiff): golden-file tests on a synthetic in-memory ENC fixture
- `73d8e33` docs(s57_to_geotiff): README (export rules + round-trip demo)
- `9e45149` fix(s57_to_geotiff): local-CI build/lint cleanups
- `04554f7` feat(s57_to_geotiff): report the chosen GGGS level per cell
- `18e9793` plan: resolve open questions + note the `readCatzocZones` overload

### What was built
- **CATZOC reader in `marine_charts`** (`readCatzocZones`): reads M_QUAL (OBJL 308)
  zones as WKB + CATZOC code; OGR stays out of the public header.
- **`s57_to_geotiff` package**: per-cell export loop — DEPARE/DRGARE band-midpoint
  depth + half-band σ floor (via GDAL rasterize), SOUNDG points (Z depth, override
  the area pixel), CATZOC→σ (`max(half-band, CATZOC)`), per-pixel chart-datum→
  ellipsoid via `marine_vertical_datum`'s full precedence chain, scale→GGGS level
  via `gggs::Level::fromCellSize(scale × 0.0005)`, largest-scale-governs clip by
  the union of finer cells' M_COVR footprints. Output: WGS84 two-band GeoTIFF
  (band1 ellipsoidal height, band2 σ, NaN no-data) in `import_geotiff`'s convention.

### Build & test results (actual)
Built in-container with `colcon build --packages-up-to s57_to_geotiff`
(ROS 2 Jazzy; GDAL 3.8.4, PROJ 9.4.0, yaml-cpp present). Dependencies
`marine_autonomy`, `marine_charts`, `marine_vertical_datum` all built.
- **s57_to_geotiff**: builds clean (no warnings/errors). `colcon test` →
  **16 tests, 0 errors, 0 failures, 4 skipped**; the gtest suite is
  **6/6 passing** (ZOC-table mapping, band midpoint + half-band σ floor,
  CATZOC-varied σ, sounding-overrides-area, level-selection-from-scale,
  finer-footprint clip). cppcheck + lint_cmake + xmllint pass.
- **marine_charts**: `colcon test` → 18 tests, 0 failures (13 skipped) — no regression.
- **CLI smoke test**: `--help` prints usage (exit 1, matching `import_geotiff`);
  an empty corpus reports "no charts found" and exits 0.

### Deviations from plan (plan re-synced this branch)
- **scale→level constant** `0.0003125`→`0.0005` (Plan Review finding #3; folded
  at `8aeb7a4`).
- **`readCatzocZones` overloads**: added a `GDALDataset*` overload beyond the
  planned path-only signature (lets the exporter/tests run on an already-open
  dataset). Additive; noted in plan step 1 at `18e9793`.
- **package structure**: `exporter.cpp` compiled into a small static
  `s57_to_geotiff_core` lib so the gtest links the core directly; `main.cpp` is a
  thin CLI over it. Consistent with the plan's exporter/main split.
- **output naming**: strip the `.000` extension (`baseLabel`) rather than using
  the raw `label()` — resolves plan open-question #2 (cell names are unique).
- **level reporting**: added the chosen GGGS level to the per-cell log line and
  `CellExport` so the operator can pass `import_geotiff --level N` for the
  round-trip (not in the original plan; makes the acceptance path usable).

### Follow-up notes (outside s57_tools — recorded, NOT edited here)
- **`import_geotiff` CLI does not accept the `chart` layer name.**
  `SourceLayer::Chart` exists (`bathy_cell.hpp`, uma#275) but
  `import_geotiff_main.cpp`'s `layerFromName()` maps only `survey|reference`.
  Teaching it `chart` (+ the `chart_staging_writable` / `replaceChartLayer`
  staging swap ADR-0010 D7 mandates) is a `unh_marine_autonomy` follow-up. Until
  then the README documents exercising the round-trip against `reference`
  (identical two-band ellipsoidal convention + multi-level import). This does not
  block this issue's exporter deliverable.

### Not done here (per resolved constraints, do not re-litigate)
- No real-NOAA-cell round-trip was executed (no ENC corpus or VDatum/geoid grids
  in-container); the round-trip is **documented** in the README as the acceptance
  path, and the golden-file tests cover the band/σ/level/clip logic on a synthetic
  fixture. The uma#276 cost-model gate is a live-costmap precondition, not a
  blocker for this offline bench (per Issue Review resolution).
- No push / PR / GitHub writes (host publishes later).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-31 16:15 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-27 at `54351a3`
**Mode**: pre-push
**Depth**: Deep (reason: 1512 lines > 200 and 11 files > 10; new C++/GDAL package + cross-package marine_charts edit)
**Must-fix**: 1 | **Suggestions**: 9
**Round**: 1 | **Ship**: continue — one mechanical input-validation guard; rest are suggestions, convergence expected next round
**Static analysis**: run (cppcheck 2.13, xmllint) — clean on new code (shadowVariable hits are pre-existing getGrid lines, outside the diff)
**Claude Adversarial**: 2 passes (Lens A + Lens B). **Copilot**: off (default). **Local**: skipped (Ollama not reachable).

### Findings
- [x] (must-fix) Validate `chart_scale > 0` and cap raster width/height before allocating — a scale==0 malformed cell reaches `gggs::Level::fromCellSize(0.0f)` where `static_cast<int>(ceil(log2(+inf)))` is UB, and `ceil(extent/pixel)` is unbounded — `s57_to_geotiff/src/exporter.cpp:198` — **fixed at `fc19042`** (early `chart_scale > 0` guard; dims computed in double and capped at `kMaxRasterDim` before the int narrowing)
- [x] (suggestion) Null-check MEM `work`/`mask` `Create()` returns (GTiff `out` is checked, MEM is not) — `s57_to_geotiff/src/exporter.cpp:224` — **fixed at `fc19042`** (both `Create()` returns null-checked)
- [x] (suggestion) SOUNDG with no M_QUAL gets σ=0.0 (no floor); band2=0 can read as false certainty — consider a documented minimum σ — `s57_to_geotiff/src/exporter.cpp:274` — **fixed at `fc19042`** (`kMinSoundingSigma = 0.5 m`, the CATZOC A1 base, floors every sounding's σ)
- [x] (suggestion) DEPARE/DRGARE CATZOC sampled at bbox centroid, burned across whole polygon; note the zone-straddling limitation — `s57_to_geotiff/src/exporter.cpp:257` — **fixed at `fc19042`** (documented in-code)
- [x] (suggestion) `runExport`'s documented "-1 on fatal setup error" never happens; setup failures (ignored `create_directories` ec, unavailable VDatum) exit 0 — wire a fatal path or drop the contract — `s57_to_geotiff/src/exporter.cpp:477` — **fixed at `fc19042`** (create_directories failure now returns -1; VDatum-unavailable stays a per-pixel no-data by design, not a fatal setup error)
- [x] (suggestion) All-no-data cell (written==0) still writes a GeoTIFF reported as "exported" — skip/warn — `s57_to_geotiff/src/exporter.cpp:350` — **fixed at `fc19042`** (written==0 now logs a warning and is not counted as exported)
- [x] (suggestion) `std::stod` for `--lake-datum` is unguarded — bad input aborts instead of printing usage — `s57_to_geotiff/src/main.cpp:55` — **fixed at `62966ec`** (try/catch → usage())
- [x] (suggestion) Equal-scale overlapping cells don't clip each other (strict `<`) — confirm import dedups or document — `s57_to_geotiff/src/exporter.cpp:511` — **fixed at `fc19042`** (documented: same-scale bands don't overlap in a standard ENC corpus; residual overlap left for import_geotiff to dedup)
- [x] (suggestion) `tf2`/`tf2_geometry_msgs` declared+linked but no direct use found (geographic_msgs is a genuine transitive dep) — confirm or prune — `s57_to_geotiff/CMakeLists.txt:17` — **fixed at `255b81d`** (pruned; both come transitively via marine_autonomy — verified build + 6/6 gtest still pass)
- [x] (suggestion) cppcheck: `for (Cell & cell : cells)` can be `const Cell &` — `s57_to_geotiff/src/exporter.cpp:538` — **fixed at `fc19042`** (const-qualified)
- [x] (note) Plan-drift: plan step 5 says SOUNDG depth = VALSOU, code uses geometry Z (getZ) — actually more correct for S-57; add to deviations list — `s57_to_geotiff/src/exporter.cpp:279` — **fixed at `fc19042`** (in-code comment records the deviation; also in the deviations list below)

## Implementation
**Status**: complete
**When**: 2026-07-31 16:27 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-27 at `255b81d`
**Addressed**: `## Local Review (Pre-Push)` (changes-requested, 2026-07-31 16:15 +00:00, branch at `54351a3`) — all 11 open findings (1 must-fix, 9 suggestions, 1 note)
**Commits**: `fc19042` (exporter.cpp), `62966ec` (main.cpp --lake-datum), `255b81d` (tf2 prune)

### Actions
- [x] (must-fix) chart_scale>0 guard + raster-dim cap before allocation — `s57_to_geotiff/src/exporter.cpp` — `fc19042`
- [x] Null-check MEM `work`/`mask` `Create()` returns — `s57_to_geotiff/src/exporter.cpp` — `fc19042`
- [x] Sounding σ floored at `kMinSoundingSigma` (0.5 m, CATZOC A1 base) — `s57_to_geotiff/src/exporter.cpp` — `fc19042`
- [x] Documented CATZOC bbox-centroid / zone-straddling limitation — `s57_to_geotiff/src/exporter.cpp` — `fc19042`
- [x] `runExport` returns -1 on `create_directories` failure (fatal-setup contract now real) — `s57_to_geotiff/src/exporter.cpp` — `fc19042`
- [x] All-no-data cell (written==0) warns instead of "exported" — `s57_to_geotiff/src/exporter.cpp` — `fc19042`
- [x] `--lake-datum` `std::stod` wrapped in try/catch → usage() — `s57_to_geotiff/src/main.cpp` — `62966ec`
- [x] Documented strict equal-scale non-clipping (import_geotiff dedups) — `s57_to_geotiff/src/exporter.cpp` — `fc19042`
- [x] Pruned unused `tf2`/`tf2_geometry_msgs` deps — `s57_to_geotiff/CMakeLists.txt`, `package.xml` — `255b81d`
- [x] const-qualified footprint-release loop (cppcheck) — `s57_to_geotiff/src/exporter.cpp` — `fc19042`
- [x] (note) Documented SOUNDG depth = geometry Z (not VALSOU) plan-drift — `s57_to_geotiff/src/exporter.cpp` — `fc19042`

### Deviations / decisions
- **SOUNDG depth source**: code reads the point geometry's Z ordinate, not a
  VALSOU attribute as plan step 5 worded it. Geometry Z is the correct S-57
  encoding for soundings; recorded here as the resolution of the plan-drift note.
- **VDatum-unavailable is not a fatal setup error**: the fatal path wired for
  the "-1" contract covers only `create_directories` failure. An unavailable
  VDatum query is intentionally a per-pixel no-data outcome (documented in
  `buildDatum`), not a run-fatal condition, so it stays a warning.
- **tf2 prune confirmed by build**: `colcon build --packages-select s57_to_geotiff`
  succeeds and the gtest suite stays 6/6 without the explicit tf2/tf2_geometry_msgs
  find_package/link/depend entries (they resolve transitively via marine_autonomy).

### Build & test (actual, post-fix)
`./build.sh s57_to_geotiff` → clean. `./test.sh s57_to_geotiff` → 0 errors,
0 failures; `test_exporter.gtest.xml` shows **6/6 gtest cases passing**
(ZOC mapping, band midpoint + half-band σ, CATZOC-varied σ, sounding-overrides-area,
level-selection, finer-footprint clip). No regressions from the σ-floor change
(the sole sounding test uses CATZOC A1 → σ 0.55 > the 0.5 floor).

### No push / PR
Local commits only; host publishes. No deferred findings — every open item was
actioned with a real commit.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to
a fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 27 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-31 16:42 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-27 at `4c47234`
**Mode**: pre-push
**Depth**: Deep (reason: 1645 lines > 200 and 11 files > 10; new C++/GDAL package + shared marine_charts public-API edit)
**Must-fix**: 0 | **Suggestions**: 5
**Round**: 2 | **Ship**: recommended — 0 must-fix after all 11 round-1 findings addressed; the one round-2 must-fix candidate was a verified false positive; remaining items are defensive-hardening suggestions
**Static analysis**: run (cppcheck 2.13, xmllint) — clean on new code (only cross-TU unusedStructMember false positives + one un-enforced useStlAlgorithm nit, dropped)
**Claude Adversarial**: 2 passes (Lens A logic + Lens B systemic). **Copilot**: off (default). **Local**: skipped (Ollama not reachable).

### Findings
- [x] (rejected/false-positive) Lens A "must-fix": lon pixel span uses lat angular span → wrong georeferencing — REJECTED: GGGS `latitudeScaleFactor`==1 for |lat|<72°, so cells are square-in-degrees in all mid-latitude waters; the square-degree geotransform matches the GGGS grid and is self-consistent — `s57_to_geotiff/src/exporter.cpp:216` — **no action** (dismissal, not a fix; the `<72°` validity boundary it rests on is now documented in-code + README at `cb5e01f`)
- [x] (suggestion) all-cells-failed / all-no-data corpus returns 0 and exits 0 (indistinguishable from success); return nonzero when `cells.size()>0 && exported==0` — `s57_to_geotiff/src/exporter.cpp:557` / `main.cpp:78` — **fixed at `98641d1`** (`runExport` returns -1 when a non-empty corpus exports nothing → `main` exits 1; header contract updated)
- [x] (suggestion) two `GDALRasterizeGeometries` returns unchecked — a CE_Failure silently drops depth pixels or under-clips (stale coarser data leaks through a finer footprint); check + warn per-cell like every other GDAL call — `s57_to_geotiff/src/exporter.cpp:294,369` — **fixed at `9277bf9`** (both returns checked; a failed depth burn or clip mask now fails the cell with a per-cell warning)
- [x] (suggestion) `kMaxRasterDim` blocks the int-overflow UB but a cap-sized cell still allocs ~TB → uncaught `bad_alloc` → terminate (no top-level catch); cap the pixel product and/or wrap the per-cell body to degrade to a skipped-cell warning — `s57_to_geotiff/src/exporter.cpp:38,228` — **fixed at `f53587e`** (added `kMaxRasterPixels` product cap + try/catch around the per-cell `exportCell` degrading any exception to a skipped-cell warning)
- [x] (suggestion) area pixels get σ exactly 0.0 for flat band + no CATZOC while soundings floor at `kMinSoundingSigma`; consider a matching area floor or document the asymmetry (low: real ENCs rarely encode DRVAL1==DRVAL2) — `s57_to_geotiff/src/exporter.cpp:289` — **fixed at `368f48f`** (floor only the degenerate σ==0 area case at `kMinSoundingSigma`; a genuinely narrow half-band is left intact and the asymmetry is documented in-code)
- [x] (note) polar-only (>72° lat): exporter keeps factor 1 while GGGS steps lon 3×/9× → E-W oversampling there (not data-loss; import resamples); a "square-degree, <72° lat" README note closes it — `s57_to_geotiff/src/exporter.cpp:216` — **fixed at `cb5e01f`** (README "square-degree, <72° lat" note + in-code comment)

### Next step
Lifecycle: **Local Review (approved)** → push / open PR → **triage-reviews**. The
five suggestions are non-blocking (apply now or track); no must-fix gates the push.

## Implementation
**Status**: complete
**When**: 2026-07-31 16:53 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-27 at `cb5e01f`
**Addressed**: `## Local Review (Pre-Push)` (approved, 2026-07-31 16:42 +00:00, branch at `4c47234`) — all 5 open findings (4 suggestions + 1 note); the 1 rejected/false-positive is a dismissal, not actioned.
**Commits**: `98641d1`, `9277bf9`, `f53587e`, `368f48f`, `cb5e01f`

### Actions
- [x] Non-empty corpus that exports nothing now returns -1 → CLI exits 1 (distinct from an empty corpus, which stays 0); header contract updated — `s57_to_geotiff/src/exporter.cpp:598`, `exporter.hpp` — `98641d1`
- [x] Both `GDALRasterizeGeometries` returns checked — a failed depth burn or clip-mask rasterize now fails the cell with a per-cell warning (no silently-dropped depth pixels, no under-clip leak) — `s57_to_geotiff/src/exporter.cpp` — `9277bf9`
- [x] Added `kMaxRasterPixels` pixel-product cap + try/catch around the per-cell `exportCell` degrading any exception (incl. `bad_alloc`) to a skipped-cell warning instead of a process terminate — `s57_to_geotiff/src/exporter.cpp` — `f53587e`
- [x] Degenerate σ==0 area pixel (DRVAL1==DRVAL2, no CATZOC) floored at `kMinSoundingSigma`; a genuinely narrow half-band is left intact; asymmetry vs SOUNDG documented in-code — `s57_to_geotiff/src/exporter.cpp` — `368f48f`
- [x] (note) Square-degree geotransform `<72°` lat validity documented — README "square-degree, <72° lat" note + in-code comment; >72° = E-W oversampling, not data loss — `s57_to_geotiff/README.md`, `s57_to_geotiff/src/exporter.cpp` — `cb5e01f`
- [x] (rejected/false-positive) Lens A lon-span "must-fix" — **no action** (the review already rejected it as a false positive; the `<72°` validity boundary it rests on is now documented, `cb5e01f`)

### Decisions
- **Area σ floor is targeted, not blanket.** An area's half-band *is* its stated
  uncertainty and can be legitimately small, so flooring every area pixel at
  `kMinSoundingSigma` (as soundings are) would inflate genuinely-narrow DEPARE
  bands. Only the degenerate exact-zero case (DRVAL1==DRVAL2 *and* no CATZOC),
  which is the "false certainty" the finding flagged, is floored.
- **Rasterize failures fail the cell rather than warn-and-continue.** A dropped
  depth burn or under-clipped mask is a silent correctness defect; skipping the
  cell (a per-cell warning in `runExport`) is safer than emitting a cell with
  missing pixels or leaked coarse data.
- **`kMaxRasterPixels` = 2.5e8.** Bounds the product well above any realistic ENC
  cell (tens of millions of pixels) while rejecting pathological extents; the
  per-cell try/catch is the backstop for any residual allocation failure.

### Build & test (actual)
`./build.sh s57_to_geotiff` → clean (no warnings/errors). `./test.sh
s57_to_geotiff` → **34 tests, 0 errors, 0 failures, 17 skipped**;
`test_exporter.gtest.xml` shows the gtest suite **6/6 passing** (ZOC mapping,
band midpoint + half-band σ, CATZOC-varied σ, sounding-overrides-area,
level-selection, finer-footprint clip) — no regression from the σ-floor or
rasterize-check changes.

### No push / PR
Local commits only; the host publishes. No deferred findings — every open item
was actioned with a real commit, except the rejected false-positive (a dismissal,
correctly left un-actioned).

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to
a fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 27 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-31 17:08 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-27 at `1953919` (code state `cb5e01f`; HEAD adds only progress.md)
**Mode**: pre-push
**Depth**: Deep (reason: new C++/GDAL package + shared marine_charts public-API edit; 1792 lines / 11 files)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 3 | **Ship**: recommended — 0 must-fix; round-2 approved and its 5 findings all addressed; this round's items are non-blocking suggestions / verified false-positives; loop converged
**Static analysis**: run (cppcheck 2.13, xmllint) — clean on new code (useStlAlgorithm nit + pre-existing getGrid shadowVariable dropped)
**Claude Adversarial**: 2 passes (Lens A logic + Lens B systemic). **Copilot**: off (default). **Local**: skipped (Ollama unreachable).

### Findings
- [ ] (suggestion) `work`/`mask`/`out` GDAL datasets use `unique_ptr`'s default deleter, not `GDALClose` like sibling marine_charts (`GDALDeleter`) / `GDALDatasetUniquePtr` — functionally correct (6/6 tests reopen and read back the written GeoTIFF), so idiom-consistency cleanup, not a bug — `s57_to_geotiff/src/exporter.cpp:266,385,459`
- [ ] (suggestion) dropped M_QUAL zone on WKB round-trip failure is silent; a missing zone removes its σ floor (possible false certainty) — very low likelihood (round-tripping freshly-produced WKB) but a one-line log would surface data-quality issues — `marine_charts/src/s57_dataset.cpp:436` / `s57_to_geotiff/src/exporter.cpp:102`
- [ ] (suggestion) marine_charts now exposes `readCatzocZones(GDALDataset*)` but doesn't `ament_export_dependencies(GDAL)`; sole current consumer (s57_to_geotiff) finds GDAL itself so no impact now — forward-looking note for future consumers — `marine_charts/CMakeLists.txt`
- [x] (rejected/false-positive) Lens B "critical resource leak / UB" on the `unique_ptr<GDALDataset>` deleter — REJECTED: GDALDataset has a public virtual destructor that flushes; the write-then-reopen-and-read tests pass 6/6, direct evidence the GTiff flushes/closes on destruction — **no action** (downgraded to suggestion #1, an idiom nit)
- [x] (rejected/below-threshold) CRS-not-WGS84 assumption (ENC spec-locked to WGS84), DRVAL2<DRVAL1 unwarned (`max()`→0 handles it), multiple-soundings-per-pixel last-wins (expected at raster res), sounding-index int overflow (bounded by kMaxRasterPixels) — **no action**
- [x] (rejected/false-positive) Lens A lon/lat square-degree "must-fix" — REJECTED again: same verified FP dismissed in round 2 (GGGS factor 1 for |lat|<72°); Lens A independently re-derived it as documented/acceptable — **no action**

### Governance & plan-drift
No governance concerns: consequences map fully addressed (marine_charts header→link, SOUNDG/M_QUAL comments, README round-trip, cost-model gate out-of-scope); [uma]ADR-0010 D7 / ADR-0002 D2 and [ws]ADR-0008 compliant. Plan drift: matches Files-to-Change; all deviations documented in prior entries. No undisclosed drift.

### Next step
Lifecycle: **Local Review (approved)** → push / open PR → **triage-reviews**. The three
suggestions are non-blocking (apply now or track); no must-fix gates the push.

## Integrated Review
**Status**: complete
**When**: 2026-07-31 14:15 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #29 at `18698a8`
**Sources**: 3 (Copilot R1 @ `18698a8`, Local Review (Pre-Push) R3 @ `1953919` — same code, `18698a8` is the progress-only commit on top, CI rollup)
**Cross-source confirmations**: 1
**CI**: all-pass (`build-and-test` success, `copilot-pull-request-reviewer` success)

### Findings
- [x] (cross-confirmed, must-fix) Output GeoTIFF `out` is destroyed by `unique_ptr`'s default deleter, so the close-time flush's `CPLErr` is discarded and `exportCell` returns `true` — a disk-full / I/O failure on the final GTiff write is silently swallowed and the cell is still counted "exported". Not the leak/UB that round 3's Lens B claimed (correctly rejected: `GDALDataset` has a public virtual dtor that flushes) — the defect is the *unreported* error. In-workspace precedent documents exactly this for GTiff writes (`marine_tiled_raster_store/src/tile_io.cpp:177-186`, checks `GDALClose(ds) != CE_None`), and this very file already uses `GDALClose` in a custom deleter at `exporter.cpp:497` — internal inconsistency. Fix: close `out` explicitly and check the `CPLErr` (GDAL 3.7+; workspace targets 3.8), failing the cell on error. MEM `work`/`mask` (266, 385) stay idiom-only — no file backing, nothing to flush — `s57_to_geotiff/src/exporter.cpp:459`
- [x] (must-fix, Copilot) `buildDatum()` builds the VDatum query only when `--geoid` is set, so `--vdatum-dir` alone is silently ignored: `make_vdatum_query` never runs, its "geoid_grid is empty" diag never fires, and every pixel falls through to no-data with no message. The reverse (`--geoid` alone) *is* diagnosed by the library. Usage text advertises the two flags independently. Fix: enter the block when either flag is set (letting `make_vdatum_query`'s diag report the missing one), or warn explicitly when exactly one is given — `s57_to_geotiff/src/exporter.cpp:522-534`
- [x] (suggestion, Copilot) Unknown `-`-prefixed arguments are accepted as positionals while fewer than two have been seen: `s57_to_geotiff --badflag out` runs with `enc_root="--badflag"` and exits 0 with "no charts found". Fix: reject an unrecognized argument beginning with `-` via `usage()` before the positional branch — `s57_to_geotiff/src/main.cpp:65`
- [x] (suggestion, Copilot) Test fixture `SyntheticCell` dereferences the results of `GetDriverByName("Memory")`, `Create()`, and `CreateLayer()` unchecked — any failure crashes the test binary instead of failing with a message. Production code null-checks the analogous MEM driver lookup (`exporter.cpp:259`), so the harness is the outlier. Fix: `ASSERT_NE(..., nullptr)` (or throw) on each — `s57_to_geotiff/test/test_exporter.cpp:25-28`
- [x] (suggestion, Copilot) `addField()` ignores `CreateField()`'s return; on failure the later `SetField()` calls silently no-op and tests fail far from the cause. Fix: assert `OGRERR_NONE` — `s57_to_geotiff/test/test_exporter.cpp:99-103`
- [x] (suggestion, Local Review R3 carry-over) A M_QUAL zone dropped on WKB export/round-trip failure is silent; the missing zone removes its σ floor (possible false certainty). One-line log at both ends — `marine_charts/src/s57_dataset.cpp:436` / `s57_to_geotiff/src/exporter.cpp:102`
- [x] (suggestion, Local Review R3 carry-over) `marine_charts` exposes `readCatzocZones(GDALDataset*)` without `ament_export_dependencies(GDAL)`. Verified low: the public header forward-declares `class GDALDataset` (`s57_dataset.h:12`) so it stays self-contained, and the sole consumer finds GDAL itself — forward-looking only — `marine_charts/CMakeLists.txt:72`

### False positives
- None dismissed outright this round — all 5 Copilot comments verified against local code as valid at some severity.
- (Copilot, partial) The `unique_ptr` deleter comment's implied resource-leak framing remains rejected on the round-3 grounds (virtual dtor flushes; 6/6 write-then-reopen tests pass). Only its error-reporting consequence is carried forward as the cross-confirmed must-fix above.

### Next step
Lifecycle: **Integrated Review** → `address-findings` (2 must-fix + 5 suggestions open).

## Implementation
**Status**: complete
**When**: 2026-07-31 18:32 +00:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #29 at `9aceeb3`   <!-- branch feature/issue-27 HEAD; fix commits are local, host pushes to PR #29 -->
**Addressed**: `## Integrated Review` (complete, 2026-07-31 14:15 -04:00, PR #29 at `18698a8`) — all 7 open findings (2 must-fix + 5 suggestions)
**Commits**: `0637a83`, `932d469`, `ca9f809`, `198946c`, `825fb3c`, `e9ef4da`, `f33a8f8`, `9aceeb3`

### Actions
- [x] (cross-confirmed, must-fix) GTiff `out` closed explicitly via `GDALClose(out.release())`; the close-time `CPLErr` now fails the cell instead of being swallowed by the default `unique_ptr` deleter. MEM `work`/`mask` left idiom-only (no file backing) — `s57_to_geotiff/src/exporter.cpp:459` — `0637a83`
- [x] (must-fix, Copilot) `buildDatum` enters the VDatum block when *either* `--geoid` or `--vdatum-dir` is set, so a lone `--vdatum-dir` is no longer silently ignored and `make_vdatum_query`'s missing-grid diagnostic fires — `s57_to_geotiff/src/exporter.cpp:522-534` — `932d469`
- [x] (suggestion, Copilot) Unknown `-`-prefixed CLI argument rejected via `usage()` before the positional branch (verified: `--badflag out` → "error: unknown option", exit 1) — `s57_to_geotiff/src/main.cpp:65` — `ca9f809`
- [x] (suggestion, Copilot) Fixture `SyntheticCell` guards the MEM driver/dataset/layer handles — `s57_to_geotiff/test/test_exporter.cpp:22-40` — `198946c` (+ `9aceeb3`, see decision below)
- [x] (suggestion, Copilot) Fixture `addField` checks `CreateField`'s return (`EXPECT_EQ … OGRERR_NONE`) — `s57_to_geotiff/test/test_exporter.cpp:100-104` — `825fb3c`
- [x] (suggestion, R3 carry-over) Dropped M_QUAL/CATZOC zone now logged at both ends (WKB export in `marine_charts`, WKB parse in the exporter); also fixes a latent geometry leak on a parse failure that left a partial geometry allocated — `marine_charts/src/s57_dataset.cpp:436` / `s57_to_geotiff/src/exporter.cpp:102` — `e9ef4da`
- [x] (suggestion, R3 carry-over) `ament_export_dependencies(GDAL)` added for the public `readCatzocZones(GDALDataset*)` API — `marine_charts/CMakeLists.txt:72` — `f33a8f8`

### Decisions
- **Fixture guard is a `throw`, not `ASSERT_NE`.** The Copilot finding offered "`ASSERT_NE(..., nullptr)` (or throw)". `ASSERT_NE` expands to a *value-returning* `return`, which is illegal in a constructor — the first build failed with `error: returning a value from a constructor`. The `throw std::runtime_error` alternative has the same effect (gtest reports it as a test failure with the message) and additionally prevents the downstream null-deref the finding was about. Corrective compile fix in `9aceeb3`, on top of the original `198946c`.
- **No deferred findings.** Every open item was actioned with a real commit. The source entry's two `False positives` plain bullets (the `unique_ptr` leak framing; the "none dismissed" note) are dismissals, not actions, and were correctly left un-actioned.

### Build & test (actual, post-fix)
`./build.sh marine_charts s57_to_geotiff` → clean (no warnings/errors). `./test.sh
marine_charts s57_to_geotiff` → **34 tests, 0 errors, 0 failures, 17 skipped**;
`test_exporter.gtest.xml` shows the gtest suite **6/6 passing**. CLI smoke:
`s57_to_geotiff --badflag /tmp/out` → "error: unknown option '--badflag'" + usage,
exit 1; `--help` exits 1 (matching `import_geotiff`).

### No push / PR
Local commits only; the host publishes to PR #29.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to
a fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 27 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-31 18:43 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-27 at `1569d1e`
**Mode**: pre-push
**Depth**: Deep (reason: new C++/GDAL package + shared marine_charts public-API edit; 1939 lines / 12 files)
**Must-fix**: 1 | **Suggestions**: 2
**Round**: 4 | **Ship**: continue — one new must-fix (scale-0 cell silently clips valid neighbors); mechanical one-line guard, fix-and-ship. Requires malformed input; happy path + 6/6 tests unaffected.
**Static analysis**: run (cppcheck 2.13, xmllint) — clean on new code (useStlAlgorithm nit unenforced; shadowVariable hits are pre-existing getGrid lines outside the diff)
**Claude Adversarial**: 2 passes (Lens A logic + Lens B systemic). **Copilot**: off (default). **Local**: skipped (Ollama unreachable).

### Findings
- [x] (must-fix) Scale-0 malformed cell (M_COVR present but no readable DSPM_CSCL -> chartScale()==0) is kept in Pass A `cells` with scale 0; Pass B clip predicate `other.scale < cell.scale` is `0 < scale` -> true for every real cell, so its footprints NaN out valid depth pixels from every overlapping cell while it fails its own export via the existing `chart_scale > 0` guard — silent data loss, inconsistent with that guard. Fix: filter scale<=0 cells out of Pass A with a warning — `s57_to_geotiff/src/exporter.cpp:620`
- [x] (suggestion) SOUNDG point exactly on the MaxX/MaxY extent edge with an exact-integer pixel ratio maps to col==width/row==height and is dropped; in the no-M_COVR fallback path the boundary-defining sounding can be silently lost — `s57_to_geotiff/src/exporter.cpp:347`
- [x] (suggestion) `forEachFeature` leaks the in-flight OGRFeature* if the callback throws (bounded single-feature leak per aborted cell on the OOM path runExport's try/catch handles); wrap in a scope guard — `s57_to_geotiff/src/exporter.cpp:73`

### Governance & plan-drift
No governance concerns: consequences map fully addressed (marine_charts header->link + GDAL export, SOUNDG/M_QUAL comments, README round-trip, cost-model gate documented out-of-scope); [uma]ADR-0010 D7 / ADR-0002 D2 and [ws]ADR-0008 compliant. Plan drift: matches Files-to-Change; all deviations documented in prior entries. No undisclosed drift.

### Next step
Lifecycle: **Local Review (changes-requested)** → `address-findings` (1 must-fix + 2 suggestions open) → re-dispatch review-code. The must-fix is a single mechanical guard; the two suggestions are non-blocking (apply or track).
