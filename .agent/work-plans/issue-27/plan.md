# Plan: s57_to_geotiff — export ENC bathymetry as two-band GeoTIFFs for the chart layer

## Issue

https://github.com/rolker/s57_tools/issues/27

## Context

`marine_charts` already opens S57 ENC files via GDAL and rasterizes them into `grid_map::GridMap`
for the costmap pipeline. That path processes DEPARE/DRGARE with only DRVAL1 (min depth), ignores
SOUNDG (case 129), and silently discards M_QUAL/CATZOC data (case 308). The new tool needs
DRVAL2 for band-midpoint depth and half-band σ, SOUNDG points for sounding depth, and CATZOC from
M_QUAL for σ floor — none of which are exposed by the existing API.

Both upstream dependencies are satisfied: `marine_vertical_datum` merged 2026-07-24 (uma PR#279)
and the `chart` store layer merged 2026-07-30 (uma PR#280). The round-trip acceptance test
(export → `import_geotiff` → store query) does not depend on the cost-model rework gate
(uma#276) — that gate applies only to live costmap deployment, not the offline bench test.

## Approach

1. **Add CATZOC-zone reader to `marine_charts`** — new free function
   `readCatzocZones(const std::string& path)` in `s57_dataset.cpp` / `s57_dataset.h`
   that reads M_QUAL (OBJL 308) features and returns
   `std::vector<CatzocZone>{wkb, catzoc_code}`. OGR types stay out of the public
   header; the caller reconstructs geometry from WKB. Case 308 in `getGrid()`
   stays a no-op (the costmap path does not consume CATZOC); the new reader is a
   separate entry point.

2. **New `s57_to_geotiff` ROS 2 package** in this repo (CLI binary only; no store dependency):
   - `s57_to_geotiff/package.xml` — depends on `marine_charts`, `marine_vertical_datum`,
     **`marine_autonomy`** (header-only `gggs::Level::fromCellSize`; NOT pulled in
     transitively by `marine_vertical_datum`), GDAL
   - `s57_to_geotiff/CMakeLists.txt`
   - `s57_to_geotiff/src/main.cpp` — argument parsing, top-level loop
   - `s57_to_geotiff/src/exporter.hpp/cpp` — core export logic

3. **Exporter algorithm** (per-cell loop):
   - `S57Catalog` discovers all cells; sort by chart scale coarsest→finest
     (largest scale-denominator first; finest = smallest denominator)
   - **Footprint clip**: the exporter reads each cell's **M_COVR (OBJL 302)
     coverage geometry directly** from the OGR layer (S57Dataset exposes only a
     bbox, not the polygon). The clip mask subtracted from a coarser cell is the
     union of all finer-scale cells' M_COVR footprints.
   - Per cell: open GDAL dataset via `S57Dataset::filePath()`; call `readCatzocZones()`;
     compute GGGS level via `gggs::Level::fromCellSize(chartScale() * 0.0005)`
     (see note below on the constant)
   - DEPARE/DRGARE (OBJL 42/46): depth = (DRVAL1+DRVAL2)/2; σ = max((DRVAL2-DRVAL1)/2, catzoc_sigma(catzoc_at_centroid))
   - SOUNDG (OBJL 129): depth = VALSOU; σ = catzoc_sigma(catzoc_at_point)
   - Datum: `marine_vertical_datum::make_vdatum_query()` (full precedence chain via
     `resolve_datum`) gives the chart datum's ellipsoidal height per pixel;
     band1 = datum_z − depth (WGS84 ellipsoidal, up-positive)
   - Write GeoTIFF: WGS84 geographic, band1=ellipsoidal height (m), band2=σ (m), NaN no-data

   **Scale→level constant (finding #3, reconciled):** ADR-0010 D7 specifies
   "≈0.5 mm-at-scale resolvable ground distance". The exporter uses
   `chartScale() * 0.0005` (0.5 mm) accordingly. This deliberately differs from
   `S57Dataset::recommendedResolution()`'s `0.0003125` (0.3125 mm): that constant
   is the **S52 minimum display-pixel size** (864 lines / 270 mm ⇒ 3.2 lines/mm ⇒
   0.3125 mm/pixel) governing on-screen chart legibility — a different quantity
   than the resolvable ground distance that governs GGGS level selection. Per
   operator guidance, absent a documented derivation tying the 0.3125 mm display
   figure to resolvable ground distance, we align level selection with the ADR's
   0.5 mm cartographic-resolution rule and document the distinction here and in code.

4. **CATZOC σ mapping** (free function, `exporter.cpp`):
   A1→ 0.5 m + 0.01·depth, A2/B→ 1.0 m + 0.02·depth, C→ 2.0 m + 0.05·depth, D/U→ 1000 m

5. **Tests** (`s57_to_geotiff/test/`):
   - `test_exporter.cpp` — golden-file C++ test on a minimal synthetic S57 fixture (programmatic GDAL OGR layers, no real ENC file required): verifies band values, σ from CATZOC variation, level selection from chart scale, and coarser-cell clip where a finer cell covers the same footprint
   - Fixture built in-memory; no network or download needed

6. **README** — usage, CLI flags, and documented round-trip demo:
   `s57_to_geotiff <enc_root> <output_dir> [--geoid <g2018.tif>] [--vdatum-dir <dir>]`
   followed by `import_geotiff <store> chart <output_dir>/US4NH01M.tif`
   exercised against a real NOAA cell (New Castle / Isles of Shoals).

## Files to Change

| File | Change |
|------|--------|
| `marine_charts/include/marine_charts/s57_dataset.h` | Add `CatzocZone` struct + `readCatzocZones()` declaration |
| `marine_charts/src/s57_dataset.cpp` | Implement `readCatzocZones()` (reads M_QUAL features; case 308 in getGrid() stays a no-op) |
| `s57_to_geotiff/CMakeLists.txt` | New package build |
| `s57_to_geotiff/package.xml` | New package manifest (deps: marine_charts, marine_vertical_datum, marine_autonomy, GDAL) |
| `s57_to_geotiff/src/main.cpp` | CLI entry point |
| `s57_to_geotiff/src/exporter.hpp` | Exporter class interface |
| `s57_to_geotiff/src/exporter.cpp` | Core export logic |
| `s57_to_geotiff/test/test_exporter.cpp` | Golden-file + clip tests |
| `s57_to_geotiff/README.md` | Usage + round-trip demo |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | CATZOC wiring in marine_charts is done here; SOUNDG case 129 comment updated |
| Test what breaks | Golden-file tests verify band values, σ, level selection, and clip — the four failure modes that would silently corrupt store content |
| Only what's needed | No store dependency; no orchestration logic; datum conversion via existing library |
| Workspace vs. project separation | Tool lives in s57_tools (project repo); no workspace-layer changes |

## ADR Compliance

ADR numbering collides between namespaces (both the project and the workspace
have an ADR-0010, ADR-0002, etc.), so each row below is qualified: **[uma]** =
project ADR (`unh_marine_autonomy/docs/decisions/`), **[ws]** = workspace ADR
(`docs/decisions/`).

| ADR | Triggered | How addressed |
|---|---|---|
| [uma] ADR-0010 D7 (geospatial world model) | Yes — this IS the implementation | DEPARE/DRGARE/SOUNDG → two-band GeoTIFF per the spec; CATZOC σ, datum via marine_vertical_datum, scale→level, finer-footprint clip |
| [uma] ADR-0002 D2 (multi-level store) | Yes | GGGS level derived from chart scale via `gggs::Level::fromCellSize`; passed as `options.level` to `import_geotiff` |
| [ws] ADR-0008 (ROS 2 naming) | Yes | Package name `s57_to_geotiff`, binary `s57_to_geotiff`, ament_cmake |
| [ws] ADR-0009 (Python pkg management) | No | C++ only |
| [ws] ADR-0018 (local-first CI) | Yes | `ci_local.sh` on this repo at merge |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `marine_charts` public header (CatzocZone) | `s57_to_geotiff` CMakeLists.txt must link marine_charts | Yes |
| SOUNDG case 129 in `s57_dataset.cpp` (currently ignored) | Comment only — getGrid() intentionally skips soundings for the costmap path | Yes (comment) |
| Output GeoTIFF band semantics | README round-trip demo command | Yes |
| Feeding chart cells to a **live costmap** ([uma] ADR-0010 D7 gate; uma#276 cost-model rework) | Out of scope here — the gate applies to live-costmap deployment, NOT this issue's offline round-trip bench (export → import_geotiff → store query, which touches no costmap). Noted so acceptance isn't unexpectedly blocked. | N/A (documented) |

## Open Questions

- [ ] Does `marine_vertical_datum::make_vdatum_query()` need to be called once per cell or once per corpus? Thread safety note in the header says one call per thread — the per-cell loop is single-threaded in v1, so one factory call total is correct.
- [ ] Naming convention for output files: `{cell_label}.tif` matches the label from `S57Dataset::label()` (filename without path); confirm this is unambiguous across the New Castle corpus before committing to it.

## Estimated Scope

Single PR. All changes in this repo (s57_tools). Cross-repo dependency `marine_vertical_datum`
is already merged and available in the workspace underlay.
