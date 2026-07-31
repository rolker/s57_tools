# s57_to_geotiff

Export ENC (S-57) chart bathymetry as two-band **(depth, σ)** GeoTIFFs for
import into the bathymetry store's `chart` layer, per
[unh_marine_autonomy ADR-0010 D7](../../unh_marine_autonomy/docs/decisions/0010-geospatial-world-model.md).

It is a standalone CLI tool: it depends on `marine_charts` (S-57 reading),
`marine_vertical_datum` (datum resolution), and `marine_autonomy` (GGGS level
math), but **not** on the store library — coupling to the store is only through
the GeoTIFF interchange that `import_geotiff` already consumes.

## What it produces

For every ENC cell in a corpus, one WGS84-geographic GeoTIFF:

| Band | Meaning | No-data |
|------|---------|---------|
| 1 | Seafloor depth as **WGS84 ellipsoidal height** (m, up-positive) | NaN |
| 2 | **1-σ** vertical uncertainty (m) | NaN |

These are exactly the band semantics `import_geotiff` expects (band 1 =
ellipsoidal height, band 2 = 1-σ, non-finite = no-data), so no `--depth-scale` /
`--depth-offset` conversion is needed at import.

## Export rules (ADR-0010 D7)

- **Depth sources**
  - `DEPARE` / `DRGARE` polygons → depth = band midpoint `(DRVAL1+DRVAL2)/2`;
    σ floor = half-band `(DRVAL2−DRVAL1)/2` (areas missing either limit are
    skipped — no defensible midpoint).
  - `SOUNDG` points → depth from the sounding's Z; a sounding overwrites the
    area value at its pixel (a discrete measurement is authoritative).
- **CATZOC → σ** (from `M_QUAL`, newly wired in `marine_charts::readCatzocZones`):
  A1 → 0.5 m + 1 %d, A2/B → 1.0 m + 2 %d, C → 2.0 m + 5 %d, D/U → a large σ
  (1000 m). The final σ is `max(half-band floor, CATZOC σ)`. Where no `M_QUAL`
  zone covers a feature, only the half-band floor applies.
- **Datum**: per-pixel chart-datum → WGS84 ellipsoid via `marine_vertical_datum`
  (full precedence chain: lake datum → config override → VDatum → config
  fallback). Band 1 = `datum_z − depth`. Pixels with no resolvable datum are
  written as no-data.
- **Scale → GGGS level**: each cell exports at the GGGS level matching its
  compilation scale, using **0.5 mm-at-scale** resolvable ground distance
  (`chartScale × 0.0005 m`) fed to `gggs::Level::fromCellSize`. The store is
  multi-level, so each cell lands at the level its scale warrants.

  > This 0.5 mm figure is the ADR's cartographic resolvable-ground-distance rule
  > and is deliberately **not** `S57Dataset::recommendedResolution()`'s 0.3125 mm,
  > which is the S52 minimum *display-pixel* size (864 lines / 270 mm) — a
  > different quantity governing on-screen legibility, not level selection.
- **Largest scale governs**: each cell's raster is clipped by the union of all
  strictly-finer cells' `M_COVR` coverage footprints (read directly from the
  OGR `M_COVR` geometry), so coarse cells are never generated where finer
  coverage exists.

## Usage

```
s57_to_geotiff <enc_root> <output_dir>
    [--geoid <geoid_grid>] [--vdatum-dir <dir>]
    [--datum-config <config.yaml>] [--lake-datum <metres>]
```

- `<enc_root>` — ENC corpus root (a `CATALOG.031`, or one subdirectory per cell).
- `--geoid` / `--vdatum-dir` — grids for the `marine_vertical_datum` query
  (ellipsoid ↔ MLLW). PROJ networking is disabled; the grids must be on local
  disk. Without them, datums come only from `--datum-config` / `--lake-datum`
  and any pixel with no resolvable datum is written as no-data.
- `--datum-config` — polygon→datum YAML (precedence-chain override/fallback).
- `--lake-datum` — constant lake-surface ellipsoidal height (m); wins outright
  everywhere (for inland waters with a single known surface).

## Round-trip demo (New Castle / Isles of Shoals)

This is the acceptance path: exporter output → `import_geotiff` → store query.
Uses NOAA ENC cells covering the New Castle, NH / Isles of Shoals area (e.g.
`US5NH02M` harbor, `US4NH01M` approach — download the current editions from the
[NOAA ENC catalog](https://charts.noaa.gov/ENCs/ENCsIndv.shtml)).

```bash
# 1. Export the corpus to GeoTIFFs (with local VDatum + geoid grids).
s57_to_geotiff ~/data/world/charts/ENC_ROOT /tmp/chart_export \
    --geoid /opt/vdatum/g2018u0.tif \
    --vdatum-dir /opt/vdatum

# 2. Import a cell into the store's chart layer at the level the exporter chose.
#    The export log prints the GGGS level per cell, e.g.
#      exported /tmp/chart_export/US5NH02M.tif (... GGGS level 12 -> import_geotiff --level 12)
#    Pass that level so the import matches the source resolution.
import_geotiff ~/data/world/store chart /tmp/chart_export/US5NH02M.tif --level 12

# 3. Query the store and confirm depths/σ round-trip to the charted values.
#    A charted 9.1 m (30 ft) sounding at MLLW should return the matching
#    ellipsoidal height ± its CATZOC σ.
```

The exporter writes ellipsoidal heights and 1-σ directly, so the import needs no
vertical rescale (`--depth-scale 1 --depth-offset 0`, the defaults).

### Note on the `chart` import target

The store's `Chart` source layer exists
([uma#275](https://github.com/rolker/unh_marine_autonomy/issues/275),
`SourceLayer::Chart`, ADR-0010 D3/D7). At the time of writing, the
`import_geotiff` **CLI** only maps the layer names `survey` and `reference`;
teaching its `layerFromName` the `chart` name (and the
`chart_staging_writable` opt-in / `replaceChartLayer` staging swap that D7
mandates) is a follow-up in `unh_marine_autonomy`, out of scope for this
S-57-side tool. Until then, exercise the round-trip against the `reference`
layer, which shares the identical two-band ellipsoidal convention and
multi-level import.

## Design notes

- **One vdatum factory per run.** `make_vdatum_query()` is built once (the export
  loop is single-threaded); its per-point query owns the PROJ context. A future
  parallel exporter must build one query per thread (see the header's
  thread-safety note).
- **Output naming.** `{cell_label}.tif`, where the label is the cell base name
  with its `.000` extension stripped. S-57 cell names are unique across a
  corpus, so the names never collide.
- **CATZOC wiring lives in `marine_charts`.** `readCatzocZones()` reads the
  `M_QUAL` (OBJL 308) zones that the costmap `getGrid()` path deliberately
  ignores; this exporter is their only consumer.
