# Plan: Chart updater — keep ENC corpus current and regenerate chart layer on change

## Issue

https://github.com/rolker/s57_tools/issues/28

## Context

`s57_to_geotiff` (issue #27, PR #29, merged 2026-07-31) and `import_geotiff
--stage/--commit` (already merged in `unh_marine_autonomy`) cover the
export and atomic-swap steps. What is missing is the cron-friendly
orchestrator — the `enc_updater` package — that drives the full D7 pipeline:
download → change-detect → regenerate → interlock-check → swap → health-record.

The operator has resolved both open questions from the Issue Review:

1. **Nav-down liveness signal = ROS graph probe**: at swap time only, the
   updater sources the ROS env and runs `ros2 node list` (or `ros2 topic
   list`) with a short timeout; the presence of any known nav-stack
   node/topic aborts the swap. No new sentinel-file contract.
2. **Regeneration-API invocation = CLI subprocess**: `import_geotiff
   --stage` and `import_geotiff --commit` are called as subprocesses via
   `subprocess.run`, matching the existing chart-chain CLI pattern.

## Approach

1. **New `enc_updater` package** (`ament_python`) in `s57_tools/enc_updater/`.
   Pure offline tooling: no ROS runtime dependency except the transient graph
   probe at swap time. No pip-only runtime deps (ADR-0009); HTTP via stdlib
   `urllib` (review suggestion adopted — no `requests`).

2. **`downloader.py`** — fetch the NOAA ENC product catalog
   (`https://charts.noaa.gov/ENCs/ENCProdCat.xml`, verified live 2026-08-03:
   `<cell>` elements carrying `name`/`edtn`/`updn`/`zipfile_location`/
   `zipfile_size`; the catalog provides **no checksums**) for the configured
   region (list of cell IDs); compare edition/update fields against the corpus
   manifest (per-cell JSON in `corpus_dir/.manifest.json`); download only
   changed/new cells as ZIP; validate before extracting — byte count vs the
   catalog's `zipfile_size` plus zip CRC verification (`zipfile.testzip`), the
   strongest integrity signal the catalog supports; update manifest on
   success. Failed or validation-failing downloads leave the corpus and
   existing manifest untouched.

3. **`registry.py`** — read/write the edition registry (`editions.json`) that
   records per-cell edition+update numbers for the **active** chart layer.
   Written inside **`<staged_dir>/chart/`** before `--commit` — `--commit`
   swaps only the `chart/` subdir (`replaceChartLayer`, which skips non-`.tif`
   entries), so the registry must live there to ride the atomic rename as the
   single commit point; old registry stays at `<store_dir>/chart/editions.json`
   until the swap completes.

4. **`regenerator.py`** — orchestrate the regeneration cycle:
   a. Create a fresh staged dir **adjacent to the store**
      (`<store_dir>/../.enc_updater_staging.<pid>`) — never `/tmp`: `--commit`'s
      atomic `rename(2)` requires staged and store on the **same filesystem**
      (EXDEV otherwise; review must-fix). Fail if non-empty to prevent stale
      tile accumulation per `import_geotiff --stage` warning.
   b. Call `s57_to_geotiff <corpus_dir> <geotiff_tmp> [--geoid …] [--vdatum-dir …]`
      as a subprocess. Parse the per-cell export log lines
      (`exported <path> (WxH, N cells, scale 1:S, GGGS level L -> import_geotiff --level L)`)
      to get each GeoTIFF's GGGS level; cells logged as empty/warned are not
      staged.
   c. For each exported GeoTIFF, call
      `import_geotiff --stage <staged_dir> chart <geotiff> --level <L>` as a
      subprocess — the level always passed explicitly from the export log
      (review suggestion: no silent `--cell-size` derivation), with optional
      config overrides for `--cell-size`.
   d. Sanity-check: at least one staged tile, all files non-zero, spot-check
      that band-1 values are in a plausible range (default −12 000 m to
      **+100 m** ellipsoidal — the +100 upper bound admits inland/lake surfaces,
      e.g. Massabesic ≈ +52 m ellipsoidal; config-overridable) via the GDAL
      Python bindings (`python3-gdal` rosdep key — cleaner to mock and no
      extra CLI packaging question vs a `gdalinfo` subprocess).
   e. Write `editions.json` into `<staged_dir>/chart/` from the downloader
      manifest.
   f. **Nav-down interlock**: source `/opt/ros/<distro>/setup.bash` in a
      subprocess; run `ros2 node list`; abort swap if any of the configured
      nav-liveness node names appears. Log the refusal and exit non-zero.
   g. Call `import_geotiff --commit <staged_dir> <store_dir>` as a subprocess.
   h. Remove `<geotiff_tmp>` and the now-committed staged dir.

5. **`health.py`** — maintain `<corpus_dir>/.updater_health.json` with:
   `last_download_attempt`, `last_download_ok`, `last_regen_ok`, `last_error`.
   Written after each phase so repeated failures age the layer loudly.

6. **`__main__.py`** — argparse CLI with `--dry-run`, `--config`, `--force`
   (skip change-detection). `--dry-run` runs download + change-detect + export
   but skips the interlock check and `--commit`. Exits 0 on no-change (idempotent).

7. **`config/region_example.yaml`** — example region config for New Castle /
   Isles of Shoals cells, with cron example and store/corpus paths. Must state
   the deployment prerequisite (review must-fix, resolved): the target store's
   deployed consumer code must include `unh_marine_autonomy#276`
   (worst-case-clearance cost model + confidence gate, **merged 2026-08-03**) —
   older deployed `bathymetry_layer` builds render high-uncertainty chart cells
   LETHAL under `unsurveyed_is_lethal`, so hosts still on pre-#276 builds must
   rebuild before this updater's store feeds a live costmap.

8. **Tests** (`test/`) — no network, no real ENC corpus:
   - `test_downloader.py`: mock HTTP failure mid-download → corpus unchanged;
     checksum mismatch → corpus unchanged.
   - `test_regenerator.py`: mock `s57_to_geotiff` failure → staged dir absent;
     mock sanity-check failure → commit never called, prior layer intact; mock
     `--commit` failure → prior layer intact.
   - `test_nav_liveness.py`: mock `ros2 node list` returning a nav node name
     → swap refused (non-zero exit, `--commit` not called); mock empty list →
     swap proceeds.
   - `test_idempotent.py`: two runs with same manifest → second run is no-op
     (no subprocess calls past change-detect).

9. **README.md** — operational guide: installation, config schema, cron
   example (`0 2 * * * enc_updater --config /etc/enc_updater/region.yaml`),
   health-file location, nav-liveness node list config, troubleshooting.

## Files to Change

| File | Change |
|------|--------|
| `enc_updater/package.xml` | New `ament_python` package |
| `enc_updater/setup.py` | Entry point: `enc_updater = enc_updater.__main__:main` |
| `enc_updater/setup.cfg` | Package config |
| `enc_updater/enc_updater/__init__.py` | Empty |
| `enc_updater/enc_updater/__main__.py` | CLI (`--dry-run`, `--config`, `--force`) |
| `enc_updater/enc_updater/config.py` | YAML region-config load + validation |
| `enc_updater/enc_updater/downloader.py` | NOAA catalog fetch + checksum + corpus manifest |
| `enc_updater/enc_updater/registry.py` | Edition registry R/W in staged dir |
| `enc_updater/enc_updater/nav_liveness.py` | ROS graph probe (`ros2 node list`) |
| `enc_updater/enc_updater/regenerator.py` | Orchestrate s57_to_geotiff + stage + sanity + commit |
| `enc_updater/enc_updater/health.py` | Health JSON read/write |
| `enc_updater/config/region_example.yaml` | Example config (New Castle / Isles of Shoals) |
| `enc_updater/test/test_downloader.py` | Simulated-failure: failed/bad-checksum download |
| `enc_updater/test/test_regenerator.py` | Simulated-failure: failed export/sanity/commit |
| `enc_updater/test/test_nav_liveness.py` | Interlock: swap refused with nav node present |
| `enc_updater/test/test_idempotent.py` | No-change run = no-op |
| `enc_updater/README.md` | Operational guide + cron example |
| `README.md` | Add `enc_updater` to package inventory |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Robustness is not optional | Every failure mode (download, export, sanity, swap) leaves the prior layer + registry fully intact; interlock is enforced, not assumed |
| Test what breaks | Tests target the four failure modes from the acceptance criteria, not happy-path coverage |
| A change includes its consequences | README and top-level package inventory updated in this PR |
| Only what's needed | No new store library coupling; subprocess CLIs already exist |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0010 D7 (**unh_marine_autonomy** `docs/decisions/0010-geospatial-world-model.md` — not the workspace repo's ADR-0010, which is git-bug; review suggestion) | Yes | Wholesale regeneration, atomic swap, nav-down interlock, registry-inside-staged-dir are all implemented as specified |
| ADR-0009 | Yes | `ament_python` package; no pip-only runtime deps; standard library + ROS 2 CLI subprocesses |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add `enc_updater` package | `README.md` package inventory | Yes — step 9 + files table |
| Nav-liveness node list is config-driven | `region_example.yaml` documents the field | Yes — step 7 |
| `--commit` CLI in `import_geotiff` | No s57_tools change needed; already merged | N/A |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `README.md` package inventory (currently lists
  only s57_grids, s57_layer, marine_charts, s57_msgs — misses s57_to_geotiff too; fix
  both in this PR).
- **Agent-instruction candidates**: nav-liveness probe pattern (source ROS env in a
  subprocess, parse `ros2 node list` output) may be worth capturing in
  `.agents/README.md` for the next agent working in this repo.

## Open Questions

- [ ] No open questions — operator checkpoint has resolved all; plan is review-plan-ready.

## Estimated Scope

Single PR. Closes #28 and Closes #5.
