# Plan: enc_updater: provision world/datum/ geoid and VDatum grids

## Issue

https://github.com/rolker/s57_tools/issues/37

## Context

`enc_updater` consumes `geoid` and `vdatum_dir` from region config for the
D7 chart export (`s57_to_geotiff --geoid / --vdatum-dir`), but has no
provisioning step to download those files. ADR-0010 D3 (amended 2026-08-20,
uma#288) defines `~/data/world/datum/geoid/` and `~/data/world/datum/vdatum/`
as the canonical grid locations and marks them as "intended to be
updater-managed". The CMake-download block in `mru_transform` (uma#288 item 6)
cannot be removed until **both gabby and salmon** show `world/datum/`
population in their deploy logs.

**Geoid fetch mechanism (plan checkpoint — operator reviews before
implementation):** the plan uses direct cdn.proj.org HTTP, which mirrors the
`downloader.py` pattern (no subprocess / projsync dependency, scheme
allow-listed, Content-Length integrity check, same download-cap guards). The
alternative, `projsync` CLI, is cleaner for multi-grid scenarios but adds a
CLI dependency that may not be present on field hosts. See Open Questions.

## Approach

1. **Add `enc_updater/datum_provisioner.py`** — new module with two public
   functions: `ensure_geoid(cfg)` and `ensure_vdatum(cfg)`. Both are
   idempotent (check presence before downloading), follow the same
   HTTP/integrity pattern as `downloader.py`, and raise `UpdaterError` on
   failure so callers see a clean error message.

   - `ensure_geoid`: if `cfg.geoid` is set and the file is absent, derive the
     CDN filename from `os.path.basename(cfg.geoid)` (e.g.
     `us_noaa_g2018u0.tif`), download from `cfg.geoid_cdn_base_url +
     filename`, validate Content-Length, write to `cfg.geoid` (creating parent
     dirs). Skip if file already exists (idempotent). Record download in the
     corpus-dir health file.

   - `ensure_vdatum`: if `cfg.vdatum_dir` is set and `cfg.vdatum_regions` is
     non-empty, for each region check for a marker file
     `<vdatum_dir>/.{region}_installed`; if absent, download
     `<vdatum_cdn_base_url>vdatum_{region}.zip`, validate Content-Length,
     extract only `*.gtx` files to `cfg.vdatum_dir`, write the marker. Uses
     `_safe_members`-style path validation before extraction.

   - Reuses `downloader._open_url` (scheme allow-list guard) and
     `downloader._copy_capped` (download-size cap).

2. **Update `enc_updater/config.py`** — add three optional keys to
   `UpdaterConfig` and `_TOP_LEVEL_KEYS`:
   - `vdatum_regions: List[str]` — default `[]`; which NOAA VDatum region
     bundles to provision (e.g. `['NewEngland']`)
   - `geoid_cdn_base_url: str` — default `'https://cdn.proj.org/'`
   - `vdatum_cdn_base_url: str` — default
     `'https://vdatum.noaa.gov/download/data/'`

3. **Update `enc_updater/__main__.py`** — call `datum_provisioner.ensure_geoid`
   and `datum_provisioner.ensure_vdatum` immediately after `load_config`, before
   `downloader.update_corpus`. Failure raises `UpdaterError` → exit 1 (same as
   download failure; the D7 export would fail anyway if grids are absent).

4. **Update `config/region_example.yaml`** — change datum paths to
   `~/data/world/datum/` layout; add `vdatum_regions` key:
   ```yaml
   geoid: ~/data/world/datum/geoid/us_noaa_g2018u0.tif
   vdatum_dir: ~/data/world/datum/vdatum
   vdatum_regions:
     - NewEngland
   ```

5. **Update `README.md`** — add a "Datum grid provisioning" section documenting
   that `enc_updater` auto-provisions geoid + VDatum grids on first run when
   `geoid`, `vdatum_dir`, and `vdatum_regions` are configured; note idempotency
   and the marker-file mechanism.

6. **Add `test/test_datum_provisioner.py`** — mock-HTTP tests using
   `monkeypatch` on `downloader._open_url` (same pattern as
   `test_downloader.py`):
   - Happy path: geoid provisioned from scratch, file appears at expected path
   - Happy path: VDatum region provisioned, `*.gtx` files extracted and marker
     written
   - Idempotency: no re-download if file/marker already present
   - Failed download: `UpdaterError` raised, no partial file left on disk
   - Content-Length mismatch: rejected, no file installed
   - Scheme guard: inherited from `_open_url` (document with a test or note
     that `test_downloader.py` already covers it)
   - VDatum zip missing gtx files: `UpdaterError`, no marker written
   - Zip-slip guard on VDatum extraction: `UpdaterError` on absolute / `..`
     member paths

## Files to Change

| File | Change |
|------|--------|
| `enc_updater/datum_provisioner.py` | New module — geoid + VDatum fetch |
| `enc_updater/config.py` | Add `vdatum_regions`, `geoid_cdn_base_url`, `vdatum_cdn_base_url` |
| `enc_updater/__main__.py` | Call provisioner before ENC download |
| `config/region_example.yaml` | Update datum paths; add `vdatum_regions` |
| `README.md` | Add "Datum grid provisioning" section |
| `test/test_datum_provisioner.py` | New mock-HTTP test module |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | `config/region_example.yaml` and `README.md` updated in same PR; `vdatum_regions` new config key documented |
| Test what breaks | Mock-HTTP tests cover idempotency, integrity, and partial-download recovery — the exact failure modes that matter in a cron context |
| Only what's needed | Three new optional config keys; no new required keys; provisioner is a single focused module |
| Enforce over document | Provisioner raises `UpdaterError` (exit 1) on failure rather than silently continuing with missing grids |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| uma ADR-0010 D3 (amended 2026-08-20, #288) | Yes | Target paths (`datum/geoid/`, `datum/vdatum/`) match the D3 amendment canonical layout |
| uma ADR-0010 D7 | Yes | Provisioning runs before the D7 regeneration cycle; grids are guaranteed present when `s57_to_geotiff` runs |

## Consequences

| If we change… | Also update… | Included in plan? |
|---|---|---|
| `config/region_example.yaml` geoid/vdatum paths | Existing host configs (gabby, salmon) — operators must update manually | No — field ops, out of scope |
| New `vdatum_regions` config key | `README.md` and `config/region_example.yaml` example | Yes — step 4 + 5 |
| `_TOP_LEVEL_KEYS` in `config.py` | `test_config.py` unknown-key rejection test | Yes — covered by adding keys to allowlist |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `README.md` lacks any mention of
  datum grid provisioning; `config/region_example.yaml` has stale `/opt/vdatum`
  paths — both updated in steps 4 and 5 above.
- **Agent-instruction candidates** (proposals only — operator decides): a note
  in `.agent/knowledge/` about the provisioner's marker-file idempotency
  mechanism (`.{region}_installed`) could help future agents understand why
  a populated `vdatum_dir` with no marker will re-provision — but this is a
  one-time edge case, not a recurring pattern. Not recommended unless it
  surfaces as a recurring confusion.

## Open Questions

- [ ] **Geoid fetch**: plan uses direct cdn.proj.org HTTP (no projsync
  dependency). Operator: confirm this is acceptable, or prefer the projsync CLI
  path? Projsync handles multi-grid dependencies automatically but requires the
  PROJ tools to be installed and internet-accessible.
- [ ] **Provision-failure behavior**: if a VDatum region download fails (network
  error) but no existing `.gtx` files are present, the plan exits 1. If grids
  already exist from a prior run, the provisioner is skipped (idempotent). Is
  "exit 1 on first-time failure, skip if already present" the right contract,
  or should the provisioner always verify grid integrity?

## Estimated Scope

Single PR.
