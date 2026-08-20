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

**Geoid fetch mechanism (settled at the 2026-08-20 plan checkpoint)**: direct
cdn.proj.org HTTP **plus a pinned SHA-256** — mirrors the `downloader.py`
pattern (no subprocess / projsync dependency, scheme allow-listed, download-cap
guards) and exceeds projsync on integrity (projsync does freshness/size
bookkeeping, not cryptographic verification; for a single known file it adds
nothing). If a future need for many PROJ grids appears, the `ensure_geoid`
contract stays stable and the fetch internals can swap to projsync then.

**VDatum URL scheme (host-verified 2026-08-20)**: the originally asserted
`vdatum_{region}.zip` scheme **404s**. NOAA serves per-area bundles at
`https://vdatum.noaa.gov/download/data/<BundleName>.zip` with version-suffixed
names (verified live: `MENHMAgome23_8301.zip` = ME/NH/MA Gulf of Maine,
`DEdelbay33_8301.zip` = Delaware Bay). Config therefore takes **verbatim bundle
names** (operator decision) — no friendly-name mapping table to go stale on
NOAA version bumps.

## Approach

1. **Add `enc_updater/datum_provisioner.py`** — new module with two public
   functions: `ensure_geoid(cfg)` and `ensure_vdatum(cfg)`. Both are
   idempotent (check presence before downloading), follow the same
   HTTP/integrity pattern as `downloader.py`, and raise `UpdaterError` on
   failure so callers see a clean error message.

   - `ensure_geoid`: if `cfg.geoid` is set and the file is absent, derive the
     CDN filename from `os.path.basename(cfg.geoid)` (e.g.
     `us_noaa_g2018u0.tif`), download from `cfg.geoid_cdn_base_url + filename`
     **to a temp file in the destination directory, verify its SHA-256 against
     `cfg.geoid_sha256`, then atomically rename** to `cfg.geoid` (creating
     parent dirs) — mirrors `downloader._install_cell`; a partial file from an
     interrupted run can never masquerade as complete. Skip if file already
     exists (idempotent-by-presence — trustworthy because writes are atomic).
     On failure call `health.record_error(cfg.corpus_dir, 'provision', ...)`
     (the existing API — there is no generic record-download call; success
     needs no health record) and raise `UpdaterError`.
     The pinned SHA-256 ships in `enc_updater/config/region_example.yaml`;
     **obtain it from PROJ's published per-file checksums (proj-data repo
     metadata) and cross-check against an independent fresh download — never
     hand-type it.**

   - `ensure_vdatum`: if `cfg.vdatum_dir` is set and `cfg.vdatum_bundles` is
     non-empty, for each **verbatim bundle name** (e.g. `MENHMAgome23_8301`)
     check for a marker file `<vdatum_dir>/.{bundle}_installed`; if absent,
     download `<vdatum_cdn_base_url>{bundle}.zip`, validate Content-Length +
     zip CRC pass, extract only `*.gtx` files to `cfg.vdatum_dir`, write the
     marker last. Uses `_safe_members`-style path validation before extraction.
     Failure: `health.record_error(cfg.corpus_dir, 'provision', ...)` +
     `UpdaterError`, no marker written.

   - Reuses `downloader._open_url` (scheme allow-list guard) and
     `downloader._copy_capped` (download-size cap).

2. **Update `enc_updater/config.py`** — add four optional keys to
   `UpdaterConfig` and `_TOP_LEVEL_KEYS`:
   - `vdatum_bundles: List[str]` — default `[]`; verbatim NOAA VDatum bundle
     names to provision (e.g. `['MENHMAgome23_8301']`)
   - `geoid_sha256: str` — default `None`; expected SHA-256 of the geoid file
     (required when `geoid` provisioning is active — fail loud if unset rather
     than silently skipping verification)
   - `geoid_cdn_base_url: str` — default `'https://cdn.proj.org/'`
   - `vdatum_cdn_base_url: str` — default
     `'https://vdatum.noaa.gov/download/data/'`

3. **Update `enc_updater/__main__.py`** — call `datum_provisioner.ensure_geoid`
   and `datum_provisioner.ensure_vdatum` immediately after `load_config`, before
   `downloader.update_corpus`. Failure raises `UpdaterError` → exit 1 (same as
   download failure; the D7 export would fail anyway if grids are absent).

4. **Update `enc_updater/config/region_example.yaml`** — change datum paths to
   `~/data/world/datum/` layout; add the new keys:
   ```yaml
   geoid: ~/data/world/datum/geoid/us_noaa_g2018u0.tif
   geoid_sha256: <pin obtained per step 1 — never hand-typed>
   vdatum_dir: ~/data/world/datum/vdatum
   vdatum_bundles:
     - MENHMAgome23_8301   # ME/NH/MA Gulf of Maine (verified live 2026-08-20)
   ```

5. **Update `enc_updater/README.md`** — add a "Datum grid provisioning" section
   documenting that `enc_updater` auto-provisions geoid + VDatum grids on first
   run when `geoid`+`geoid_sha256`, `vdatum_dir`, and `vdatum_bundles` are
   configured; note idempotency, atomic writes, the marker-file mechanism, and
   where to find bundle names (vdatum.noaa.gov download page, verbatim).

6. **Add `enc_updater/test/test_datum_provisioner.py`** — mock-HTTP tests using
   `monkeypatch` on `downloader._open_url` (same pattern as
   `test_downloader.py`):
   - Happy path: geoid provisioned from scratch, file appears at expected path
   - Happy path: VDatum region provisioned, `*.gtx` files extracted and marker
     written
   - Idempotency: no re-download if file/marker already present
   - Failed download: `UpdaterError` raised, no partial file left on disk
     (atomic temp+rename: destination never exists in a partial state)
   - SHA-256 mismatch (geoid): rejected, temp file removed, nothing installed
   - `geoid_sha256` unset while geoid provisioning active: fail loud
   - Content-Length mismatch (vdatum): rejected, no file installed
   - Scheme guard: inherited from `_open_url` (document with a test or note
     that `test_downloader.py` already covers it)
   - VDatum zip missing gtx files: `UpdaterError`, no marker written
   - Zip-slip guard on VDatum extraction: `UpdaterError` on absolute / `..`
     member paths

## Files to Change

| File | Change |
|------|--------|
| `enc_updater/enc_updater/datum_provisioner.py` | New module — geoid + VDatum fetch (atomic writes, SHA-256 geoid pin) |
| `enc_updater/enc_updater/config.py` | Add `vdatum_bundles`, `geoid_sha256`, `geoid_cdn_base_url`, `vdatum_cdn_base_url` |
| `enc_updater/enc_updater/__main__.py` | Call provisioner before ENC download |
| `enc_updater/config/region_example.yaml` | Update datum paths; add `vdatum_bundles` + `geoid_sha256` |
| `enc_updater/README.md` | Add "Datum grid provisioning" section |
| `enc_updater/test/test_datum_provisioner.py` | New mock-HTTP test module |

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

- [x] ~~**Geoid fetch**~~ — **settled 2026-08-20 (operator)**: direct HTTP +
  pinned SHA-256 (option 1). projsync adds nothing for a single known file
  (same CDN, no cryptographic verification) and would double the code paths;
  revisit only if a multi-grid need appears.
- [x] ~~**Provision-failure behavior**~~ — **settled 2026-08-20 (operator)**:
  exit 1 on first-time failure; skip when present; atomic temp+rename writes
  make presence a trustworthy completeness signal. No per-cycle re-hash of
  existing grids.

## Estimated Scope

Single PR.
