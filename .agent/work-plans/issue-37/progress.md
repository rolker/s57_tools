---
issue: 37
---

# Issue #37 — enc_updater: provision world/datum/ geoid and VDatum grids

## Issue Review
**Status**: complete
**When**: 2026-08-20 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #37
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Specify the VDatum region selection mechanism in the plan: the config currently has `vdatum_dir` (a directory path) but no region-identifier list; the provisioner needs to know *which* VDatum bundles to download. Plan should define the config keys (e.g. `vdatum_regions: [NewEngland, ...]`) consistent with the existing config schema and `config/region_example.yaml`.
- [ ] Define the test strategy for the new provisioning step: network-dependent downloads require mock-HTTP tests for idempotent behavior, integrity verification, and partial-download recovery — same rigour as `downloader.py`. Flag in plan.
- [ ] Decide and record the geoid fetch mechanism in the plan (projsync CLI vs. direct cdn.proj.org HTTP): projsync is cleaner but adds a CLI dependency; direct HTTP mirrors the existing `downloader.py` pattern. Both work offline from the field side. Record the choice rationale.
- [ ] Ensure the README and `config/region_example.yaml` are updated in the same PR to document the new provisioning entry point and any new config keys.

## Plan Authored
**Status**: complete
**When**: 2026-08-20 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-37/plan.md` at `f0bc2df`
**Branch**: feature/issue-37 at `f0bc2df`
**Phases**: single

### Open questions
- [ ] Geoid fetch mechanism: plan uses direct cdn.proj.org HTTP (no projsync dependency). Operator: confirm acceptable, or prefer projsync CLI?
- [ ] Provision-failure behavior: exit 1 on first-time download failure (no existing grids), skip if already present — is always-verify-integrity preferred instead?

## Plan Review
**Status**: complete
**When**: 2026-08-20 14:59 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-37/plan.md` at `f0bc2df`
**PR**: PR-less (`--issue` mode)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) File targeting — the Files table lists `README.md` and `config/region_example.yaml`, but both an `enc_updater/README.md` and a top-level `s57_tools/README.md` exist. Name the targets explicitly: `enc_updater/README.md` and `enc_updater/config/region_example.yaml` — `plan.md:72,100-101`
- [ ] (suggestion) Geoid integrity is overstated as parity with the ENC path. `downloader.py` validates cell zips against the catalog-*declared* size (an independent, trusted source) plus a zip CRC pass; a cdn.proj.org `.tif` has no independent declared size and no self-CRC, so a Content-Length check catches truncation only, not a wrong/corrupt file. Pin an expected SHA-256 (PROJ publishes per-file checksums) or explicitly document the weaker guarantee — `plan.md:20-23,36-38`
- [ ] (suggestion) Make the geoid write atomic — the approach relies on idempotency-by-presence ("skip if file exists"), but a partial file from an interrupted run would then be treated as complete. Specify download-to-temp + rename (mirrors `downloader._install_cell`) — `plan.md:36-38,83`
- [ ] (suggestion) Verify the concrete VDatum download URL and region-bundle naming before implementation. `https://vdatum.noaa.gov/download/data/vdatum_{region}.zip` and `NewEngland` are asserted but unverified; NOAA VDatum regional bundles use specific codes and the URL scheme may differ. The plan flags the *geoid* mechanism as an operator checkpoint but treats the VDatum URL as settled — fold it into the same checkpoint / Open Questions — `plan.md:44,67-70`
- [ ] (suggestion) Map health recording to the existing API — the plan says "Record download in the corpus-dir health file", but `health.py` exposes `record_download_attempt/ok`, `record_regen_ok`, and `record_error(corpus_dir, phase, message)`; there is no generic record-download call. Use `record_error(cfg.corpus_dir, 'provision', ...)` on failure and decide whether success needs a record at all — `plan.md:38`

## Implementation
**Status**: complete
**When**: 2026-08-20 00:00 +00:00
**By**: Claude Opus

**Branch**: feature/issue-37 at `6d95033`
**Plan**: `.agent/work-plans/issue-37/plan.md` (followed as written; no divergence — no plan edits needed)

### What was done
Implemented datum grid provisioning for `enc_updater` per the settled plan, in
three atomic commits under the agent identity:

- `c7942e3` — **config**: four optional keys on `UpdaterConfig` + allowlist:
  `geoid_sha256`, `geoid_cdn_base_url` (default `https://cdn.proj.org/`),
  `vdatum_bundles` (validated list of non-empty names), `vdatum_cdn_base_url`
  (default `https://vdatum.noaa.gov/download/data/`).
- `b06637c` — **module + wiring + tests**: new `enc_updater/datum_provisioner.py`
  with `ensure_geoid(cfg)` / `ensure_vdatum(cfg)`, called from `__main__` right
  after `load_config` and before `downloader.update_corpus`. geoid = direct
  cdn.proj.org HTTP → temp file in dest dir → SHA-256 verify → atomic
  `os.replace`; unset pin while active is a hard error. VDatum = verbatim bundle
  names, `.{bundle}_installed` marker (written last), Content-Length + zip CRC +
  `_safe_members` zip-slip/bomb guards, only `*.gtx` extracted. Both reuse
  `downloader._open_url` / `_copy_capped` and `health.record_error(cfg.corpus_dir,
  'provision', ...)` on failure, raising `UpdaterError` (exit 1, previous layer
  intact).
- `6d95033` — **docs**: README "Datum grid provisioning" section;
  `region_example.yaml` moved to `~/data/world/datum/` layout with the
  host-verified pin `us_noaa_g2018u0.tif =
  fa9a407ac7ee3f5a3694008e4bcd09ce9cc250452f0c3b11700a4960340abce2` and
  `vdatum_bundles: [MENHMAgome23_8301]`.

### Tests
`enc_updater/test/test_datum_provisioner.py` — 15 mock-HTTP tests (monkeypatch
`downloader._open_url`, same pattern as `test_downloader.py`): geoid happy path,
idempotency, unset-config no-op, failed download (no partial left), SHA mismatch,
unset pin fails loud, non-http scheme guard; VDatum happy path (only `*.gtx`
installed + marker), marker idempotency, unset no-op, Content-Length mismatch,
missing-gtx, failed download (no marker), zip-slip, corrupt zip.

Full suite green: **72 passed** (`pytest enc_updater/test/`), including the
`flake8` and `pep257` lint gates over the new module and tests. No network
access used; the SHA pin was provided host-verified.

### Next step
Ready for code review (`review-code`). No follow-ups outstanding; the
CMake-download removal in `mru_transform` (uma#288 item 6) remains gated on
gabby+salmon deploy logs showing `world/datum/` population — out of scope here.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-20 15:30 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-37 at `4b5bed9`
**Mode**: pre-push
**Depth**: Deep (reason: remote download + archive extraction, security-relevant)
**Must-fix**: 1 | **Suggestions**: 2
**Round**: 1 | **Ship**: continue — one mechanical must-fix; otherwise shippable

### Findings
- [x] (must-fix) Scheme/size-cap/zip-slip/zip-bomb failures raise UpdaterError but sit under `except OSError` / `except zipfile.BadZipFile`, so they bypass `_fail()` and record no `last_error.phase="provision"` — contradicts README/docstring "any provisioning failure is recorded" — `enc_updater/enc_updater/datum_provisioner.py:115,165,191`
- [x] (suggestion) Validate VDatum bundle-name shape (reject `/` `\` `..`); a slash yields an uncaught FileNotFoundError instead of a clean UpdaterError — `enc_updater/enc_updater/config.py:136`
- [x] (suggestion) `_content_length` return hint should be `Optional[int]` (returns None when header absent) — `enc_updater/enc_updater/datum_provisioner.py:67`

## Implementation
**Status**: complete
**When**: 2026-08-20 15:37 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-37 at `726dae5`
**Addressed**: Local Review (Pre-Push) — 2026-08-20 15:30 +00:00 at `4b5bed9`
**Commits**: `a406ea9`, `726dae5`

### Actions
- [x] (must-fix) Guard failures (scheme allow-list, size cap, zip-slip/zip-bomb) raised `UpdaterError` past the `except OSError`/`except zipfile.BadZipFile` handlers and so were never recorded to the health file. Wrapped each public entry (`ensure_geoid`, `ensure_vdatum`) in an `except UpdaterError` that routes through a new `_record()`; a `_provision_recorded` sentinel set by `_fail()` keeps already-logged errors from being double-recorded — `enc_updater/enc_updater/datum_provisioner.py` (commit `a406ea9`)
- [x] (suggestion) `_content_length` return type annotated `Optional[int]` (with `from typing import Optional`) — `enc_updater/enc_updater/datum_provisioner.py:67` (commit `a406ea9`)
- [x] (suggestion) VDatum bundle-name shape validated at config load — a `/`, `\`, or `..` now fails as a clean `UpdaterError` instead of an opaque `FileNotFoundError` — `enc_updater/enc_updater/config.py` (commit `726dae5`)

### Tests
Extended coverage: `test_geoid_rejects_non_http_scheme` and
`test_vdatum_zip_slip_rejected` now assert the guard failure lands in the
health file with `phase="provision"`; new `test_config.py` cases cover
rejected path-shaped bundle names and an accepted single-segment name. Full
package suite green: **77 passed** (`pytest enc_updater/test/`), flake8 +
pep257 gates included. No network access used.

### Next step
Ready for re-review (`review-code`, pre-push) of the fix commits — a
fresh-context sub-agent reads the diff cold and confirms the three findings
are genuinely resolved. No findings deferred.
