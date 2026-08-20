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
