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
