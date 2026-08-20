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

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-20 15:46 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-37 at `9aa4fba`
**Mode**: pre-push
**Depth**: Deep (reason: remote download + zip extraction, security-relevant)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 2 | **Ship**: recommended — round-1 must-fix resolved and regression-tested; only two low-severity suggestions remain

### Findings
- [ ] No must-fix findings. Round-1 must-fix (guard failures not recorded to health file) is resolved: `ensure_geoid`/`ensure_vdatum` route guard `UpdaterError`s through `_record` (double-record-guarded by `_provision_recorded`), and tests assert `phase="provision"` lands in the health file — `datum_provisioner.py:115-117,179-181`; `test_datum_provisioner.py:166,245`
- [ ] (suggestion) Post-download fs ops (`_sha256_file`, `os.replace`, marker `open`) sit outside any `except`, so an `OSError` (disk failure, or geoid/vdatum_dir pointing at a directory) escapes uncaught and unrecorded — contradicts the "any failure raises UpdaterError and records it" contract; low probability, still fails loud — `enc_updater/enc_updater/datum_provisioner.py:144,149,228,232`
- [ ] (suggestion) VDatum `.gtx` install flattens via `os.path.basename`, so same-named grids across bundles/nested dirs silently overwrite via `os.replace` (contained within vdatum_dir; no path escape) — `enc_updater/enc_updater/datum_provisioner.py:227-228`

### Notes
- Static analysis: `ament_flake8` + `ament_pep257` clean; full suite 77/77 green (no network used).
- Claude Adversarial: 2 passes (Lens A logic, Lens B systemic/security) — no must-fix from either. Copilot: off (default). Local: skipped (no `local_review.sh` helper in this repo; Ollama not responding).

### Next step
Approved pre-push review. Lifecycle: Local Review → push / open PR → triage-reviews. The two suggestions are optional robustness hardening; neither blocks the push.

## Integrated Review
**Status**: complete
**When**: 2026-08-20 12:20 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #38 at `428711c`
**Sources**: 3 (Copilot R1 @ `428711c`, Local Review (Pre-Push) R1–R2 @ prior SHAs, CI rollup)
**Cross-source confirmations**: 0 strict (but both Copilot findings extend the round-2 "fs OSError bypasses the record-and-raise contract" family to new sites — same class, different lines/SHA)
**CI**: all-pass (hosted build-and-test green at head)

### Findings
- [x] (valid, Copilot) `ensure_geoid` treats any existing path as installed (`os.path.exists`); a `geoid` misconfigured to an existing directory silently skips provisioning and only fails later at export — use `os.path.isfile` and fail loud (record + UpdaterError) when the path exists but is not a regular file — `enc_updater/enc_updater/datum_provisioner.py:112`
- [x] (valid, Copilot) `tempfile.mkstemp`/`mkdtemp` can raise raw `OSError`, escaping unrecorded (bypasses `_fail`) — wrap the three sites in the established `_fail` pattern — `enc_updater/enc_updater/datum_provisioner.py:137,191,205`

### False positives
- (none)

**Local-timeline reconciliation**: pre-push R1 must-fix (guard recording) and
R2 suggestions (post-download fs ops, flatten justification) all resolved
before publish; Copilot found two *new* sites in the same contract family —
treat as the completing sweep of that class (add matching tests).

## Implementation
**Status**: complete
**When**: 2026-08-20 16:29 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-37 at `a79a32f`
**Addressed**: Integrated Review — 2026-08-20 12:20 -04:00 at `428711c` (PR #38)
**Commits**: `45bce88`, `a79a32f`

Completing sweep of the "raw OSError bypasses the record-and-raise contract"
class the pre-push rounds opened and Copilot extended — both Integrated Review
findings fixed, each in an atomic commit with matching mock tests. No findings
deferred.

### Actions
- [x] (valid, Copilot) `ensure_geoid` treated any existing path as installed (`os.path.exists`), so a geoid path pointing at a directory silently skipped provisioning and only failed later at export. Now trusts only a regular file (`os.path.isfile`); an existing non-regular-file path fails loud via `_fail` (recorded + `UpdaterError`) — `enc_updater/enc_updater/datum_provisioner.py:111-117` (commit `45bce88`)
- [x] (valid, Copilot) `tempfile.mkstemp` (geoid), `tempfile.mkdtemp` (vdatum), and `os.path.getsize` (vdatum zip) could raise raw `OSError` outside any handler, escaping `_fail` and going unrecorded. Wrapped all three in the established `except OSError → _fail` pattern so a filesystem failure is recorded to the health file (`phase="provision"`) and raised as `UpdaterError` — `enc_updater/enc_updater/datum_provisioner.py:136-138,193-197,205-208` (commit `a79a32f`)

### Tests
Added four mock-HTTP tests (same monkeypatch pattern as the suite):
`test_geoid_path_is_directory_fails_loud` (finding 1), and
`test_geoid_tempfile_failure_recorded`, `test_vdatum_tempdir_failure_recorded`,
`test_vdatum_zip_stat_failure_recorded` (finding 2) — each asserts the failure
lands in the health file with `phase="provision"`. Full package suite green:
**83 passed** (`pytest enc_updater/test/`), `ament_flake8` + `ament_pep257`
gate tests included. No network access used.

Incidental (disclosed): reformatted the pre-existing `_fail_replace_for`
docstring in `test_datum_provisioner.py` to D213 style (ament ignores D212 but
enforces D213 — summary on the second line) so the `ament_pep257` gate is green
alongside the new multi-line test docstrings; no behavior change.

### Next step
Lifecycle: Implementation → review-code (re-review the fixes). Hand off to a
fresh-context sub-agent to read the diff cold and confirm both findings are
genuinely resolved:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 37 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-20 16:38 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-37 at `34b754a`
**Mode**: pre-push
**Depth**: Deep (reason: new network-fetching module with integrity/SSRF/zip-slip/zip-bomb security surface)
**Must-fix**: 0 | **Suggestions**: 4
**Round**: 3 | **Ship**: recommended — both Integrated-Review (Copilot) fixes confirmed resolved and regression-tested; all gates green (83 passed, ament_flake8/pep257 clean); only defense-in-depth suggestions remain

### Findings
- [ ] (suggestion) Grid basename collision is silent — two bundles/members sharing a basename overwrite each other, markers still written — add a collision guard — `enc_updater/enc_updater/datum_provisioner.py:252`
- [ ] (suggestion) Zip-bomb cap (`_safe_members`) runs after `zf.testzip()` fully decompresses — reorder so the cap precedes decompression — `enc_updater/enc_updater/datum_provisioner.py:233`
- [ ] (suggestion) VDatum install not fully atomic — grids `os.replace`d one-by-one before marker; stage-and-move for full atomicity — `enc_updater/enc_updater/datum_provisioner.py:246`
- [ ] (suggestion) Existing geoid trusted without re-verification; changing geoid_sha256 has no effect on on-disk file — note operators must delete grid to re-verify — `enc_updater/enc_updater/datum_provisioner.py:113`

**Note**: Diffed against local `origin/jazzy` (`f17b2ce`); fetch failed offline, but history is linear so the diff is trustworthy. Copilot/Local adversarial off (offline host); coverage from two disjoint-lens Claude passes, which cross-confirmed finding 1.
