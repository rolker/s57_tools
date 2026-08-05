---
issue: 28
---

# Issue #28 — Chart updater: keep ENC corpus current and regenerate chart layer on change

## Issue Review
**Status**: complete
**When**: 2026-08-04 03:06 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #28
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [x] Specify the nav-down liveness signal contract in the plan: resolved by operator checkpoint — ROS graph probe (`ros2 node list` in a subprocess after sourcing ROS env).
- [x] Specify the cross-repo API invocation mechanism: resolved by operator checkpoint — CLI subprocess (`import_geotiff --stage/--commit`).
- [ ] Verify issue #5 (auto-download & auto-update NOAA ENC data) is still open and add a `Closes #5` keyword to this PR so it doesn't linger as a ghost issue after the work lands.
- [x] Confirm issue #27 (`s57_to_geotiff`) is merged and available on the branch before starting implementation — confirmed: PR #29 merged 2026-07-31, present on branch base.

## Plan Authored
**Status**: complete
**When**: 2026-08-04 03:12 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-28/plan.md` at `b86543a`
**Branch**: feature/issue-28 at `b86543a`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-08-04 03:17 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-28/plan.md` at `b86543a`
**PR**: PR-less (`--issue 28`; repo remote is `gitcloud`, `gh` unauthenticated — issue body not fetched, evaluation used the recorded Issue Review entry above)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Staged dir must be created on the **same filesystem** as `store_dir`; `import_geotiff --commit`'s atomic `rename(2)` returns `EXDEV` on a cross-device staged dir and cannot commit. Step 4a's "fresh temp staged dir" invites `/tmp` (often tmpfs/separate device). Create it adjacent to the store (e.g. `<store_dir>/../.staging.<pid>`). — `plan.md:47`
- [ ] (must-fix) `import_geotiff --commit`'s target must be a **scratch store**, not a store a live costmap reads, until `unh_marine_autonomy#276` lands (ADR-0010 D7 precondition: `bathymetry_layer` renders high-uncertainty cells LETHAL under `unsurveyed_is_lethal`). The plan's premise ("keep the live chart layer current") silently conflicts with this — `region_example.yaml`/README must state the configured store must not be a live-costmap store yet, and cron deployment is gated on uma#276. — `plan.md:58`, `plan.md:69`
- [ ] (suggestion) "ADR-0010 D7" is the **unh_marine_autonomy** project ADR-0010 (per `marine_bathymetry_store/README.md`), which collides with the **workspace** ADR-0010 (git-bug). Disambiguate the citation so a reviewer doesn't look in `docs/decisions/` and find the wrong ADR. — `plan.md:123`
- [ ] (suggestion) ADR-0009: commit to stdlib `urllib` (the plan's own fallback) to avoid a runtime dep, or if `requests` is used, declare `python3-requests` via rosdep in `package.xml` — bare pip is forbidden. — `plan.md:31`
- [ ] (suggestion) `import_geotiff --stage` omits `--level`; it defaults from `--cell-size`. Consider pinning GGGS `--level`/`--cell-size` in config for consistent chart-tile resolution across cells. — `plan.md:58`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-05 17:00 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-28 at `dbb7432`
**Mode**: pre-push
**Depth**: Deep (reason: 1909-line new package, cross-layer subprocess orchestration + network download + safety interlock)
**Must-fix**: 1 | **Suggestions**: 5
**Round**: 1 | **Ship**: continue — one safety-contract must-fix (interlock silent fail-open documentation); otherwise clean, should converge in one round

### Findings
- [ ] (must-fix) Nav-down interlock can silently FAIL OPEN if the cron/probe environment's `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` differ from the live nav stack's — `ros2 node list` queries the wrong DDS domain, returns empty, and the swap proceeds while nav is active; fail-closed only covers probe *errors*, not a blind-but-successful empty probe. Document the env-alignment requirement (README + region_example.yaml) and ideally pin `ROS_DOMAIN_ID` in config for the probe. — `enc_updater/nav_liveness.py:24`
- [ ] (suggestion) Harden the probe invocation: pass `ros_setup` as a bash positional arg (`bash -c 'source "$1" ... && ros2 node list' _ "$path"`) instead of f-string interpolation into `bash -c`, so an unusual path can't be mis-executed. — `enc_updater/nav_liveness.py:27`
- [ ] (suggestion) Overlapping-run protection is best-effort (PID-named work dirs + existence check); a stray overlapping cron invocation could double-commit. Consider a lockfile on `store_dir` and/or document "runs must not overlap". — `enc_updater/regenerator.py:149`
- [ ] (suggestion) `_install_cell` recovery: if restoring the backup also fails, the old cell data is left in a `.old.<cell>.*` dir with the canonical cell dir missing until the next run re-downloads; surface the backup location in the raised error. — `enc_updater/downloader.py:172`
- [ ] (suggestion) Catalog/download hardening from an external host: parse the catalog with entity-expansion protection (or cap the response size) and cap download/extract size against a zip bomb. Low risk (HTTPS + NOAA source), defense-in-depth. — `enc_updater/downloader.py:53`
- [ ] (suggestion) A catalog `<cell>` missing `zipfile_size` silently skips the byte-count check (CRC still runs); log when size is absent so the degraded integrity check is visible. — `enc_updater/downloader.py:130`
