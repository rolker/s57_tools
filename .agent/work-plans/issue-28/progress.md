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
- [x] (must-fix) Nav-down interlock can silently FAIL OPEN if the cron/probe environment's `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` differ from the live nav stack's — `ros2 node list` queries the wrong DDS domain, returns empty, and the swap proceeds while nav is active; fail-closed only covers probe *errors*, not a blind-but-successful empty probe. Document the env-alignment requirement (README + region_example.yaml) and ideally pin `ROS_DOMAIN_ID` in config for the probe. — `enc_updater/nav_liveness.py:24`
- [x] (suggestion) Harden the probe invocation: pass `ros_setup` as a bash positional arg (`bash -c 'source "$1" ... && ros2 node list' _ "$path"`) instead of f-string interpolation into `bash -c`, so an unusual path can't be mis-executed. — `enc_updater/nav_liveness.py:27`
- [x] (suggestion) Overlapping-run protection is best-effort (PID-named work dirs + existence check); a stray overlapping cron invocation could double-commit. Consider a lockfile on `store_dir` and/or document "runs must not overlap". — `enc_updater/regenerator.py:149`
- [x] (suggestion) `_install_cell` recovery: if restoring the backup also fails, the old cell data is left in a `.old.<cell>.*` dir with the canonical cell dir missing until the next run re-downloads; surface the backup location in the raised error. — `enc_updater/downloader.py:172`
- [x] (suggestion) Catalog/download hardening from an external host: parse the catalog with entity-expansion protection (or cap the response size) and cap download/extract size against a zip bomb. Low risk (HTTPS + NOAA source), defense-in-depth. — `enc_updater/downloader.py:53`
- [x] (suggestion) A catalog `<cell>` missing `zipfile_size` silently skips the byte-count check (CRC still runs); log when size is absent so the degraded integrity check is visible. — `enc_updater/downloader.py:130`

## Implementation
**Status**: complete
**When**: 2026-08-05 17:16 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-28 at `5713d5a`
**Addressed**: Local Review (Pre-Push), When 2026-08-05 17:00 +00:00 / SHA `dbb7432` (1 must-fix + 5 suggestions, all open)
**Commits**: `caa2555` `40eb99b` `28a059f` `df4270a` `67c7070` `5713d5a`

Every finding fixed with a real change (no deferrals). One atomic commit per
finding; each touched file re-linted (ament_flake8 + ament_pep257) and the full
package suite re-run — **36 passed** (24 → 36 with the added regression tests).

### Actions
- [x] (must-fix) Nav-interlock fail-open on DDS-domain mismatch — added `nav_liveness.ros_domain_id`, exported into the probe subprocess env to pin the query to the nav stack's domain; documented the `ROS_DOMAIN_ID` + `RMW_IMPLEMENTATION` env-alignment requirement in README + region_example.yaml + module docstring — `enc_updater/enc_updater/nav_liveness.py:24`, `config.py`, `README.md`, `config/region_example.yaml` (`caa2555`)
- [x] (suggestion) Harden probe invocation — `ros_setup` now passed as a bash positional arg (`$1`) instead of f-string interpolation; verified real-bash sourcing + updated `test_ros_setup_wraps_probe_in_bash` — `enc_updater/enc_updater/nav_liveness.py:27` (`40eb99b`)
- [x] (suggestion) Overlapping-run protection — added an exclusive non-blocking `flock` on `<store parent>/.enc_updater.lock` around the whole `regenerate` cycle; a second run refuses with "runs must not overlap"; documented in README troubleshooting; new `test_overlapping_run_refuses` — `enc_updater/enc_updater/regenerator.py:149` (`28a059f`)
- [x] (suggestion) `_install_cell` rollback — if the restore rename also fails, the raised error now names the preserved `.old.<cell>.*` backup dir and the manual restore path; new `test_install_double_failure_names_backup` — `enc_updater/enc_updater/downloader.py:172` (`df4270a`)
- [x] (suggestion) Catalog/download hardening — capped the catalog response read + reject any DTD/entity declaration (billion-laughs guard), byte-capped cell-zip streaming, and capped total uncompressed extract size (zip-bomb guard); new oversized-catalog / DTD / zip-bomb tests — `enc_updater/enc_updater/downloader.py:53` (`67c7070`)
- [x] (suggestion) Missing `zipfile_size` — logs a line naming the cell when the byte-count check is skipped (CRC still enforced); new `test_missing_zipfile_size_logs_and_installs` — `enc_updater/enc_updater/downloader.py:130` (`5713d5a`)

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to a fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 28 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-05 17:26 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-28 at `42e60a5`
**Mode**: pre-push
**Depth**: Deep (reason: 2228-line new package; network download + zip extraction + cross-layer subprocess orchestration + nav-safety interlock)
**Must-fix**: 2 | **Suggestions**: 6
**Round**: 2 | **Ship**: continue — Round-1 interlock must-fix confirmed fixed; 2 new must-fixes are mechanical (~1-2 lines each), expect convergence in one address-findings round
**Static analysis**: flake8 clean; 34 functional tests pass (36 with ament linter gates)
**Claude Adversarial**: 2 passes (Lens A logic + Lens B systemic/safety); Copilot off (default); Local skipped (Ollama unreachable)

### Findings
- [x] (must-fix) Bare `float()` coercions (`timeout`, `lake_datum`, `cell_size`, `*_timeout`) escape `load_config` as an uncaught `ValueError` — crashes with a traceback instead of the documented clean exit 1; confirmed by running it — `enc_updater/enc_updater/config.py:135,150,156`
- [x] (must-fix) No URL scheme allow-list on catalog `zipfile_location` before `urllib.urlopen` — a spoofed catalog can use `file://`/SSRF; add `urlparse` + http/https check — `enc_updater/enc_updater/downloader.py:47,162`
- [x] (suggestion) Interlock fail-open not enforced: `nodes` set + `ros_domain_id` omitted is accepted; warn (or require) when nodes non-empty — `enc_updater/enc_updater/nav_liveness.py:41`
- [x] (suggestion) Document the probe→commit TOCTOU window in the README nav-liveness contract (nav can come up during the ~120s commit) — `enc_updater/enc_updater/regenerator.py:207`
- [x] (suggestion) `registry.py` docstring mis-describes `replaceChartLayer` (renames whole staged dir; does not filter non-.tif) — `enc_updater/enc_updater/registry.py:10`
- [x] (suggestion) All-nodata tile makes `ComputeRasterMinMax` raise → aborts swap as "corrupt"; skip-not-fail or document the assumption — `enc_updater/enc_updater/regenerator.py:88`
- [x] (suggestion) Release the `gdal.Open` handle explicitly (`dataset = None`) in the spot-check loop — `enc_updater/enc_updater/regenerator.py:88`
- [x] (suggestion) `flock` is same-host advisory only; document the single-host / no-shared-NFS-store assumption — `enc_updater/enc_updater/regenerator.py:141`

## Implementation
**Status**: complete
**When**: 2026-08-05 17:37 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-28 at `5ac9c69`
**Addressed**: Local Review (Pre-Push), When 2026-08-05 17:26 +00:00 / SHA `42e60a5` (2 must-fix + 6 suggestions, all open)
**Commits**: `24c40cf` `f2ed34e` `94dc990` `12d88a2` `efc11d6` `5ac9c69`

Every finding fixed with a real change (no deferrals). One atomic commit per
finding (the two `_band1_min_max` findings share a commit — same 4-line
function). Each touched module re-linted (ament_flake8 + ament_pep257) and the
full package suite re-run — **57 passed** (was 44; +13 regression tests across
config-coercion, URL-scheme, domain-warning, and real-GDAL all-nodata cases).

### Actions
- [x] (must-fix) Non-numeric numeric config fields — added `_require_float`; every `float()` coercion (`nav_liveness.timeout`, `lake_datum`, `cell_size`, `download/export/stage/commit_timeout`) now routes through it and raises `UpdaterError` (clean exit 1) instead of an uncaught `ValueError`; new `test_config.py` — `enc_updater/enc_updater/config.py:82` (`24c40cf`)
- [x] (must-fix) URL-scheme allow-list — `_open_url` now rejects any non-http(s) scheme (`file://`/SSRF/LFI) via `urllib.parse.urlparse` before `urlopen`, guarding the untrusted catalog `zipfile_location`; new `_open_url` scheme tests — `enc_updater/enc_updater/downloader.py:47` (`f2ed34e`)
- [x] (suggestion) Interlock domain-pin warning — `check_nav_down` now warns loudly when `nav_liveness.nodes` is set but `ros_domain_id` is unpinned (the fail-open condition); new warn/no-warn tests — `enc_updater/enc_updater/nav_liveness.py:65` (`94dc990`)
- [x] (suggestion) All-nodata tile — `_band1_min_max` returns `None` (empty grid, skipped by the range check) when `ComputeRasterMinMax` reports no valid pixels, distinguished from a genuinely unreadable tile (still `UpdaterError`); real-GDAL `test_sanity.py` — `enc_updater/enc_updater/regenerator.py:88` (`12d88a2`)
- [x] (suggestion) Release GDAL handle — `_band1_min_max` sets `dataset = None` in a `finally` rather than waiting on GC — `enc_updater/enc_updater/regenerator.py:88` (`12d88a2`)
- [x] (suggestion) `registry.py` docstring — rewritten to state `replaceChartLayer` renames the whole staged `chart/` dir (so `editions.json` rides along); it only *validates* `.tif` tiles, it does not filter non-`.tif` out of the swap — `enc_updater/enc_updater/registry.py:10` (`efc11d6`)
- [x] (suggestion) Probe→commit TOCTOU — documented the point-in-time nature of the interlock and the ~120 s commit window in the README nav-liveness contract — `enc_updater/README.md` (`5ac9c69`)
- [x] (suggestion) `flock` scope — documented the same-host advisory-lock / single-host-per-store (no shared-NFS) assumption in the README troubleshooting entry and the `_store_lock` docstring — `enc_updater/README.md`, `enc_updater/enc_updater/regenerator.py:141` (`5ac9c69`)

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to a fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 28 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-05 17:47 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-28 at `85a8df5`
**Mode**: pre-push
**Depth**: Deep (reason: ~2500-line new package; untrusted network download + zip extraction + cross-layer subprocess orchestration + nav-safety interlock)
**Must-fix**: 0 | **Suggestions**: 8
**Round**: 3 | **Ship**: recommended — all three prior-round must-fixes confirmed fixed; both adversarial must-fix claims dismissed on source verification (editions.json IS delivered by replaceChartLayer's whole-dir rename; interlock/SSRF claims are already-mitigated tradeoffs / narrow defense-in-depth). No must-fix survives.
**Static analysis**: ament_flake8 clean; ament_pep257 clean; 57/57 tests pass (incl. real-GDAL sanity)
**Claude Adversarial**: 2 passes (Lens A logic + Lens B systemic/safety); Copilot off (default); Local skipped (Ollama unreachable)

### Findings
- [ ] (suggestion) Interlock still fails OPEN on RMW_IMPLEMENTATION mismatch (no config key) and if ros_domain_id left unpinned; currently warn+doc only — consider requiring ros_domain_id when nodes set and adding an rmw_implementation key exported into the probe env — `enc_updater/enc_updater/nav_liveness.py:69`
- [ ] (suggestion) `_open_url` scheme allow-list checks only the initial URL; urlopen follows redirects (http/https/ftp permitted) so a spoofed catalog could 302→internal http SSRF or ftp:// (file:// stays blocked) — re-check scheme per redirect hop or disable redirects — `enc_updater/enc_updater/downloader.py:57`
- [ ] (suggestion) Leftover PID work-dir after a hard crash fails every subsequent run (exit 1) until manual cleanup; with the flock now guaranteeing no concurrent run, reclaim the stale dir under the held lock instead of refusing — `enc_updater/enc_updater/regenerator.py:210`
- [ ] (suggestion) Sanity spot-check samples sorted(tiles)[:5] — alphabetically-first tiles cluster in one coordinate corner; a localized corruption elsewhere passes — stride across the sorted list — `enc_updater/enc_updater/regenerator.py:139`
- [ ] (suggestion) Removing a cell from config.cells prunes neither its manifest entry nor its corpus dir, so it silently stays in the chart (still converges); prune manifest entries absent from cells or document manual cleanup — `enc_updater/enc_updater/downloader.py:274`
- [ ] (suggestion) Config-load failure returns exit 1 before any health record is written, so persistent config breakage is invisible in .updater_health.json — record a config-phase error — `enc_updater/enc_updater/__main__.py:59`
- [ ] (suggestion) `_band1_min_max` distinguishes empty-vs-corrupt via the GDAL English substring 'no valid pixels'; a future GDAL rewording would fail all-nodata tiles as corrupt (fail-closed; a real-GDAL test would catch it) — `enc_updater/enc_updater/regenerator.py:108`
- [ ] (suggestion) Billion-laughs guard is a raw-bytes substring scan; a UTF-16 payload could evade it while expat still expands entities — prefer defusedxml or an expat entity-rejection handler — `enc_updater/enc_updater/downloader.py:76`

## Integrated Review
**Status**: complete
**When**: 2026-08-05 14:19 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #33 at `2dfc43c`
**Sources**: 3 (Copilot R1 @ `2dfc43c`, Local Review (Pre-Push) R3 @ `85a8df5`, CI rollup @ `2dfc43c`)
**Cross-source confirmations**: 0
**CI**: all-pass (build-and-test: success; copilot-pull-request-reviewer: success)

### Findings
- [ ] (low, Copilot) PR #33 body step 4 states "Empty `nodes` list = unconfigured (refuse)", but the implementation, module docstring, README nav-liveness contract, and `region_example.yaml` all define an empty/omitted `nav_liveness.nodes` as *probe disabled (skip)* — intentional for dev hosts with no nav stack. The code is right; the PR description is wrong. Fix by editing the PR body sentence to "Empty `nodes` list = interlock not configured (probe skipped; dev hosts only)". No code change. — `enc_updater/enc_updater/nav_liveness.py:65`
- [ ] (suggestion, Copilot-adjacent / overlaps R3 finding 1) Defense-in-depth only: `_band1_min_max` could add explicit `if dataset is None` / `if band is None` guards raising `UpdaterError`, covering a hypothetical GDAL build that returns `None` from `Open` without emitting a CPLError. Not reachable on the deployed GDAL (see false positive below). — `enc_updater/enc_updater/regenerator.py:103`

### False positives
- (Copilot) "`gdal.Open(path)` can return `None`, so `dataset.GetRasterBand(1)` raises `AttributeError` that escapes the documented `UpdaterError` contract" — `_band1_min_max` calls `gdal.UseExceptions()` before `Open`, so open/format failures raise `RuntimeError`, which the function's `except RuntimeError` converts to `UpdaterError`. Verified empirically on the deployed GDAL 3.8.4 (jazzy): unrecognized-format file, missing path, and a directory path all raise `RuntimeError`, never return `None`. The behavior is also locked by a real-GDAL regression test, `test/test_sanity.py::test_band1_min_max_raises_on_unreadable_tile`, which asserts `UpdaterError` on a junk tile and passes. The `AttributeError` path the comment describes cannot be produced.

### Notes
- No human reviewer comments and no conversation comments on the PR.
- The R3 `## Local Review (Pre-Push)` suggestions at `85a8df5` remain open by design — recorded in the PR body as non-blocking follow-up candidates; none is a must-fix and none was independently raised by Copilot.
- No cross-source confirmation: Copilot's two comments are disjoint from the eight open R3 suggestions.

### Next step
Lifecycle: **Integrated Review** → PR-body correction (no code findings) → merge.
No must-fix and no cross-confirmed findings remain. The single actionable item is a
one-sentence PR-description edit (host-side `gh pr edit`); it needs no commit, no
re-review, and does not gate merge on code grounds. After the body is corrected the PR
is ready for `.agent/scripts/merge_pr.sh --issue 28` (merge commit, not squash) —
subject to the operator's content review.
