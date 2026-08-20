---
issue: 40
---

# Issue #40 — enc_updater: region-driven cell selection from the live catalog (retire hard-coded cell lists)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-20 16:32 -04:00
**By**: Claude Code Agent (Claude Fable 5)
**Verdict**: approved

**Branch**: feature/issue-40 at `8559579`
**Mode**: pre-push
**Depth**: Standard (reason: new module + behavior change, ~730 lines / 11 files)
**Must-fix**: 3 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — all findings fixed in-session (`8559579`), suite 123 green

Static analysis: covered by the package's own ament flake8/pep257 colcon tests (green). Plan drift: no work plan (direct-implementation issue). Copilot: off (default). Local model: skipped per operator guidance (--no-local while workspace#590 pends).

### Findings
- [x] (must-fix, cross-confirmed: Lens A + Lens B + lead governance) transient catalog defect (Active row, unparseable coverage / unknown status) silently pruned installed cells and shrank the chart layer with exit 0 — fixed by guard_degenerate_deselections — `enc_updater/enc_updater/downloader.py`
- [x] (must-fix, Lens A) prune ran before catalog validation/downloads: config typo or transient download failure destroyed valid corpus data — fixed by validate→download→prune ordering — `enc_updater/enc_updater/downloader.py`
- [x] (must-fix, Lens A + Lens B) previous-manifest load outside the guarded try: corrupt manifest escaped as traceback with no health record — moved inside — `enc_updater/enc_updater/__main__.py`
- [x] (suggestion, Lens B + lead governance) catalog-supplied cell names (untrusted) reach os.path.join/rmtree unvalidated — [A-Z0-9]{3,32} parse filter added — `enc_updater/enc_updater/downloader.py`
- [x] (suggestion, Lens B) --dry-run performed irreversible pruning — now prints would-be prunes only; crash-window manifest divergence self-heals via missing-.000 re-download; rmtree wrapped as UpdaterError — `enc_updater/enc_updater/downloader.py`
- [x] (suggestion, Lens B) store bootstrap makedirs could silently fork onto an unmounted volume — leaf-only mkdir with loud parent-missing error — `enc_updater/enc_updater/__main__.py`

### False positives
- (none)

## Integrated Review
**Status**: complete
**When**: 2026-08-20 16:40 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #43 at `a217dc8`
**Sources**: 3 (Copilot R1 @ `a217dc8`, Local Review (Pre-Push) @ `8559579` — all 6 findings closed, CI rollup)
**Cross-source confirmations**: 0
**CI**: all-pass (build-and-test success)

### Findings
- [ ] (minor, Copilot) store-bootstrap error says "parent must already exist" even when store_dir exists as a plain file — detect exists-as-non-dir with a distinct message — `enc_updater/enc_updater/__main__.py:83`
- [ ] (minor, Copilot) dry-run selection diff/health record is wrong: prune skipped so removals are missed, and a preview run writes last_selection_change — skip the diff + health write under dry-run — `enc_updater/enc_updater/__main__.py:116`
- [ ] (minor, Copilot) README antimeridian claim inaccurate for polygons: per-vertex validation does not prevent a ±180-straddling polygon (interpreted planar, long-way-around) — state the real limitation — `enc_updater/README.md:140`

### False positives
- (none)

## Integrated Review
**Status**: complete
**When**: 2026-08-20 16:47 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #43 at `a6c69a2` (round 3; fixes committed on top)
**Sources**: 2 (Copilot R2 @ `a6c69a2`, CI rollup; round-2 findings all closed at `a6c69a2`)
**Cross-source confirmations**: 0
**CI**: all-pass (build-and-test success on `a6c69a2`)

### Findings
- [x] (minor, Copilot) prune should name a symlinked cell dir explicitly and report a stray non-directory instead of silently skipping it — fixed with named refusal + warning, 2 regression tests — `enc_updater/enc_updater/downloader.py:259`

### False positives
- (Copilot) "rmtree can delete outside the corpus through a symlink" — impossible: shutil.rmtree raises OSError on a top-level symlink by documented contract, and prune's round-1 wrapper converts that to a clean UpdaterError before the manifest is touched; no traversal or deletion can occur (now also pre-empted by an explicit islink refusal).
