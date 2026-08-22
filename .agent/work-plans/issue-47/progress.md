---
issue: 47
---

# Issue #47 — enc_updater: fetch every chart scale — remove the usage-band filter and max_cells cap

## Integrated Review
**Status**: complete
**When**: 2026-08-22 14:40 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))

**PR**: #48 at `eb6ecad0`
**Sources**: 1 (Copilot @ `eb6ecad0`, current). No human reviews, no
conversation comments, and no prior local timeline — this entry creates it.
**Cross-source confirmations**: 0
**CI**: all pass (`build-and-test` success, `copilot-pull-request-reviewer` success)

All three comments verified against the code and **all three are valid**. No
false positives this round.

### Findings
- [ ] (valid, Copilot) `select_cells` docstring hard-codes catalog-dependent
      counts — "matches 3 of them against 77 approach/harbour cells" will drift
      with NOAA rescheming, which is the exact churn `region:` mode exists to
      absorb. The README already states it qualitatively ("a handful of cells
      against dozens"); make the docstring match —
      `enc_updater/enc_updater/selection.py:114-115`
- [ ] (valid, Copilot) Run-on line in the README: my edit joined the
      antimeridian sentence onto the end of the empty-selection paragraph,
      leaving one ~110-char line mid-paragraph —
      `enc_updater/README.md:139`
- [ ] (valid, Copilot) `test_retired_selection_keys_rejected` asserts only
      `pytest.raises(UpdaterError)` with no `match=`, so any unrelated
      validation failure satisfies it. The test's own docstring says the key must
      be "rejected as an unknown key", and the real message is
      `config: unknown key(s) ['bands'] in <path>` (verified by running it), so
      the assertion is weaker than its stated intent. Every neighbouring config
      test uses `match=` — `enc_updater/test/test_config.py:111`

### False positives
- None this round.

### Notes (not findings)
- Finding 3 is the vacuous-assert class: the test passes today and would keep
  passing if the unknown-key check were removed entirely, since a malformed
  config raises `UpdaterError` for many reasons. Worth fixing on principle
  rather than for this PR alone.
