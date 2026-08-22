---
issue: 49
---

# Issue #49 — s57_to_geotiff: stop clipping coarse charts by finer cells' M_COVR — it deletes the LOD ancestors

## Integrated Review
**Status**: complete
**When**: 2026-08-22 16:18 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))

**PR**: #50 at `a8d8137f`
**Sources**: 1 (Copilot @ `a8d8137f`, current). No human reviews, no
conversation comments. **No prior local timeline** — this issue was driven by
hand rather than through the orchestrator, so this entry creates `progress.md`
mid-lifecycle; the Issue Review / Plan Authored / Plan Review entries a full
run would have written do not exist.
**Cross-source confirmations**: 0
**CI**: all pass (`build-and-test` success)

Both comments verified against the code. Both valid; one has a remedy I would
not follow as suggested.

### Findings
- [ ] (valid, Copilot) Stale warning text: `"cannot reopen"` survives from the
      two-pass flow, where Pass A opened each cell for its footprints and Pass B
      opened it again. Collapsing to one pass made this the FIRST and only open,
      so the message misdescribes what happened and would send an operator
      looking for a first open that never occurred —
      `s57_to_geotiff/src/exporter.cpp:570`
- [ ] (valid, Copilot — remedy differs) `exportCell`'s docstring still says the
      export follows `uma-ADR-0010` D7. Copilot suggests removing the D7
      reference. **Do not.** D7 governs far more than the withdrawn clause: depth
      sources (DEPARE/DRGARE midpoint + half-band sigma floor, SOUNDG), the
      CATZOC->sigma table and its finite-sigma hard contract, per-cell chart
      datum via the D6 library, and scale->GGGS-level selection. The export still
      follows all of it. Only the "largest scale governs" clipping clause is
      withdrawn (uma#337, merged as uma#338). Qualify the reference rather than
      deleting it — deleting would leave the export looking ungoverned, which is
      further from the truth than the stale line is —
      `s57_to_geotiff/src/exporter.hpp:46`

### False positives
- None this round.

### Notes (not findings)
- The paired ADR amendment (uma#337 / PR uma#338) is **already merged**, so the
  D7 reference can now be qualified against a landed decision rather than a
  pending one.
