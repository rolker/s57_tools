---
issue: 27
---

# Issue #27 — s57_to_geotiff: export ENC bathymetry as two-band GeoTIFFs for the chart layer

## Issue Review
**Status**: complete
**When**: 2026-07-31 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #27
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [x] Clarify CATZOC wiring scope: acceptance criteria require a CATZOC-varied fixture test, which means `marine_charts` `s57_dataset.cpp` case 308 must be wired. Either include this fix explicitly in this issue's scope, or open a prerequisite issue so it doesn't land silently at implementation time.
  — **Resolved (operator, 2026-07-31)**: wire CATZOC/M_QUAL in `marine_charts` **as part of this issue** (same repo, small change, acceptance requires it). The plan must carry it as an explicit work item.
- [x] Acknowledge cost-model gating: ADR-0010 D7 gates chart cell ingestion on the cost-model rework ("Chart ingestion is gated on the cost-model rework"). The round-trip acceptance demo (`export → import_geotiff → store query`) will hit this gate. Note the sequencing constraint explicitly so the acceptance test isn't blocked unexpectedly.
  — **Resolved (host)**: the gate applies to chart data feeding a *live costmap* deployment (uma#276, tracked as an Aug deploy precondition), not to the offline round-trip bench test (export → import → store query), which touches no costmap. The plan should note the sequencing constraint in its consequences section; it does not block this issue's acceptance.
- [x] Confirm uma#274 (`marine_vertical_datum`) is available before starting implementation — the datum conversion is load-bearing for the tool's correctness, and implementing the exporter without it would require a significant rework at wire-up time.
  — **Resolved (host)**: uma#274 merged 2026-07-24 (unh_marine_autonomy PR#279, `3578292`); uma#275 chart layer merged 2026-07-30 (PR#280, `6d3ca5c`). Both dependencies are satisfied.

## Plan Authored
**Status**: complete
**When**: 2026-07-31 15:26 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-27/plan.md` at `07aa8c3`
**Branch**: feature/issue-27 at `07aa8c3`
**Phases**: single

### Open questions
- [ ] Thread-safety scope for `make_vdatum_query()`: one factory call per cell or per corpus? (per-thread in v1 single-threaded loop = one call total)
- [ ] Output file naming: confirm `{S57Dataset::label()}.tif` is unambiguous across the New Castle ENC corpus before committing to it.

## Plan Review
**Status**: complete
**When**: 2026-07-31 15:30 +00:00
**By**: Claude Code Agent (Claude Opus)  <!-- independent: shares agent name with the Sonnet plan author but is a distinct model in a separate dispatched context; not an author self-review -->

**Plan**: `.agent/work-plans/issue-27/plan.md` at `07aa8c3`
**PR**: PR-less (reviewed from local plan + progress.md; `gh` unauthenticated in this context)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) `s57_to_geotiff/package.xml` omits `marine_autonomy`, which provides `gggs::Level::fromCellSize`; `marine_vertical_datum` does not pull it in transitively — build blocker — `plan.md:29`
- [ ] (suggestion) M_COVR footprint source for the finer-scale clip is unspecified; `S57Dataset` exposes only a bbox — state the exporter reads M_COVR (OBJL 302) geometry directly — `plan.md:36`
- [ ] (suggestion) scale→level constant `0.0003125` (0.3125 mm-at-scale) diverges from ADR-0010 D7's "≈0.5 mm-at-scale" (0.0005); confirm or document — `plan.md:37`
- [ ] (suggestion) ADR table mixes project (uma ADR-0010 D7, ADR-0002 D2) and workspace (0008/0009/0018) ADRs under one unqualified namespace with number collisions; qualify the project ones — `plan.md:80`
- [ ] (suggestion) cost-model gate sequencing noted in Context but not the Consequences table; getGrid() case 308 stays a no-op — add a consequence row for completeness — `plan.md:89`
