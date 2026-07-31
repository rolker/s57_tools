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
- [ ] Clarify CATZOC wiring scope: acceptance criteria require a CATZOC-varied fixture test, which means `marine_charts` `s57_dataset.cpp` case 308 must be wired. Either include this fix explicitly in this issue's scope, or open a prerequisite issue so it doesn't land silently at implementation time.
- [ ] Acknowledge cost-model gating: ADR-0010 D7 gates chart cell ingestion on the cost-model rework ("Chart ingestion is gated on the cost-model rework"). The round-trip acceptance demo (`export → import_geotiff → store query`) will hit this gate. Note the sequencing constraint explicitly so the acceptance test isn't blocked unexpectedly.
- [ ] Confirm uma#274 (`marine_vertical_datum`) is available before starting implementation — the datum conversion is load-bearing for the tool's correctness, and implementing the exporter without it would require a significant rework at wire-up time.
