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
- [ ] Specify the nav-down liveness signal contract in the plan: the issue says the updater "checks a navigation-liveness signal" but does not define the mechanism (ROS topic heartbeat, file sentinel, service call?). The plan must nail this down — the interlock is load-bearing safety logic, so the interface must be explicit before implementation begins.
- [ ] Specify the cross-repo API invocation mechanism: the swap step calls "the store's regeneration API" from `unh_marine_autonomy`. The plan must document how the updater invokes that API (subprocess CLI call, Python import, ROS service?) — leaving this implicit risks a coupling assumption that breaks at integration time.
- [ ] Verify issue #5 (auto-download & auto-update NOAA ENC data) is still open and add a `Closes #5` keyword to this PR so it doesn't linger as a ghost issue after the work lands.
- [ ] Confirm issue #27 (`s57_to_geotiff`) is merged and available on the branch before starting implementation — the exporter is invoked as step 3 of the regeneration pipeline.
