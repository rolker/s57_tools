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
