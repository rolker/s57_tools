---
issue: 30
---

# Issue #30 — ADR-0010 D10 split: suppress depth ramp in s57_layer

## Issue Review
**Status**: complete
**When**: 2026-08-04 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #30
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

The issue proposes implementing the ADR-0010 D10 split: add a mode to `s57_layer` (in `s57_tools`) that suppresses its depth-ramp cost computation, retaining only non-depth semantics (land/LNDARE, `restricted`, `overhead`, `caution`/`unsurveyed`, point hazards). This is explicitly called out in ADR-0010 D10 as its own issue/PR; the field evidence from the 2026-08-03 Broadkill deployment (echoboats#408) confirms the urgency.

### Scope Assessment

**Well-scoped?** Yes — the change is confined to `s57_layer` (a new parameter + conditional logic in `get_cost_from_grid` + TF-optional startup path). Single PR. The boat-side echoboats config change is explicitly deferred to a separate issue, keeping scope tight.

**Right repo?** Yes — `s57_layer` lives in `s57_tools`, which is a project repo. This is squarely a project-side domain change, not workspace infra.

**Dependencies**:
- ADR-0010 D10 is the authoritative design spec. Status is `Proposed`; but this issue is one of the explicitly enumerated implementation items ("lands as its own issue/PR"), so pre-adoption of D10 is clearly sanctioned.
- `bathymetry_layer` (rolker/unh_marine_autonomy) must be the depth authority in the costmap before the echoboats config flips. This issue doesn't require that flip — the new parameter defaults to current behavior, so it's safe to merge independently.
- The `s57_to_geotiff`-based chart layer import path (D7, #27 — merged) is a precondition for the echoboats config flip, not for this PR itself.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Safety First (project) | OK | Default preserves current behavior; no regression. The suppressed mode removes a known-harmful path (chart-band blocking surveyed navigable water) |
| Hardware Agnosticism | OK | Parameter-driven; no platform coupling |
| Modularity and Decoupling | OK | The mode cleanly separates depth authority (→ bathymetry_layer) from obstacle authority (s57_layer) |
| Human control and transparency | OK | Parameter must be explicit; behavior is observable via costmap topics |
| Capture decisions, not just implementations | Watch | ADR-0010 is still "Proposed". The issue description references D10 clearly, but the plan step should verify the ADR advances to Accepted (or at minimum that the implementation notes it's tracking a Proposed ADR) |
| A change includes its consequences | Watch | The issue lists the main consequence items (no TF warning in suppressed mode; regression test coverage for default mode). The PR should include tests for both modes and ensure README/API docs for `s57_layer` parameters are updated. The `chart_datum`/`sea_surface_frame` parameters become no-ops in suppressed mode — that should be documented |
| Only what's needed | OK | The sketch is minimal: one parameter, conditional skip of the depth ramp, optional TF. No over-engineering |
| Improve incrementally | OK | Default-preserve + explicit opt-in is the right incremental shape |
| Test what breaks | Watch | Acceptance criteria include sim/replay verification and a regression test for default mode. The plan step should ensure these are included, and that the "no TF warning in suppressed mode" property is tested (currently the startup path issues a warning when `chart_datum_frame_` is set but the TF is absent — suppressed mode should either skip that or suppress it cleanly) |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0001 — Adopt ADRs | Watch | ADR-0010 is Proposed, not Accepted. The implementation is sanctioned by D10's explicit "lands as its own issue/PR" note. Implementation should note the tracking ADR |
| ADR-0002 — Worktree isolation | OK | Already in worktree `feature/issue-30` |
| ADR-0008 — Follow ROS 2 conventions | OK | A new `bool` (or enum) parameter follows standard ROS 2 `declareParameter` pattern already in `s57_layer.cpp` |
| ADR-0010 — Geospatial world model | **Triggered (primary)** | This is a direct D10 implementation item. The parameter-default (preserving current behavior) matches the ADR's sequencing note. The D5 consequence (TF not required in suppressed mode) is explicitly addressed in the issue sketch |
| ADR-0013 — progress.md vocabulary | OK | `## Issue Review` entry written here |

### Consequences (from ADR-0010 consequences map and principles review guide)

- **Parameters** — two parameters (`chart_datum_frame`, `sea_surface_frame`) become no-ops in suppressed mode. Their `declareParameter` calls stay (no breaking change), but the TF lookup branch is skipped. The `s57_layer` README / API docs should document this behavior change in the same PR.
- **`overhead` semantics** — the `overhead` channel stays active (correct per D10 / #25). The `overhead_clearance` parameter and its lookup in `get_cost_from_grid` are unaffected. Confirm no accidental regression.
- **Test coverage** — the acceptance criteria require both modes tested; a replay/sim test with a store containing low-σ survey inside a high-σ chart band (Broadkill case) is the key validation scenario.
- **Cross-repo follow-on** — the echoboats config change (flipping the parameter on the boat) is deferred but should be filed as a follow-up issue or linked from this PR so it doesn't get lost.

### Recommendations

- The `plan-task` step should check whether ADR-0010 should advance from Proposed → Accepted as part of this PR series (or at minimum add a header pointer to the tracking issue).
- Consider naming the parameter `depth_costs` (bool, default `true`) as the issue sketch suggests, for clarity. An enum `mode` is also reasonable but adds complexity. Keep it simple.
- The "no TF warning in suppressed mode" property (from the acceptance criteria) should be an explicit test case — it is the operational pain point from echoboats#408.
- The `unsurveyed_cost` path in `get_cost_from_grid` (line 524 in current source) — when `depth_costs: false`, there's no `depth` variable, so the `unsurveyed` / `caution` semantics need to be preserved without the depth ramp. Clarify in the plan whether `unsurveyed_cost` applies to a cell with only `unsurveyed`/`caution` set and no elevation (currently it falls through to the elevation branch).

### Actions
- [ ] Verify ADR-0010 status handling: plan should note the issue tracks a Proposed ADR and flag if the ADR should advance as part of this work
- [ ] Ensure `s57_layer` README/parameter docs are updated in the same PR (consequences: parameter behavior change for `chart_datum_frame`/`sea_surface_frame` in suppressed mode)
- [ ] Include test cases for both modes: default (regression) and suppressed (no TF warning; obstacles still painted; charted land/LETHAL from LNDARE retained)
- [ ] Clarify `unsurveyed`/`caution` cost path in suppressed mode (no `depth` variable — need to preserve the `unsurveyed_cost` guard without the ramp)
- [ ] File or link the echoboats config-flip follow-up so it doesn't get lost after this PR merges

## Plan Authored
**Status**: complete
**When**: 2026-08-04 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-30/plan.md` at `0b332d7`
**Branch**: feature/issue-30 at `0b332d7`
**Phases**: single

### Open questions
- [ ] Should `depth_costs: false` suppress `tide_invalidate_threshold` declaration or leave it declared-but-inert? (Recommend: inert, documented.)
- [ ] File echoboats config-flip as a new issue before this PR merges, or link in PR description and defer? (Recommend: link in PR description.)

## Plan Review
**Status**: complete
**When**: 2026-08-04 02:54 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-30/plan.md` at `0b332d7`
**PR**: PR-less (reviewed via issue #30 in worktree `feature/issue-30`)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Suppressed mode drops charted submerged point hazards, contradicting the plan's own retained-semantics claim — `plan.md:19` (and `plan.md:11`). UWTROC (underwater rock) and WRECKS are rasterized into the **`elevation`** channel as negative elevation (`-VALSOU` sounding); PIPSOL likewise uses `-DRVAL1` (`marine_charts/src/s57_dataset.cpp:290-301,277-286`). Step 3's rule "for submerged cells (elevation <= 0) … return `NO_INFORMATION`" therefore erases charted rocks/wrecks in suppressed mode, deferring them to `bathymetry_layer` — which only has data inside survey coverage. A charted wreck outside surveyed water vanishes from the costmap. The `elevation` channel carries no marker separating a DEPARE depth band (which D10 *wants* suppressed — the Broadkill fix) from a discrete UWTROC/WRECKS sounding (which the issue says to *retain*), so s57_layer cannot honor "retain point hazards" from within its own logic. Resolve by one of: (a) add a dedicated hazard channel upstream in `marine_charts` (expands scope → likely a separate issue); (b) explicitly scope-out and document the limitation with a linked follow-up, stating that in suppressed mode charted point hazards outside `bathymetry_layer` coverage are not painted; or (c) keep a LETHAL floor for submerged cells shallower than `minimum_depth_` even in suppressed mode (but this reintroduces depth/tide dependence, partly defeating the no-TF goal). At minimum the plan must surface this tension instead of listing point hazards as retained.
- [ ] (suggestion) Test case (e) spec is imprecise — `plan.md:23`. For the proposed suppressed path to return `unsurveyed_cost_`, the cell must have **both** `elevation <= 0` **and** `unsurveyed`/`caution` set. The `unsurveyed`/`caution` check lives *inside* the elevation branch (`s57_layer.cpp:523`), so a caution-only cell (elevation = NaN, e.g. a bare CTNARE/UNSARE) returns `NO_INFORMATION` in both modes. Specify that case (e)'s fixture sets a submerged elevation too, and note that pure-caution/unsurveyed cells with no coincident DEPARE elevation are a pre-existing no-op (out of scope to fix, but the plan's "retain caution/unsurveyed" is only true where elevation data coexists).
- [ ] (suggestion) `onInitialize` still logs "Tide correction enabled" (`s57_layer.cpp:91-94`) whenever the frames are set, regardless of `depth_costs_`. In suppressed mode with the frames left in config this INFO is misleading. Gate it on `depth_costs_` too, consistent with step 2's TF-lookup gating.
- [ ] (suggestion) README table is missing `tide_invalidate_threshold`, `buffer_fraction`, `allow_uncharted`, and `get_datasets_service` as well as the two frame params. Step 4 covers only `depth_costs` + the two frames; add at least the three tide-related params together since they form the coherent set this change touches (broader gap-fill optional).
- [ ] (positive) ADR-0010 pre-adoption is handled correctly (implementation sanctioned by D10's explicit "lands as its own issue/PR"); `review-issue` actions 1-3 and 5 are addressed; scope is well-sized (5 files, single PR); the `Documentation & Instruction Impact` section is present and non-silent.
