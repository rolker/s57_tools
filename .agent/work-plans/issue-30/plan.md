# Plan: ADR-0010 D10 split — suppress depth ramp in s57_layer

## Issue

https://github.com/rolker/s57_tools/issues/30

## Context

`s57_layer` currently computes depth costs from ENC DEPARE/elevation data and applies a caution-depth ramp via `get_cost_from_grid` (`s57_layer.cpp:498–528`). `bathymetry_layer` then max-combines raise-only, so survey data with better uncertainty can never lower the chart band's cost — the Broadkill deployment (echoboats#408) hit exactly this: a ZOC-B chart band at 0–1.8 m blocked a channel measured at 2.4–2.5 m.

ADR-0010 D10 (unh_marine_autonomy) designates `bathymetry_layer` as the single depth authority and gives `s57_layer` a mode that suppresses its depth ramp, retaining only non-depth semantics: land, `restricted`, `overhead`, `caution`/`unsurveyed`, and point hazards. This issue is that mode, implemented as a single parameter; the echoboats config flip and `bathymetry_layer` changes are separate.

## Approach

1. **Add `depth_costs` bool parameter (default `true`)** — In `s57_layer.h`, add `bool depth_costs_ = true;` to the private member block. In `onInitialize`, `declareParameter("depth_costs", rclcpp::ParameterValue(true))` + `get_parameter`. Default `true` preserves existing behavior.

2. **Guard the TF lookup in `updateBounds`** — The tide-correction block at `s57_layer.cpp:158–187` currently fires whenever `chart_datum_frame_` and `sea_surface_frame_` are non-empty. Gate it on `depth_costs_` so no TF lookup (and no `WARN_THROTTLE`) occurs in suppressed mode, even if the frame params remain in the config: `if(depth_costs_ && !chart_datum_frame_.empty() && !sea_surface_frame_.empty())`.

3. **Modify `get_cost_from_grid` for suppressed mode** — When `depth_costs_` is false, after the `restricted` and `overhead` checks (unchanged), handle the elevation branch differently: land (`elevation > 0`) stays `LETHAL_OBSTACLE`; for submerged cells (`elevation <= 0`), skip the depth ramp and return `unsurveyed_cost_` if the `unsurveyed` or `caution` channel is set, else return `NO_INFORMATION` (leaving the cell for `bathymetry_layer`). Cells with no elevation data continue to return `NO_INFORMATION`.

4. **Update `s57_layer/README.md`** — Add `depth_costs` to the parameter table. Add a "Suppressed-depth mode" paragraph documenting: depth ramp skipped, `chart_datum_frame`/`sea_surface_frame` ignored (no TF required), land/restricted/overhead/unsurveyed semantics preserved.

5. **Add unit tests in `test/test_depth_costs.cpp`** — Reuse the `S57LayerForTest` subclass pattern from `test_tide_offset.cpp` plus a new `DepthCostsMode` bool setter. Five cases: (a) default-mode regression — submerged cell costs as before; (b) suppressed mode — submerged non-caution cell returns `NO_INFORMATION`; (c) suppressed mode — land cell stays `LETHAL`; (d) suppressed mode — restricted cell stays `LETHAL`; (e) suppressed mode — `unsurveyed`/`caution` cell returns `unsurveyed_cost_`. Register `test_depth_costs` in `CMakeLists.txt` alongside existing gtest targets.

6. **Add integration smoke test for "no TF warning"** — In `test_depth_costs.cpp`, run a full `updateBounds` / `updateCosts` cycle with `depth_costs: false`, `chart_datum_frame` set to a non-empty string, and no TF published. Assert the layer reaches `isCurrent()` without throwing. (The full WARN suppression is best verified manually or via log-capture; the integration test confirms no crash / no exception.)

## Files to Change

| File | Change |
|------|--------|
| `s57_layer/src/s57_layer.h` | Add `bool depth_costs_ = true;` to private members |
| `s57_layer/src/s57_layer.cpp` | Declare `depth_costs` parameter; gate TF lookup; conditional branch in `get_cost_from_grid` |
| `s57_layer/README.md` | Add `depth_costs` parameter entry; document suppressed-mode behavior |
| `s57_layer/test/test_depth_costs.cpp` | New file — 5 unit tests + 1 integration smoke test |
| `s57_layer/CMakeLists.txt` | Register `test_depth_costs` as a gtest target |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | `depth_costs` must be explicit; default `true` is no-surprise. README documents suppressed-mode behavior change for `chart_datum_frame`/`sea_surface_frame` |
| A change includes its consequences | README updated same PR; `chart_datum_frame`/`sea_surface_frame` become no-ops in suppressed mode — documented |
| Only what's needed | One bool parameter, minimal conditional, no new abstractions |
| Test what breaks | Default-mode regression + suppressed-mode paths explicitly tested; TF-absent smoke test covers the echoboats#408 operational pain |
| Improve incrementally | Default preserves current behavior; suppressed mode opted in per config |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0010 (unh_marine_autonomy, D10) | **Yes — primary** | Direct implementation of D10's "`s57_layer` gains a mode suppressing its depth ramp". ADR status is Proposed; this PR is one of the explicitly enumerated D10 implementation items. Plan notes tracking; ADR advancement to Accepted is out of scope for this PR |
| ADR-0001 (workspace — adopt ADRs) | Watch | Implementation precedes ADR-0010 acceptance; sanctioned by D10's explicit "lands as its own issue/PR" listing |
| ADR-0008 (ROS 2 conventions) | OK | `declareParameter` + `get_parameter` pattern already used in `onInitialize` |
| ADR-0013 (progress.md vocabulary) | OK | `## Plan Authored` entry follows this plan |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `depth_costs` suppresses TF lookup | `chart_datum_frame`/`sea_surface_frame` behavior documented as no-ops | Yes — README step 4 |
| `get_cost_from_grid` suppressed path | `overhead` channel unaffected — confirm no accidental regression | Yes — test case (c) covers land; overhead tested via existing tests |
| New `test_depth_costs.cpp` | `CMakeLists.txt` target registration | Yes — step 5 |
| This PR merges | echoboats config flip still pending | No — deferred; file as follow-up or link from PR description |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `s57_layer/README.md` — the parameter table and "How It Works" section do not document `depth_costs`, and `chart_datum_frame`/`sea_surface_frame` are not yet in the table at all. Both gaps must be filled.
- **Agent-instruction candidates** (proposals only): the `S57LayerForTest` subclass pattern (protected-member exposure for unit testing without a full ROS node) is a reusable testing convention worth capturing in `.agent/knowledge/` — the pattern appears in `test_tide_offset.cpp` and will repeat in `test_depth_costs.cpp`.

## Open Questions

- Should `depth_costs: false` also suppress the `tide_invalidate_threshold` parameter declaration (it becomes meaningless), or leave it declared but inert? Recommend: leave declared but inert — removing it would be a breaking change for configs that set it. Document it as a no-op in suppressed mode.
- Should the echoboats config-flip follow-up be filed as a new issue before this PR merges, or linked in the PR description and deferred? Recommend: link in PR description; the echoboats repo change is a separate concern.

## Estimated Scope

Single PR.
