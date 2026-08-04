# Plan: ADR-0010 D10 split — suppress depth ramp in s57_layer

## Issue

https://github.com/rolker/s57_tools/issues/30

## Context

`s57_layer` currently computes depth costs from ENC DEPARE/elevation data and applies a caution-depth ramp via `get_cost_from_grid` (`s57_layer.cpp:498–528`). `bathymetry_layer` then max-combines raise-only, so survey data with better uncertainty can never lower the chart band's cost — the Broadkill deployment (echoboats#408) hit exactly this: a ZOC-B chart band at 0–1.8 m blocked a channel measured at 2.4–2.5 m.

ADR-0010 D10 (unh_marine_autonomy) designates `bathymetry_layer` as the single depth authority and gives `s57_layer` a mode that suppresses its depth ramp, retaining only non-depth semantics: land, `restricted`, `overhead`, `caution`/`unsurveyed`, and point hazards. This issue is that mode, implemented as a single parameter; the echoboats config flip and `bathymetry_layer` changes are separate.

## Approach

1. **Add `depth_costs` bool parameter (default `true`)** — In `s57_layer.h`, add `bool depth_costs_ = true;` to the private member block. In `onInitialize`, `declareParameter("depth_costs", rclcpp::ParameterValue(true))` + `get_parameter`. Default `true` preserves existing behavior.

2. **Guard the TF lookup in `updateBounds`** — The tide-correction block at `s57_layer.cpp:158–187` currently fires whenever `chart_datum_frame_` and `sea_surface_frame_` are non-empty. Gate it on `depth_costs_` so no TF lookup (and no `WARN_THROTTLE`) occurs in suppressed mode, even if the frame params remain in the config: `if(depth_costs_ && !chart_datum_frame_.empty() && !sea_surface_frame_.empty())`.

3. **Add a dedicated `hazard` channel in `marine_charts`** *(plan-review must-fix resolution (a))* — UWTROC/WRECKS/PIPSOL currently rasterize only into `elevation` (as `-VALSOU` / `-DRVAL1`), indistinguishable from a DEPARE band, so suppressing the depth ramp would erase charted rocks/wrecks/pipelines wherever `bathymetry_layer` has no survey coverage. In `S57Dataset::getGrid`: `ret->add("hazard")`; in the UWTROC (153) / WRECKS (159) case, additionally rasterize into `hazard` with the sounding elevation (`-VALSOU`); in the PIPSOL (94) case, additionally rasterize into `hazard` with `-DRVAL1`. The `elevation` writes stay exactly as-is (default-mode behavior unchanged). Point-hazard footprints are small; treating them as LETHAL in suppressed mode regardless of charted depth is deliberately conservative.

4. **Modify `get_cost_from_grid` for suppressed mode** — When `depth_costs_` is false, after the `restricted` and `overhead` checks (unchanged): a set `hazard` cell (guarded by `grid.exists("hazard")` for mixed-version grids) returns `LETHAL_OBSTACLE`; land (`elevation > 0`) stays `LETHAL_OBSTACLE`; for submerged cells (`elevation <= 0`), skip the depth ramp and return `unsurveyed_cost_` if the `unsurveyed` or `caution` channel is set, else `NO_INFORMATION` (leaving the cell for `bathymetry_layer`). Cells with no elevation data continue to return `NO_INFORMATION`. Default mode (`depth_costs_` true) does not consult `hazard` — behavior byte-identical to today.

5. **Update `s57_layer/README.md`** — Add `depth_costs` to the parameter table **and fill the pre-existing table gaps**: `chart_datum_frame`, `sea_surface_frame`, `tide_invalidate_threshold`, `buffer_fraction`, `allow_uncharted`, `get_datasets_service` *(plan-review suggestion)*. Add a "Suppressed-depth mode" paragraph documenting: depth ramp skipped; `chart_datum_frame`/`sea_surface_frame`/`tide_invalidate_threshold` inert (no TF required, left declared for config compatibility); land/restricted/overhead/unsurveyed semantics preserved; charted point hazards (UWTROC/WRECKS/PIPSOL) stay LETHAL via the `hazard` channel.

6. **Gate the "Tide correction enabled" startup log on `depth_costs_`** *(plan-review suggestion)* — in suppressed mode with frames configured, log "depth costs suppressed (D10 mode); tide correction inactive" instead.

7. **Add unit tests in `test/test_depth_costs.cpp`** — Reuse the `S57LayerForTest` subclass pattern from `test_tide_offset.cpp` plus a `DepthCostsMode` bool setter. Cases: (a) default-mode regression — submerged cell costs as before; (b) suppressed mode — submerged non-caution cell returns `NO_INFORMATION`; (c) suppressed mode — land cell stays `LETHAL`; (d) suppressed mode — restricted cell stays `LETHAL`; (e) suppressed mode — cell with `elevation <= 0` **and** `unsurveyed`/`caution` set returns `unsurveyed_cost_` *(both conditions required — plan-review suggestion)*; (f) suppressed mode — `hazard` cell returns `LETHAL` even with deep `elevation`; (g) default mode — `hazard` channel present does not alter the depth-ramp result; (h) suppressed mode — grid **without** a `hazard` layer does not throw (mixed-version guard). Register `test_depth_costs` in `CMakeLists.txt` alongside existing gtest targets.

8. **Add integration smoke test for "no TF warning"** — In `test_depth_costs.cpp`, run a full `updateBounds` / `updateCosts` cycle with `depth_costs: false`, `chart_datum_frame` set to a non-empty string, and no TF published. Assert the layer reaches `isCurrent()` without throwing. (The full WARN suppression is best verified manually or via log-capture; the integration test confirms no crash / no exception.)

## Files to Change

| File | Change |
|------|--------|
| `marine_charts/src/s57_dataset.cpp` | Add `hazard` grid channel; rasterize UWTROC/WRECKS (`-VALSOU`) and PIPSOL (`-DRVAL1`) into it (elevation writes unchanged) |
| `s57_layer/src/s57_layer.h` | Add `bool depth_costs_ = true;` to private members |
| `s57_layer/src/s57_layer.cpp` | Declare `depth_costs` parameter; gate TF lookup + startup log; suppressed-mode branch in `get_cost_from_grid` incl. `hazard` → LETHAL |
| `s57_layer/README.md` | Add `depth_costs` + missing existing params to table; document suppressed-mode behavior |
| `s57_layer/test/test_depth_costs.cpp` | New file — 8 unit tests + 1 integration smoke test |
| `s57_layer/CMakeLists.txt` | Register `test_depth_costs` as a gtest target |

*(marine_charts has no test infrastructure; the `hazard`-channel semantics are covered from the consumer side via the (f)/(g)/(h) unit tests on hand-built grids, plus build verification. Standing up S-57 synthetic-cell tests for marine_charts — the `test_exporter.cpp` pattern lives in s57_to_geotiff — is out of scope.)*

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

*(Both resolved at the plan-review checkpoint, per the recommended defaults.)*

- `tide_invalidate_threshold` under `depth_costs: false`: **left declared but inert** — removing it would be a breaking change for configs that set it. Documented as a no-op in suppressed mode.
- echoboats config-flip follow-up: **linked in the PR description and deferred** — the echoboats repo change is a separate concern.

## Estimated Scope

Single PR.
