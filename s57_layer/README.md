# s57_layer

A **Nav2 Costmap 2D** plugin that creates a costmap layer from S57 nautical charts.

This plugin queries the `s57_grids` node for relevant chart data and populates the costmap based on water depth, overhead clearance, and restricted areas.

## Usage

Add the plugin to your Nav2 params file (usually `nav2_params.yaml`) under `local_costmap` or `global_costmap`.

### Example Configuration

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["s57_layer", "inflation_layer"]
      s57_layer:
        plugin: "s57_layer::S57Layer"
        enabled: true
        minimum_depth: 2.0        # Meters. Depths < this are LETHAL.
        maximum_caution_depth: 5.0 # Meters. Depths between min and max get scaled cost.
        overhead_clearance: 5.0    # Meters. Objects lower than this are LETHAL.
        unsurveyed_cost: 128       # Cost for unsurveyed areas (0-255).
        s57_grids_namespace: ""    # Namespace of s57_grids_node (optional)
```

## How It Works

1.  Determines the geographic bounds of the costmap window.
2.  Calls the `get_datasets` service (provided by `s57_grids`) to find available ENCs.
3.  Subscribes to the grid map topics for those ENCs.
4.  Converts depth and feature info into costmap values:
    *   **Restricted Areas**: LETHAL_OBSTACLE
    *   **Low Overhead**: LETHAL_OBSTACLE
    *   **Depth < minimum_depth**: LETHAL_OBSTACLE
    *   **Depth < maximum_caution_depth**: Scaled cost (Non-Lethal to Free)

## Suppressed-depth mode (ADR-0010 D10)

With `depth_costs: false`, this layer stops computing depth costs — `bathymetry_layer`
(unh_marine_autonomy) becomes the single depth authority, arbitrating charted and
surveyed depths by per-cell uncertainty. The layer then paints only:

*   **Land / built features** (`elevation > 0`): LETHAL_OBSTACLE
*   **Restricted areas** and **low overhead clearance**: LETHAL_OBSTACLE (unchanged)
*   **Charted point hazards** (UWTROC underwater rocks, WRECKS, PIPSOL pipelines,
    via the grid's `hazard` channel): LETHAL_OBSTACLE regardless of charted depth —
    deliberately conservative, so a charted rock or wreck outside survey coverage
    never vanishes from the costmap. Soundingless UWTROC/WRECKS are assumed awash
    and painted lethal (hazard channel only; default-mode costs unchanged).
    **Safety caveat**: a PIPSOL charted without `DRVAL1` is *not* rasterized —
    blanket-lethal on a long buried-pipeline route could wrongly close a whole
    channel — so in suppressed mode such pipelines rely on `bathymetry_layer`
    coverage or operator awareness.
*   **Unsurveyed / caution** submerged areas that carry a charted depth band:
    `unsurveyed_cost` (a bare UNSARE/CTNARE mark with no `elevation` data is
    NO_INFORMATION in both modes)
*   All other submerged cells: NO_INFORMATION (left to `bathymetry_layer`)

In this mode the depth-related parameters — `minimum_depth`, `maximum_caution_depth`,
`chart_datum_frame`, `sea_surface_frame`, `tide_invalidate_threshold` — are **inert**
(still declared, for config compatibility). No `chart_datum` TF is required or looked
up: a missing tide transform can neither warn nor invalidate tiles.

## Parameters

| Parameter | Type | Description | Default |
|---|---|---|---|
| `enabled` | bool | Enable/Disable the layer. | true |
| `depth_costs` | bool | `false` suppresses the depth ramp (ADR-0010 D10 mode, see above). | true |
| `minimum_depth` | double | Water depth (m) considered an obstacle. Inert when `depth_costs: false`. | 0.0 |
| `maximum_caution_depth` | double | Water depth (m) where cost scaling ends. Inert when `depth_costs: false`. | 5.0 |
| `overhead_clearance` | double | Height (m) of required clearance. | 10.0 |
| `unsurveyed_cost` | int | Cost (0-255) for unsurveyed areas. | 100 |
| `update_timeout` | double | Max time (s) to wait for chart updates. | 0.5 |
| `tile_size` | int | Internal tile size for sub-grids. | 100 |
| `buffer_fraction` | double | Fraction of the window size used to buffer chart requests. | 0.05 |
| `allow_uncharted` | bool | Leave uncharted cells untouched (`true`) or cost them LETHAL (`false`). | true |
| `chart_datum_frame` | string | TF frame of the chart datum for tide correction. Empty disables tide correction. Inert when `depth_costs: false`. | "" |
| `sea_surface_frame` | string | TF frame of the sea surface for tide correction. Empty disables tide correction. Inert when `depth_costs: false`. | "map_tide" |
| `tide_invalidate_threshold` | double | Tide-offset change (m) that invalidates cached tiles. Inert when `depth_costs: false`. | 0.01 |
| `get_datasets_service` | string | Override for the `s57_grids` dataset service name. | `<ns>/get_datasets` |
| `s57_grids_namespace` | string | Namespace prefix for `s57_grids` services/topics. | "" |
