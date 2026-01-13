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

## Parameters

| Parameter | Type | Description | Default |
|---|---|---|---|
| `enabled` | bool | Enable/Disable the layer. | true |
| `minimum_depth` | double | Water depth (m) considered an obstacle. | |
| `maximum_caution_depth` | double | Water depth (m) where cost scaling ends. | |
| `overhead_clearance` | double | Height (m) of required clearance. | |
| `unsurveyed_cost` | int | Cost (0-255) for unsurveyed areas. | |
| `update_timeout` | double | Max time (s) to wait for chart updates. | |
| `tile_size` | int | Internal tile size for sub-grids. | |
| `s57_grids_namespace` | string | Namespace prefix for `s57_grids` services/topics. | "" |
