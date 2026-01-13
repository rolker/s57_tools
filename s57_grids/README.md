# s57_grids

ROS 2 package for generating Grid Maps from S57 Electronic Navigation Charts (ENCs).

This package provides a **Lifecycle Node** `s57_grids_node` that reads S57 chart data and publishes it as `grid_map_msgs/GridMap` and optional `nav_msgs/OccupancyGrid` messages.

## Usage

### Prerequisites
You must have a directory containing your S57 ENC files (e.g., `.000` files).

### Environment Variable
The node uses the `ROS_S57_ENC_ROOT` environment variable to locate the root directory of your S57 charts. Alternatively, you can provide the `enc_root` parameter.

```bash
export ROS_S57_ENC_ROOT=/path/to/charts
```

### Running the Node
Since this is a lifecycle node, it starts in the `Unconfigured` state. You must transition it to `Active`.

```bash
# Terminal 1: Run the node
ros2 run s57_grids s57_grids_node

# Terminal 2: Configure and Activate
ros2 lifecycle set /s57_grids_node configure
ros2 lifecycle set /s57_grids_node activate
```

## Parameters

| Parameter | Type | Description | Default |
|---|---|---|---|
| `enc_root` | string | Path to ENC root directory. Fallback for `ROS_S57_ENC_ROOT`. | (env var) |
| `resolution_factor` | double | Factor to scale the chart resolution. | |
| `map_frame` | string | The TF frame for the generated maps. | |
| `grid_republish_period` | double | Period (s) to republish grids. 0.0 disables republishing. | |
| `publish_costmaps` | bool | If true, also publishes `OccupancyGrid` messages. | |

## Services

*   `~/list_datasets` (`s57_msgs/srv/GetDatasets`): Returns a list of available chart datasets for a given geographic bounding box.
*   `~/get_datasets` (`s57_msgs/srv/GetDatasets`): Requests the node to load and publish specific datasets.

## Published Topics

*   `datasets/<dataset_name>` (`grid_map_msgs/msg/GridMap`): The S57 data as a multi-layer grid map (elevation, overhead, restricted, unsurveyed, caution).
*   `datasets/occupancy_grids/<dataset_name>` (`nav_msgs/msg/OccupancyGrid`): (Optional) Standard costmap representation.
