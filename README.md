# s57_tools

A collection of ROS 2 packages for using S57 ENC (Electronic Navigation Charts) in robotic navigation.

## Overview

This suite allows you to load S57 charts, convert them into Grid Maps, and use them directly in the Nav2 stack as a costmap layer.

## Packages

*   **[s57_grids](s57_grids)**: The core node. Reads S57 files and publishes them as `grid_map` messages.
*   **[s57_layer](s57_layer)**: A `nav2_costmap_2d` plugin. Subscribes to `s57_grids` outputs and adds them to the costmap (marking shallow water, restricted areas, etc. as obstacles).
*   **[marine_charts](marine_charts)**: Underlying C++ library for parsing S57 data.
*   **[s57_msgs](s57_msgs)**: Custom service and message definitions.
*   **[s57_to_geotiff](s57_to_geotiff)**: CLI exporter — ENC bathymetry as two-band (depth, σ) GeoTIFFs for the bathymetry store's `chart` layer (ADR-0010 D7).
*   **[enc_updater](enc_updater)**: Cron-friendly chart updater — downloads NOAA ENC updates and regenerates the store's `chart` layer wholesale, with an enforced nav-down interlock (ADR-0010 D7).

## Workflow

1.  **Data Prep**: Acquire S57 ENC files (`.000`).
2.  **Launch**: Start `s57_grids_node` pointing to your data directory.
3.  **Configure**: Set up your Nav2 usage to include the `s57_layer` plugin.
4.  **Navigate**: As the robot moves, `s57_layer` dynamically requests map data from `s57_grids` based on the robot's location.

## Quick Start relative to s57_grids

See [s57_grids/README.md](s57_grids/README.md) for details on running the grid publisher.
See [s57_layer/README.md](s57_layer/README.md) for details on configuring the Nav2 plugin.
