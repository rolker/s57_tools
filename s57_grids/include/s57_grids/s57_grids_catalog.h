#ifndef S57_GRIDS_S57_GRIDS_CATALOG_H
#define S57_GRIDS_S57_GRIDS_CATALOG_H


#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geographic_msgs/msg/bounding_box.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "s57_msgs/msg/dataset_info.hpp"
#include "s57_msgs/srv/get_datasets.hpp"

namespace grid_map
{
class GridMap;
}

namespace s57_grids
{

using GridMapFuture = std::future<std::shared_ptr<grid_map::GridMap> >;

using S57GridsDataset = std::pair<s57_msgs::msg::DatasetInfo, GridMapFuture>;

/// Implements an interface similar to marine_charts::S57Catalog to access
/// S57 datasets using the s57_grids package.
/// This class issues service calls to the s57_grids services and provides
/// topics the client can us to
/// subscribe to the GridMap messages published by the
/// s57_grids package.
class S57GridsCatalog
{
public:
  S57GridsCatalog(rclcpp_lifecycle::LifecycleNode::WeakPtr node);

  std::vector<S57GridsDataset> get_datasets(const geographic_msgs::msg::BoundingBox &bounds, double minimum_scale = 0.0);

//  bool ecefToLatLong(double x, double y, double z, double &lat, double &lon);
//  bool llToECEF(double lat, double lon, double &x, double &y, double &z);

  //std::future<std::shared_ptr<S57Dataset> > dataset(std::string label) const;
private:

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;


  using GridMapPromise = std::promise<std::shared_ptr<grid_map::GridMap> >;

  struct GridMapDataset
  {
    s57_msgs::msg::DatasetInfo info;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscription;
    std::shared_ptr<grid_map::GridMap> grid_map;
    std::vector<GridMapPromise> promises;
  };

  std::map<std::string, GridMapDataset> grid_map_datasets_;

  rclcpp::Client<s57_msgs::srv::GetDatasets>::SharedPtr client_;

};


} // namespace s57_grids

#endif
