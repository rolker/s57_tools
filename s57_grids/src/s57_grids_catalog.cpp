#include "s57_grids/s57_grids_catalog.h"
#include "grid_map_ros/grid_map_ros.hpp"

namespace s57_grids
{

S57GridsCatalog::S57GridsCatalog(rclcpp_lifecycle::LifecycleNode::WeakPtr node_ptr)
  : node_(node_ptr)
{
  auto node = node_.lock();
  if(!node->has_parameter("get_datasets_service"))
  {
    node->declare_parameter("get_datasets_service", rclcpp::ParameterValue("get_datasets"));
  }
  auto service_name = node->get_parameter("get_datasets_service").as_string();
  client_ = node->create_client<s57_msgs::srv::GetDatasets>(service_name);
}

std::vector<S57GridsDataset > S57GridsCatalog::get_datasets(
  const geographic_msgs::msg::BoundingBox &bounds, double minimum_scale)
{
  auto request = std::make_shared<s57_msgs::srv::GetDatasets::Request>();
  request->bounds = bounds;
  request->minimum_scale = minimum_scale;

  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(node_.lock()->get_logger(), "Service %s not available", client_->get_service_name());
    return {};
  }

  auto result_future = client_->async_send_request(request);

  if(rclcpp::spin_until_future_complete(
    node_.lock(),
    result_future,
    std::chrono::seconds(2)
  ) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_.lock()->get_logger(), "Failed to call service %s", client_->get_service_name());
    return {};
  }

  auto result = result_future.get();

  std::vector<S57GridsDataset > datasets;
  if(result)
  {
    for (const auto &dataset_info : result->datasets)
    {
      grid_map_datasets_[dataset_info.label].info = dataset_info;
      auto &dataset = grid_map_datasets_[dataset_info.label];
      if(dataset.grid_map)
      {
        GridMapPromise promise;
        datasets.emplace_back(dataset_info, std::move(promise.get_future()));
        promise.set_value(dataset.grid_map);
        continue;
      }
      else
      {
        dataset.promises.emplace_back(GridMapPromise());
        datasets.emplace_back(dataset_info, std::move(dataset.promises.back().get_future()));
        if(!dataset.subscription)
        {
          auto node = node_.lock();
          dataset.subscription = node->create_subscription<grid_map_msgs::msg::GridMap>(
            dataset_info.topic, 10,
            [this, label = dataset_info.label](const grid_map_msgs::msg::GridMap::SharedPtr msg)
            {
              auto &dataset = grid_map_datasets_[label];
              dataset.grid_map = std::make_shared<grid_map::GridMap>();
              grid_map::GridMapRosConverter::fromMessage(*msg, *dataset.grid_map);
              for (auto &promise : dataset.promises)
              {
                promise.set_value(dataset.grid_map);
              }
              dataset.promises.clear();
            });
        }
      }
    }
  }

  return datasets;
}

} // namespace s57_grids
