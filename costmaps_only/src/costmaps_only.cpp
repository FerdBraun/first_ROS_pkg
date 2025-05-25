#include "costmaps_only/costmaps_server.hpp"

nav2_util::CallbackReturn CostmapsServer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring costmaps");
  global_costmap_->configure();
  local_costmap_->configure();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CostmapsServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating costmaps");
  global_costmap_->activate();
  local_costmap_->activate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CostmapsServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating costmaps");
  global_costmap_->deactivate();
  local_costmap_->deactivate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CostmapsServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up costmaps");
  global_costmap_->cleanup();
  local_costmap_->cleanup();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CostmapsServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down costmaps");
  return nav2_util::CallbackReturn::SUCCESS;
}