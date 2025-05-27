#ifndef COSTMAPS_SERVER_HPP_
#define COSTMAPS_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

class CostmapsServer : public nav2_util::LifecycleNode
{
public:
  explicit CostmapsServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CostmapsServer();

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  bool checkTransform(
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Duration & timeout);

  void logCostmapInfo(const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap, 
                     const std::string & name);

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::unique_ptr<std::thread> costmap_thread_;
};

#endif  // COSTMAPS_SERVER_HPP_
