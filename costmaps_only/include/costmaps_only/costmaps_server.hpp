#ifndef COSTMAPS_SERVER_HPP_
#define COSTMAPS_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

class CostmapsServer : public nav2_util::LifecycleNode
{
public:
  explicit CostmapsServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : nav2_util::LifecycleNode("costmaps_server", "", false, options)
  {
    RCLCPP_INFO(get_logger(), "Creating costmaps server");

    // Получаем параметр use_sim_time
    
    bool use_sim_time = get_parameter("use_sim_time").as_bool();

    // Инициализируем costmaps
    global_costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap",
      std::string{get_namespace()},
      "global_costmap");
    
    local_costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "local_costmap",
      std::string{get_namespace()},
      "local_costmap");

    // Передаём параметр use_sim_time
    global_costmap_->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
    local_costmap_->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));

    // Запуск потоков для обновления costmap'ов
    costmap_threads_.push_back(std::make_unique<nav2_util::NodeThread>(global_costmap_));
    costmap_threads_.push_back(std::make_unique<nav2_util::NodeThread>(local_costmap_));
  }

  ~CostmapsServer()
  {
    costmap_threads_.clear();
    global_costmap_.reset();
    local_costmap_.reset();
  }

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap_;
  std::vector<std::unique_ptr<nav2_util::NodeThread>> costmap_threads_;
};

#endif  // COSTMAPS_SERVER_HPP_