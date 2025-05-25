#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp" // Сообщения для состояния lifecycle ноды
#include "costmaps_only/costmaps_server.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CostmapsServer>();

  // Получаем указатель на базовый интерфейс NodeBaseInterface
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base =
    node->get_node_base_interface();

  // Создаём MultiThreadedExecutor и добавляем ноду
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_base);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}