#include "rclcpp/rclcpp.hpp"
#include "costmaps_only/costmaps_server.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CostmapsServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
