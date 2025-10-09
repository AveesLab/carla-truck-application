#include "rclcpp/rclcpp.hpp"
#include "platoon_coordinator/platoon_coordinator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlatoonCoordinator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
