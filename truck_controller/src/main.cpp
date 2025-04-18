#include <rclcpp/rclcpp.hpp>
#include "truck_controller.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TruckController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}