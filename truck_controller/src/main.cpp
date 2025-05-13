#include <rclcpp/rclcpp.hpp>
#include "truck_controller.hpp"
#include <string>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int argu_id = std::stoi(argv[1]);
  auto node = std::make_shared<TruckController>(argu_id);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}