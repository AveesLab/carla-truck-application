#pragma once

#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/truck_status.hpp"
#include "ros2_msg/msg/truck_command.hpp"
#include <unordered_map>
#include <vector>
#include <string>

struct TruckState {
  double soc;
  double speed;
  double position;
  std::string mode;
};

struct TruckCommandData {
  std::string mode;
  bool change_flag;
};

class PlatoonCoordinator : public rclcpp::Node {
public:
  PlatoonCoordinator();

private:
  void status_callback(const ros2_msg::msg::TruckStatus::SharedPtr msg);
  void optimization_timer_callback();
  void publish_commands();
  std::string get_next_mode(const std::string &current_mode);

  std::unordered_map<int, TruckState> truck_states_;
  std::unordered_map<int, TruckCommandData> command_buffer_;
  std::unordered_map<int, double> previous_soc_;

  std::vector<rclcpp::Subscription<ros2_msg::msg::TruckStatus>::SharedPtr> truck_subscribers_;  
  std::vector<rclcpp::Publisher<ros2_msg::msg::TruckCommand>::SharedPtr> command_publishers_;
  rclcpp::TimerBase::SharedPtr optimization_timer_;

  //파라미터 설정 
  int max_trucks_ = 3; 
};
