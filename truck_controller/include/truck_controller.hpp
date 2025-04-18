#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ros2_msg/msg/gnss.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <GeographicLib/LocalCartesian.hpp>

struct Waypoint {
  double x, y, z;
};

class TruckController : public rclcpp::Node
{
public:
  TruckController();

private:
  void load_waypoints(const std::string &csv_path);
  void gnss_callback(const ros2_msg::msg::GNSS::SharedPtr msg);
  void compute_control();

  // Subscriber
  rclcpp::Subscription<ros2_msg::msg::GNSS>::SharedPtr sub_gnss_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vel_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steer_;

  int actor_id_;

  // Waypoints and state
  std::vector<Waypoint> waypoints_;
  size_t current_wp_idx_;
  double lookahead_dist_;
  double max_speed_;
  double k_angular_;

  // Current ENU position
  double cur_x_, cur_y_, cur_z_;
  double ref_lat_,ref_lon_,ref_alt_;
  bool origin_initialized_;

  // Geographic converter (from geographic_lib or similar)
  GeographicLib::LocalCartesian geo_converter_;
};
