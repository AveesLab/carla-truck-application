#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ros2_msg/msg/truck_status.hpp"
#include "ros2_msg/msg/truck_command.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "BMS.h"
#include <chrono>
#include <unistd.h> 
#include <fstream>


class SOCEstimatorNode : public rclcpp::Node
{
public:
    SOCEstimatorNode();

private:
    void formation_start_callback(const ros2_msg::msg::TruckCommand::SharedPtr msg);
    void formation_end_callback(const std_msgs::msg::Int32::SharedPtr msg, int id);
    double compute_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    void start_baseline_measurement();
    void timer_callback();

    //void publish_output(double soc, double throttle, double brake);

    std::ofstream csv_file_;
    std::string truck_id_;
    std::string mode_;
    int pred_id_;
    int max_trucks_ = 3; 
    std::vector<int> last_flag;

    BMS BMSObj;

    rclcpp::Publisher<ros2_msg::msg::TruckStatus>::SharedPtr status_pub_;
    //rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr soc_pub_;
    //rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_pub_;
    //rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr control_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;

    rclcpp::Subscription<ros2_msg::msg::TruckCommand>::SharedPtr command_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> formation_subs_;

    rclcpp::TimerBase::SharedPtr timer_;
   	rclcpp::Time last_sim_time_;
	const double step_interval_sec_ = 0.1;

    // Formation 변경 측정용
    geometry_msgs::msg::Point pos_at_change_start_;
   	rclcpp::Time time_at_change_start_;
	double soc_at_change_start_;
	bool is_measuring_change_ = false;

    // 위치 정보
    geometry_msgs::msg::Point ego_pos;
    geometry_msgs::msg::Point target_pos;

    double current_speed_ = 0.0;
    double throttle_control_ = 0.0; 
    double last_soc_= 100.0;
    double last_pos_= 0.0;
    double last_vel_= 0.0;
    bool mode_changed_ = false;
    rclcpp::Time start_time_;

    geometry_msgs::msg::Point pos_at_baseline_start_;
    double soc_at_baseline_start_ = 0.0;
    rclcpp::Time time_at_baseline_start_;
    double baseline_target_distance_ = 0.0;
    bool measuring_baseline_ = false;
};

