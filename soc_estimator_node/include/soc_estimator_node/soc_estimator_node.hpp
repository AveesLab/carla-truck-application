#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ros2_msg/msg/truck_status.hpp"
#include "ros2_msg/msg/truck_command.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "BMS.h"
#include <chrono>
#include <unistd.h> 

class SOCEstimatorNode : public rclcpp::Node
{
public:
    SOCEstimatorNode();

private:
    void currentspeed_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void command_callback(const ros2_msg::msg::TruckCommand::SharedPtr msg);
    void formation_end_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void predecessor_status_callback(const ros2_msg::msg::TruckStatus::SharedPtr msg);
    void timer_callback();

    //void publish_output(double soc, double throttle, double brake);

    std::string truck_id_;
    std::string mode_;
    int pred_id_;
    int max_trucks_ = 3; 
    int last_flag = 0;

    BMS BMSObj;

    rclcpp::Publisher<ros2_msg::msg::TruckStatus>::SharedPtr status_pub_;
    //rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr soc_pub_;
    //rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_pub_;
    //rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Subscription<ros2_msg::msg::TruckCommand>::SharedPtr command_sub_;
    rclcpp::Subscription<ros2_msg::msg::TruckStatus>::SharedPtr predecessor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lane_change_end_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
   	rclcpp::Time last_sim_time_;
   	rclcpp::Time change_start_time_;
   	
	double soc_at_change_start_;
	bool is_measuring_change_ = false;
	const double step_interval_sec_ = 0.1;
		

    double lv_curr_speed_ = 0.0;
    double pos_front_ = 0.0;
    double vel_front_ = 0.0;
    double last_soc_= 100.0;
    double last_pos_= 0.0;
    double last_vel_= 0.0;
    bool mode_changed_ = false;
    rclcpp::Time start_time_;
};

