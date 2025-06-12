#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ros2_msg/msg/gnss.hpp>
#include <ros2_msg/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <array>

// 컨트롤러 상태 정의
enum class ControllerState {
    INITIALIZING_HEADING,
    RUNNING
};

struct Point2D { double x, y; };

struct Waypoint {
    double x, y, z;
};

class TruckController : public rclcpp::Node
{
public:
    TruckController(int argu_id);
    int actor_id_ = 99;
    int formation_id_ = 99;

private:
    // 상수 정의
    const double MIN_GAP = 8.0;       // 최소 간격 (미터)
    const double DESIRED_GAP = 10.0;   // 희망 간격 (미터)
    const double EMERGENCY_GAP = 5.0;
    const double max_steer_angle_rad = 0.7;
    const double min_dist_threshold_sq = 0.01 * 0.01;
    const double TRUCK_LENGTH = 16.8;
    const double TRUCK_WIDTH = 2.5;
    const double SENSOR_TO_FRONT = 4.0;
    const double SENSOR_TO_REAR = 12.0;
    const double STEERING_THRESHOLD = 0.04;                  // 조향각 임계값
    const double ACC_SPEED = 100.0;
    const double SLOW_SPEED = 30.0;
    const double STABLE_SPEED = 70.0;
    
    
    // 멤버 변수
    //int actor_id_;                     // 트럭 ID
    //int formation_id_;                 // Formation ID
    int formation_change_count_;       // Formation 변경 횟수
    int current_wp_idx_;              // 현재 웨이포인트 인덱스
    int lane_number_;
    // 위치 관련 변수
    double cur_x_, cur_y_, cur_z_;    // 현재 위치
    double prev_x_, prev_y_;          // 이전 위치
    double start_x_, start_y_;        // 시작 위치
    double current_yaw_;              // 현재 방향
    double prev_steer_;               // 이전 조향각
    
    // 속도 제어 관련 변수
    double current_velocity_;          // 현재 속도
    double target_velocity_;          // 목표 속도
    double prev_velocity_;            // 이전 속도
    double current_steering_;         // 현재 조향각
    double prev_error_;               // PID 이전 오차
    double integral_;                 // PID 적분값
    double dt_;                       // PID 시간 간격
    double prev_velocity_error_;      // 속도 제어 이전 오차
    double integral_velocity_error_;  // 속도 제어 적분값
    
    // 파라미터
    double lookahead_dist_;
    double max_speed_;
    double wheel_base_;
    double dist_threshold_;
    double max_accel_;
    double min_gap_ = MIN_GAP;
    double desired_gap_ = DESIRED_GAP;
    double emergency_gap_ = EMERGENCY_GAP;
    double time_gap_;
    double init_dist_ = 5.0;
    double init_speed_;
    const double control_dt_ = 0.1;

    

    // 웨이포인트 관련
    std::vector<Waypoint> _waypoints_0;
    std::vector<Waypoint> _waypoints_1;
    std::vector<Waypoint> waypoints_;
    
    // Formation 관리
    std::array<geometry_msgs::msg::Point, 3> truck_positions_;
    ControllerState current_state_;
    GeographicLib::LocalCartesian geo_converter_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vel_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_ENU_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_formation_change_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_server_enu_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_truck0_pos_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_truck1_pos_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_truck2_pos_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_current_velocity_;

    // Callback 함수들
    void formation_change_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void server_enu_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void truck0_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void truck1_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void truck2_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void current_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);

    // Formation 관련 함수들
    void update_formation_id();
    int calculate_leader_truck_number();

    // 속도 제어 관련 함수들
    double get_reference_velocity(int fid, double distance_to_leader);
    double adjust_velocity_for_steering(double base_velocity, double steering);
    double calculate_pid_output(double current_vel, double target_vel, 
                              double kp, double ki, double kd);
    double normalize_control_output(double pid_output);
    double calculate_platoon_velocity(double current_fid, 
                                    double distance_to_leader,
                                    double current_velocity,
                                    double current_steering);
    double get_distance_to_leader();

    // 유틸리티 함수들
    void compute_control();
    void load_waypoints_0(const std::string &csv_path);
    void load_waypoints_1(const std::string &csv_path);
    double distSq(Point2D p1, Point2D p2);
    //double calculate_target_velocity(double steer_angle);
    void publish_odom(double cur_x_, double cur_y_, double cur_z_, double yaw);
};