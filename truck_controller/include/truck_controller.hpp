#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ros2_msg/msg/gnss.hpp>
#include <ros2_msg/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros2_msg/msg/truck_command.hpp>

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

struct Missionidx
{
    int start_idx;
    int finish_idx;   
};

class TruckController : public rclcpp::Node
{
public:
    TruckController(int argu_id);
    int actor_id_ = 99;
    int formation_id_ = 99;

private:
    // 상수 정의

    const double max_steer_angle_rad = 0.7;
    const double min_dist_threshold_sq = 0.01 * 0.01;
    const double TRUCK_LENGTH = 16.8;
    const double TRUCK_WIDTH = 2.5;
    const double SENSOR_TO_FRONT = 4.0;
    const double SENSOR_TO_REAR = 12.0;
    const double STEERING_THRESHOLD = 0.04;                  // 조향각 임계값

    const Missionidx Curve_idx_1 ={300-300,1800+300};
    const Missionidx Curve_idx_2 ={78100-300,79600+300};
    const Missionidx Curve_idx_3= {155900-300,157400+300};
    const Missionidx Curve_idx_4= {233250-300,235750+300};

    // const Missionidx Straight_idx_1 ={1800 + 7500,78100 - 7500};
    // const Missionidx Straight_idx_2 ={79600 + 7500,155900 - 7500};
    // const Missionidx Straight_idx_3 ={157400 + 7500,233250 - 7500}; 
    // const Missionidx Straight_idx_4 ={235750 + 7500,311000 - 6000};

    
    
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
    
    // 성능 측정 변수들
    long long loop_computation_time_us_;  // 반복문 연산 시간 (마이크로초)
    double loop_memory_usage_kb_;         // 반복문 메모리 사용량 (KB)
    
    // 파라미터
    double lookahead_dist_;
    double curve_lookahead_dist_;
    double straight_lookahead_dist_;
    double WHEEL_BASE_;
    double dist_threshold_;
    double min_gap_;
    double desired_gap_;
    double emergency_gap_;
    double ACC_SPEED_;
    double SLOW_SPEED_;
    double STABLE_SPEED_;
    
    double init_dist_ = 5.0;
    double init_speed_;
    bool deadband_flag_ = true; 

    // 차선변경 관련 플래그 
    bool formation_change_end_flag_ = false;
    bool start_lane_change_flag_ = true;
    bool doing_lane_change_flag_ = false;
    bool overrun_lane_change_flag_ = false;
    bool lane_change_flag_ = false;
    bool decrease_speed_flag_ = false;
    int start_lane_change_idx_;     //차선변경 시작 웨이포인트 인덱스
    int overrun_lane_change_idx_;       //overrun 차량 웨이포인트 인덱스

    double distance_to_leader_;
    double throttle_value_;

    // *changed
    int formation_change_flag_=0;

    // *changed
    double truck0_velocity_;
    double truck1_velocity_;
    double truck2_velocity_;

    // *changed
    bool truck0_overspeed_flag_;
    bool truck1_overspeed_flag_;
    bool truck2_overspeed_flag_;


    

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
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_lane_change_end_flag_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_reference_velocity_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_ready_;
    
    //*changed
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_formation_change_flag_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_formation_end_change_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_server_enu_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_truck0_pos_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_truck1_pos_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_truck2_pos_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_current_velocity_;
    rclcpp::Subscription<ros2_msg::msg::TruckCommand>::SharedPtr sub_formation_command_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_frame_;



    // *changed
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_truck0_velocity_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_truck1_velocity_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_truck2_velocity_;

    // 타이머 (주기적 제어용)
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback 함수들
    void formation_change_end_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void server_enu_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void truck0_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void truck1_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void truck2_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void formation_command_callback(const ros2_msg::msg::TruckCommand::SharedPtr msg);

    void current_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void truck0_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void truck1_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void truck2_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    bool check_stable_speeds();
    bool check_overspeed();

    void on_frame_tick(const std_msgs::msg::UInt32::SharedPtr msg);

    // Formation 관련 함수들
    int calculate_leader_truck_number();

    // 속도 제어 관련 함수들
    double get_reference_velocity(int fid, double distance_to_leader);
    double adjust_velocity_for_steering(double base_velocity, double steering);
    double calculate_pid_output(double current_vel, double target_vel, 
                              double kp, double ki, double kd);
    double normalize_control_output(double pid_output);
    double calculate_platoon_velocity(int current_fid, 
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
    void check_mission_state_(int cur_idx_);    

    void check_overrun();
    void check_lane_change_end();
    void update_formation_id();
    

};