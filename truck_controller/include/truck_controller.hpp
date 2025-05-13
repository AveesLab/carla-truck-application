#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ros2_msg/msg/gnss.hpp> // GNSS 제거
#include <ros2_msg/msg/imu.hpp>  // IMU 제거
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp> // ENU 위치 수신용
#include <nav_msgs/msg/odometry.hpp>   // Odometry 발행용
#include <GeographicLib/LocalCartesian.hpp>
#include <sensor_msgs/msg/imu.hpp> // IMU 제거
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <string>
#include <cmath>
#include <limits>

// 컨트롤러 상태 정의
enum class ControllerState {
  INITIALIZING_HEADING, // 초기 헤딩 설정을 위해 직진 중
  RUNNING               // 일반 Pure Pursuit 주행 중
};

struct Point2D { double x, y; }; // 필요시 사용

struct Waypoint {
  double x, y, z; // ENU 좌표
};

class TruckController : public rclcpp::Node
{
public:
  TruckController(int argu_id);
  int actor_id_= 99;

private:
  // --- 주요 함수 ---
  void load_waypoints_0(const std::string &csv_path);
  void load_waypoints_1(const std::string &csv_path);
  // void gnss_callback(...); // 제거
  void compute_control(); // <<< 이 함수를 수정 (GPS 기반 제어)
  // void imu_callback(...); // 제거
  void publish_odom(double cur_x_, double cur_y_, double cur_z_, double yaw);
  // void gnss_to_carla(...); // load_waypoints 에서 사용한다면 유지
  void server_enu_callback(const geometry_msgs::msg::Point::SharedPtr msg); // 위치 업데이트 및 compute_control 호출
  double distSq(Point2D p1, Point2D p2);
  // --- Subscriber ---
  // rclcpp::Subscription<...>::SharedPtr sub_gnss_; // 제거
  // rclcpp::Subscription<...>::SharedPtr sub_imu_;   // 제거
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_server_enu_; // ENU 위치 구독용

  // --- Publishers ---
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vel_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_ENU_;

  // --- 상태 변수 ---
  std::vector<Waypoint> _waypoints_0;
  std::vector<Waypoint> _waypoints_1;
  std::vector<Waypoint> waypoints_;

  int current_wp_idx_;      // 생성자에서 0으로 초기화됨
  double cur_x_, cur_y_, cur_z_; // 현재 ENU 위치
  double prev_x_, prev_y_;      // 이전 ENU 위치 (헤딩 추정용)
  double current_yaw_;          // 현재 ENU Yaw (라디안, 추정/초기화됨, NaN으로 시작)
  double prev_steer_ = 0.0;     // 조향 스무딩용
  double start_x_, start_y_;    // Initial position for heading initialization phase
  ControllerState current_state_; // Controller's current state
  // --- 파라미터 ---
  GeographicLib::LocalCartesian geo_converter_; // Waypoint 로딩 시 필요할 수 있음
  double lookahead_dist_; // Kappa 계산 시 사용됨
  double max_speed_;
  double wheel_base_;     // 기본값, 실제 차량 모델 확인 필요
  double dist_threshold = 5;

  // --- 상수 ---
  const double max_steer_angle_rad = 0.7; // <<<=== 실제 차량 최대 조향각(라디안)으로 수정 필요!
  double init_dist_ = 5.0;      // 초기 직진 거리 (m) (파라미터화 가능)
  const double min_dist_threshold_sq = 0.01 * 0.01; // 헤딩 추정용 최소 이동 거리 제곱 (m^2)
  double init_speed_ = 6.0;     // INITIALIZING_HEADING 상태에서의 직진 속도 (파라미터화 가능)

};