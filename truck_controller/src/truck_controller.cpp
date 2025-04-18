#include "truck_controller.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <cfloat>

TruckController::TruckController()
: Node("truck_controller_node"),
  current_wp_idx_(0)
{
  // declare & get parameters
  this->declare_parameter<std::string>("csv_path", "");
  this->declare_parameter<double>("reference_lat", 0.0);
  this->declare_parameter<double>("reference_lon", 0.0);
  this->declare_parameter<double>("reference_alt", 0.0);
  this->declare_parameter<double>("lookahead_dist", 5.0);
  this->declare_parameter<double>("max_speed", 5.0);
  this->declare_parameter<double>("k_angular", 1.0);

  std::string csv_path;
  double ref_lat, ref_lon, ref_alt;
  this->get_parameter("csv_path", csv_path);
  this->get_parameter("reference_lat", ref_lat);
  this->get_parameter("reference_lon", ref_lon);
  this->get_parameter("reference_alt", ref_alt);
  this->get_parameter("lookahead_dist", lookahead_dist_);
  this->get_parameter("max_speed", max_speed_);
  this->get_parameter("k_angular", k_angular_);

  // ENU converter 초기화
  geo_converter_.Reset(ref_lat, ref_lon, ref_alt);

  // 웨이포인트 로드
  load_waypoints(csv_path);
  RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", waypoints_.size());

  std::string ns = "/truck" + std::to_string(actor_id_);

  // ROS 통신 설정 (namespace 덕분에 /truck0/gnss 등으로 구독합니다)
  sub_gnss_ = this->create_subscription<ros2_msg::msg::GNSS>(
    ns + "/gnss", 10,
    std::bind(&TruckController::gnss_callback, this, std::placeholders::_1)
  );
  pub_vel_ = this->create_publisher<std_msgs::msg::Float64>(ns + "/velocity_control", 10);
  pub_steer_ = this->create_publisher<std_msgs::msg::Float32>(ns + "/steer_control", 10);
}

void TruckController::load_waypoints(const std::string &csv_path)
{
  std::ifstream ifs(csv_path);
  std::string line;
  std::getline(ifs, line);  // 헤더 스킵
  Waypoint wp;
  while (std::getline(ifs, line)) {
    std::istringstream ss(line);
    ss >> wp.x >> wp.y >> wp.z;
    waypoints_.push_back(wp);
  }
}

void TruckController::gnss_callback(const ros2_msg::msg::GNSS::SharedPtr msg) {
  if (!origin_initialized_) {
    ref_lat_ = msg->latitude;
    ref_lon_ = msg->longitude;
    ref_alt_ = msg->altitude;
    geo_converter_.Reset(ref_lat_, ref_lon_, ref_alt_);
    origin_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Origin 초기화: lat=%.6f lon=%.6f alt=%.2f",
                ref_lat_, ref_lon_, ref_alt_);
    return;  // 이 콜백은 변환 전이라 리턴
  }

  geo_converter_.Forward(
    msg->latitude, msg->longitude, msg->altitude,
    cur_x_, cur_y_, cur_z_);
  compute_control();
}

void TruckController::compute_control()
{
  if (waypoints_.empty()) return;

  // 1) 윈도우 크기 정의 (±50포인트)
  const size_t W = 50;
  size_t start = (current_wp_idx_ > W ? current_wp_idx_ - W : 0);
  size_t end   = std::min(current_wp_idx_ + W, waypoints_.size() - 1);

  // 2) 윈도우 내에서 nearest 검색
  size_t nearest_idx = start;
  double min_d2 = 1e12;
  for (size_t i = start; i <= end; ++i) {
    double dx = waypoints_[i].x - cur_x_;
    double dy = waypoints_[i].y - cur_y_;
    double d2 = dx*dx + dy*dy;
    if (d2 < min_d2 && i >= current_wp_idx_) { // 뒤로 돌아가지 않도록
      min_d2 = d2;
      nearest_idx = i;
    }
  }
  double nearest_dist = std::sqrt(min_d2);

  // 3) lookahead 거리에 있는 target 찾기 (accumulate)
  size_t target_idx = nearest_idx;
  double acc = 0.0;
  while (target_idx + 1 < waypoints_.size()) {
    double dx = waypoints_[target_idx+1].x - waypoints_[target_idx].x;
    double dy = waypoints_[target_idx+1].y - waypoints_[target_idx].y;
    double seg = std::hypot(dx, dy);
    if (acc + seg > lookahead_dist_) break;
    acc += seg;
    ++target_idx;
  }

  const size_t MAX_OFFSET = 10;  // nearest_idx 기준 최대 몇 포인트 앞까지만 허용
  if (target_idx > nearest_idx + MAX_OFFSET) {
    target_idx = nearest_idx + MAX_OFFSET;
    // acc도 실제 거리로 재계산 (optional)
    acc = 0.0;
    for (size_t i = nearest_idx; i < target_idx; ++i) {
      double dx = waypoints_[i+1].x - waypoints_[i].x;
      double dy = waypoints_[i+1].y - waypoints_[i].y;
      acc += std::hypot(dx, dy);
    }
  }

  // 4) Pure Pursuit 각도 계산
  double dx = waypoints_[target_idx].x - cur_x_;
  double dy = waypoints_[target_idx].y - cur_y_;
  double alpha = std::atan2(dy, dx);

  double raw_steer = k_angular_ * alpha;

  // 5) 조향 스무딩 (간단한 1차 IIR 필터)
  //    new_steer = 0.8 * old_steer + 0.2 * raw_steer
  static double prev_steer = 0.0;
  double now_steer = 0.8 * prev_steer + 0.2 * raw_steer;
  prev_steer = now_steer;

  double norm_steer = std::clamp(now_steer / 0.7, -1.0, 1.0);


  // 7) steer 메시지 (Float64 → topic 에 맞추세요)
  std_msgs::msg::Float32 steer_msg;
  steer_msg.data = norm_steer;
  

  // 6) throttle 메시지
  std_msgs::msg::Float64 vel_msg;
  vel_msg.data = max_speed_;


  pub_vel_->publish(vel_msg);
  pub_steer_->publish(steer_msg);

  // 8) current_wp_idx_ 업데이트
  current_wp_idx_ = nearest_idx;

  RCLCPP_INFO(this->get_logger(),
    "nearest=%zu (%.2fm) → target=%zu(acc=%.2fm) | steer=%.3f vel=%.2f",
    nearest_idx, nearest_dist, target_idx, acc, norm_steer, vel_msg.data);
}
