#include "truck_controller.hpp"
#include <fstream>
#include <sstream>
#include <cmath>      // atan2, sqrt, isnan, M_PI 등
#include <limits>     // quiet_NaN, infinity
#include <algorithm>  // clamp, max, min
#include <stdexcept>

TruckController::TruckController()
: Node("truck_controller_node"),
  current_wp_idx_(18250) // 요청대로 시작 인덱스 초기화
{
  // --- 파라미터 선언 및 값 가져오기 ---
  this->declare_parameter<std::string>("csv_path", "");
  this->declare_parameter<double>("reference_lat", 0.0); // Waypoint 변환에 필요할 수 있음
  this->declare_parameter<double>("reference_lon", 0.0);
  this->declare_parameter<double>("reference_alt", 0.0);
  this->declare_parameter<double>("lookahead_dist", 10.0); // Kappa 계산 시 사용될 기본값
  this->declare_parameter<double>("max_speed", 5.0);      // 기본값
  this->declare_parameter<double>("wheel_base", 3.0);    // 기본값, 확인 필요
  // k_angular 파라미터는 현재 로직에서 사용 안 함

  std::string csv_path;
  double ref_lat, ref_lon, ref_alt;
  this->get_parameter("csv_path", csv_path);
  this->get_parameter("reference_lat", ref_lat);
  this->get_parameter("reference_lon", ref_lon);
  this->get_parameter("reference_alt", ref_alt);
  this->get_parameter("lookahead_dist", lookahead_dist_);
  this->get_parameter("max_speed", max_speed_);
  this->get_parameter("wheel_base", wheel_base_);
  // this->get_parameter("max_steer_rad", max_steer_angle_rad); // 파라미터로 받을 경우

  // --- 상태 변수 초기화 ---
  prev_x_ = std::numeric_limits<double>::quiet_NaN(); // 이전 위치 없음 표시
  prev_y_ = std::numeric_limits<double>::quiet_NaN();
  start_x_ = std::numeric_limits<double>::quiet_NaN(); // 시작 위치는 아직 모름
  start_y_ = std::numeric_limits<double>::quiet_NaN();
  current_yaw_ = 0.0; // 초기 Yaw는 우선 0 또는 경로 방향으로 설정 (직진 유도용)

  // ENU converter 초기화 (Waypoint 로딩에 필요 시)
  geo_converter_.Reset(ref_lat, ref_lon, ref_alt);

  // 웨이포인트 로드 (이 함수는 waypoints_ 멤버 변수를 ENU 좌표로 채운다고 가정)
  load_waypoints(csv_path);
  int actor_id_;

    // --- 초기 Yaw 설정 (경로 18250->18251 방향): 초기 직진 방향 가이드용 ---
    if (waypoints_.size() >= 2) {
      size_t start_idx = 18250; size_t next_idx = start_idx + 1;
      if (start_idx < waypoints_.size() && next_idx < waypoints_.size()) {
          double dx_init = waypoints_[next_idx].x - waypoints_[start_idx].x;
          double dy_init = waypoints_[next_idx].y - waypoints_[start_idx].y;
          if (std::abs(dx_init) > 1e-6 || std::abs(dy_init) > 1e-6) {
              current_yaw_ = std::atan2(dy_init, dx_init);
              RCLCPP_INFO(this->get_logger(), "Set initial yaw guidance for straight driving: %.3f rad", current_yaw_);
          } else { /* zero length segment */ }
      } else { /* index out of bounds */ }
  } else { /* not enough waypoints */ }
  // 만약 초기 Yaw가 0이면 그대로 0 유지




  // 초기 Yaw 설정 로직은 compute_control 첫 호출 시 수행됨
  std::string ns = "/truck" + std::to_string(actor_id_);
  // ROS 통신 설정 (namespace 덕분에 /truck0/gnss 등으로 구독합니다)

  pub_vel_ = this->create_publisher<std_msgs::msg::Float64>(ns + "/velocity_control", 10);
  pub_steer_ = this->create_publisher<std_msgs::msg::Float32>(ns + "/steer_control", 10);


  sub_server_enu_ = this->create_subscription<geometry_msgs::msg::Point>(
    ns + "/server/enu", 10,
    std::bind(&TruckController::server_enu_callback, this, std::placeholders::_1));


  pub_ENU_ = this->create_publisher<nav_msgs::msg::Odometry>(ns + "/ENU", 10 );

  RCLCPP_INFO(this->get_logger(), "TruckController node initialized (IMU-less GPS Steer Mode). Yaw will be initialized on first control cycle.");
} // 생성자 끝




// ENU 위치 수신 콜백: 위치 업데이트 후 제어 함수 호출
void TruckController::server_enu_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_ERROR(this->get_logger(), "Received null server ENU message pointer");
        return;
    }
    // 현재 위치 업데이트
    cur_x_ = msg->x;
    cur_y_ = msg->y;
    cur_z_ = msg->z;
    // 최초 콜백 시 시작 위치 기록 및 prev_pos 초기화
    if (std::isnan(start_x_)) {
      start_x_ = cur_x_;
      start_y_ = cur_y_;
      prev_x_ = cur_x_; // 첫 prev_pos는 시작 위치와 동일하게
      prev_y_ = cur_y_;
      RCLCPP_INFO(this->get_logger(), "Stored starting position: (%.2f, %.2f)", start_x_, start_y_);
    }
    // 제어 로직 호출
    compute_control();
}
// 상태 기반 제어 로직
void TruckController::compute_control()
{
    // 유효한 현재 위치 및 웨이포인트 필요
    if (std::isnan(cur_x_) || waypoints_.empty()) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for valid position or waypoints.");
        return;
    }

    // === 상태 머신 ===
    if (current_state_ == ControllerState::INITIALIZING_HEADING)
    {
        // 시작 위치가 기록되었는지 확인
        if (std::isnan(start_x_)) {
             RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for initial position record to start heading initialization.");
             // 안전 명령 발행
             std_msgs::msg::Float32 steer_msg; steer_msg.data = 0.0f; pub_steer_->publish(steer_msg);
             std_msgs::msg::Float64 vel_msg; vel_msg.data = 0.0; pub_vel_->publish(vel_msg);
             return;
        }

        // 시작점으로부터 실제 이동 거리 계산
        double dx_from_start = cur_x_ - start_x_;
        double dy_from_start = cur_y_ - start_y_;
        double dist_from_start = std::sqrt(dx_from_start * dx_from_start + dy_from_start * dy_from_start);

        RCLCPP_DEBUG(this->get_logger(), "State: INITIALIZING_HEADING. Dist moved = %.3f m", dist_from_start);

        // 목표 초기 이동 거리(init_dist_) 미만이면 계속 직진
        if (dist_from_start < init_dist_) {
            // --- 직진 명령 발행 ---
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Driving straight for heading init (%.2f / %.2f m). Publishing vel=%.2f, steer=0.0",
                                 dist_from_start, init_dist_, init_speed_);

            std_msgs::msg::Float32 steer_msg; steer_msg.data = 0.0f; pub_steer_->publish(steer_msg);
            std_msgs::msg::Float64 vel_msg; vel_msg.data = init_speed_; pub_vel_->publish(vel_msg);

            // Odometry 발행 (생성자에서 설정된 초기 가이드 Yaw 사용)
            if (!std::isnan(current_yaw_)) { publish_odom(cur_x_, cur_y_, cur_z_, current_yaw_); }

            // 다음 계산 위해 이전 위치 업데이트
            prev_x_ = cur_x_;
            prev_y_ = cur_y_;
            return; // 상태 유지하며 현재 제어 주기 종료

        } 
        else {
            // --- 목표 거리 도달: 생성자에서 설정한 Yaw 값으로 RUNNING 상태 시작 ---
            RCLCPP_INFO(this->get_logger(), "Initial distance %.2f m reached. Using constructor-initialized yaw (%.3f rad) to start RUNNING state.", init_dist_, current_yaw_);


            // 생성자에서 설정된 current_yaw_ 값이 유효한지 최종 확인
            if (std::isnan(current_yaw_)) {
                 RCLCPP_ERROR(this->get_logger(), "Initial yaw from constructor was NaN! Setting to 0.");

            }

            // 상태를 RUNNING으로 변경
            current_state_ = ControllerState::RUNNING;
            RCLCPP_INFO(this->get_logger(), "Switching state to RUNNING.");

            // 첫 RUNNING 계산을 위해 prev_pos 업데이트
            prev_x_ = cur_x_;
            prev_y_ = cur_y_;

            // 아래 RUNNING 로직으로 바로 넘어감 (Fall through)
        }
    } // INITIALIZING_HEADING 상태 끝


    // === RUNNING 상태 로직 ===
    if (current_state_ == ControllerState::RUNNING)
    {
        // --- 1. 헤딩 추정 (이동량 기반) ---
        bool heading_updated_this_cycle = false;
        if (!std::isnan(prev_x_) && !std::isnan(prev_y_)) { // 이전 위치 유효 확인
            double dx_move = cur_x_ - prev_x_;
            double dy_move = cur_y_ - prev_y_;
            double dist_moved_sq = dx_move * dx_move + dy_move * dy_move;

            if (dist_moved_sq > min_dist_threshold_sq) { // 충분히 움직였으면 업데이트
                current_yaw_ = std::atan2(dy_move, dx_move);
                heading_updated_this_cycle = true;
                RCLCPP_DEBUG(this->get_logger(), "Yaw updated from movement: %.3f", current_yaw_);
            } else { // 움직임 작으면 이전 값 유지
                RCLCPP_DEBUG(this->get_logger(), "Movement too small, keeping yaw: %.3f", current_yaw_);
                 heading_updated_this_cycle = !std::isnan(current_yaw_); // 이전에 유효했으면 계속 유효
            }
        } else {
            // 이 상태에서는 prev_x_가 NaN이면 안 됨 (로직 오류 가능성)
            RCLCPP_ERROR(this->get_logger(), "prev_x_ or prev_y_ is NaN in RUNNING state!");
             heading_updated_this_cycle = !std::isnan(current_yaw_); // 현재 yaw 유효성만 믿음
        }

        // 다음 계산 위해 이전 위치 업데이트 (여기서 업데이트해야 위에서 사용 가능)
        prev_x_ = cur_x_;
        prev_y_ = cur_y_;

        // 헤딩 유효성 최종 확인
        if (std::isnan(current_yaw_)) {
             RCLCPP_WARN_ONCE(this->get_logger(), "Current yaw is NaN in RUNNING state. Skipping control.");
             return;
        }
        // --- 헤딩 추정 끝 ---


        // --- 2. Pure Pursuit 로직 (이전과 동일) ---
        double heading = current_yaw_;

        // 가장 가까운 웨이포인트 찾기 (Window 사용)
        // ... (nearest_idx_pp 계산 로직 - 이전 답변 참고) ...
         int window_pp = 100; int start_pp = std::max<int>(current_wp_idx_ - window_pp, 0); 
         int end_pp = std::min<int>(current_wp_idx_ + window_pp, (int)waypoints_.size() - 1); int nearest_idx_pp = current_wp_idx_;
         if (start_pp <= end_pp && start_pp < (int)waypoints_.size() && end_pp >= 0) 
         { 
          double min_d2_pp = std::numeric_limits<double>::infinity(); 
          for (int i = start_pp; i <= end_pp; ++i)
           { 
            if (i < 0 || i >= (int)waypoints_.size()) continue;
            double dx_wp = waypoints_[i].x - cur_x_;
            double dy_wp = waypoints_[i].y - cur_y_;
            double d2 = dx_wp * dx_wp + dy_wp * dy_wp;
            if (d2 < min_d2_pp)
            { 
              min_d2_pp = d2; nearest_idx_pp = i;
            }
           } 
           if (nearest_idx_pp < 0 || nearest_idx_pp >= (int)waypoints_.size())
            { 
              nearest_idx_pp = current_wp_idx_;
             } 
          } 
          if (nearest_idx_pp < 0 || nearest_idx_pp >= (int)waypoints_.size()) nearest_idx_pp = 0;
         current_wp_idx_ = nearest_idx_pp;

          // --- [수정됨] 목표점 찾기 (Lookahead Distance 기반 기하학적 탐색) ---
          Point2D lookahead_point = {0, 0}; // 최종 목표 지점 좌표 (x, y)
          bool found_lookahead = false;    // 목표점 찾기 성공 여부
          double lookahead_dist_sq = lookahead_dist_ * lookahead_dist_; // 목표 거리 제곱 (계산 최적화)

          // 가장 가까운 웨이포인트 인덱스(nearest_idx_pp)부터 경로를 따라 탐색 시작
          size_t search_idx = nearest_idx_pp;

          for (size_t i = 0; i < waypoints_.size(); ++i) { // 경로 전체를 최대 한 바퀴 탐색
              size_t p1_idx = search_idx;                      // 현재 탐색 선분의 시작점 인덱스
              size_t p2_idx = (search_idx + 1) % waypoints_.size(); // 현재 탐색 선분의 끝점 인덱스 (순환 처리)

              Point2D p1 = {waypoints_[p1_idx].x, waypoints_[p1_idx].y}; // 선분 시작점 좌표
              Point2D p2 = {waypoints_[p2_idx].x, waypoints_[p2_idx].y}; // 선분 끝점 좌표
              Point2D segment_vec = {p2.x - p1.x, p2.y - p1.y};        // 선분 방향 벡터 v

              // 차량 현재 위치(current_pos = {cur_x_, cur_y_}) 중심, 반지름 lookahead_dist_ 인 원과
              // 선분 p1 -> p2 의 교점 t 찾기 (이차방정식 활용: a*t^2 + b*t + c = 0)
              Point2D d_vec = {p1.x - cur_x_, p1.y - cur_y_};              // p1 - current_pos 벡터 d
              double a = segment_vec.x * segment_vec.x + segment_vec.y * segment_vec.y; // dot(v,v)
              double b = 2 * (d_vec.x * segment_vec.x + d_vec.y * segment_vec.y);       // 2*dot(d,v)
              double c = d_vec.x * d_vec.x + d_vec.y * d_vec.y - lookahead_dist_sq;    // dot(d,d) - L^2
              double discriminant = b * b - 4 * a * c;                     // 판별식

              if (discriminant >= 0 && a > 1e-9) { // 실근이 존재하고 선분 길이가 0이 아닐 때
                  double sqrt_discriminant = std::sqrt(discriminant);
                  double t1 = (-b - sqrt_discriminant) / (2 * a); // 해 1
                  double t2 = (-b + sqrt_discriminant) / (2 * a); // 해 2

                  // 선분 내(0 <= t <= 1)에 있는 교점 t 찾기 (더 먼 쪽 우선)
                  double t_intersect = -1.0; // 유효한 교점 t 값 초기화
                  if (t2 >= 0.0 && t2 <= 1.0) {
                      t_intersect = t2;
                  } else if (t1 >= 0.0 && t1 <= 1.0) {
                      t_intersect = t1;
                  }

                  if (t_intersect >= 0.0) { // 유효한 교점 찾음
                      // 교점의 실제 좌표 계산
                      lookahead_point.x = p1.x + t_intersect * segment_vec.x;
                      lookahead_point.y = p1.y + t_intersect * segment_vec.y;
                      found_lookahead = true; // 찾기 성공 플래그 설정
                      RCLCPP_DEBUG(this->get_logger(), "Found lookahead point on segment %zu -> %zu at t=%.3f", p1_idx, p2_idx, t_intersect);
                      break; // 찾았으면 탐색 루프 종료
                  }
              }
              // 다음 선분으로 이동
              search_idx = p2_idx;
          } // for 루프 끝

          if (!found_lookahead) { // 모든 선분을 탐색했는데도 교점을 못 찾은 경우
              RCLCPP_WARN(this->get_logger(), "Lookahead point not found via intersection, using fallback logic.");
              // 대체 로직 사용 (예: 가장 가까운 경로 선분 방향으로 lookahead_dist 만큼 떨어진 점)
              size_t next_idx_fb = (nearest_idx_pp + 1) % waypoints_.size();
              Point2D seg_start_fb = {waypoints_[nearest_idx_pp].x, waypoints_[nearest_idx_pp].y};
              Point2D seg_end_fb = {waypoints_[next_idx_fb].x, waypoints_[next_idx_fb].y};
              Point2D seg_dir_fb = {seg_end_fb.x - seg_start_fb.x, seg_end_fb.y - seg_start_fb.y};
              double seg_mag_fb = std::sqrt(distSq(seg_start_fb, seg_end_fb)); // 이전 답변의 distSq 함수 필요
              if (seg_mag_fb > 1e-6){ seg_dir_fb.x /= seg_mag_fb; seg_dir_fb.y /= seg_mag_fb; } // 단위 벡터화
              else { seg_dir_fb = {std::cos(heading), std::sin(heading)}; } // 방향 모르면 현재 헤딩 방향 사용
              // 현재 위치에서 경로 방향으로 lookahead_dist 만큼 떨어진 점 (근사치)
              lookahead_point = {cur_x_ + lookahead_dist_ * seg_dir_fb.x,
                                  cur_y_ + lookahead_dist_ * seg_dir_fb.y};
              // found_lookahead = true; // 대체 로직으로라도 찾았다고 처리
          }
          // --- [수정됨] 목표점 찾기 끝 ---


          // --- 조향각 계산 (Pure Pursuit 스타일) ---
          // *** 중요: target_idx 대신 계산된 lookahead_point 사용 ***
          double dx_target = lookahead_point.x - cur_x_; // 목표점 좌표 사용
          double dy_target = lookahead_point.y - cur_y_; // 목표점 좌표 사용
          double path_angle = std::atan2(dy_target, dx_target); // 목표 각도 (라디안)

          double alpha = path_angle - heading; // 각도 오차 (라디안)

          // alpha 정규화 [-pi, pi]
          while (alpha > M_PI)   alpha -= 2 * M_PI;
          while (alpha < -M_PI)  alpha += 2 * M_PI;

          // 곡률 및 원시 조향각 계산
          // 분모에 실제 lookahead 거리를 사용하는 것이 더 정확할 수 있음
          double actual_lookahead_dist = std::sqrt(dx_target * dx_target + dy_target * dy_target);
          double kappa = 0.0;
          if (actual_lookahead_dist > 0.1) { // 0 나누기 방지
              kappa = 2.0 * std::sin(alpha) / actual_lookahead_dist; // 실제 거리 사용
              // 또는 파라미터 사용: kappa = 2.0 * std::sin(alpha) / lookahead_dist_;
          }


          double raw_steer = std::atan(wheel_base_ * kappa); // 라디안

        // 조향각 스무딩
        // ... (now_steer 계산 로직 - 이전 답변 참고) ...
         double smooth_factor = 0.8;
          double now_steer = smooth_factor * prev_steer_ + (1.0 - smooth_factor) * raw_steer;
          prev_steer_ = now_steer;


        // 속도 제어 (max_speed_ 사용, 감속 옵션)
        // ... (target_velocity 계산 로직 - 이전 답변 참고) ...
         double target_velocity = max_speed_;
         // Optional: target_velocity = max_speed_ * (1.0 - 0.8 * std::abs(norm_steer));

        // 명령 발행 (부호 확인!)
        // ... (steer_msg, vel_msg 발행 로직 - 이전 답변 참고) ...
         std_msgs::msg::Float32 steer_msg;
          steer_msg.data = static_cast<float>(now_steer * -30); /* <<< 부호 확인! */
         if(abs(steer_msg.data)>0.04) target_velocity = 0.1;

         pub_steer_->publish(steer_msg);
         std_msgs::msg::Float64 vel_msg;
        vel_msg.data = target_velocity;
        pub_vel_->publish(vel_msg);

        // Odometry 발행 및 로깅
        publish_odom(cur_x_, cur_y_, cur_z_, heading);
        RCLCPP_INFO(this->get_logger(), "[RUNNING] Head: %.3f | TargetAng: %.3f | Alpha: %.3f | Steer: %.3f",
                 heading, path_angle, alpha, now_steer);
                 std::cout<< "vel : "<<target_velocity<<std::endl;

    } // RUNNING 상태 끝
} // compute_control 끝

void TruckController::load_waypoints(const std::string &csv_path)
{
  std::ifstream ifs(csv_path);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file: %s", csv_path.c_str());
    rclcpp::shutdown(); // 또는 다른 오류 처리 로직
    return;
  }

  waypoints_.clear(); // 기존 웨이포인트 비우기
  std::string line;
  std::getline(ifs, line); // 헤더 라인 스킵

  Waypoint wp;
  int line_count = 1;
  while (std::getline(ifs, line)) {
    line_count++;
    std::istringstream ss(line);
    char comma; // 콤마 무시용

    // CSV 형식이 "x,y,z" 라고 가정 (쉼표로 구분)
    if (ss >> wp.x >> comma >> wp.y >> comma >> wp.z) {
        waypoints_.push_back(wp);
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to parse waypoint line %d: %s", line_count, line.c_str());
    }
}
}

// publish_odom 함수 구현 (기존과 동일)
void TruckController::publish_odom(double cur_x_, double cur_y_, double cur_z_, double yaw) {
  auto msg = nav_msgs::msg::Odometry();
  msg.header.stamp = this->now();
  msg.header.frame_id = "map"; // or "odom"
  msg.pose.pose.position.x = cur_x_;
  msg.pose.pose.position.y = cur_y_;
  msg.pose.pose.position.z = cur_z_;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  msg.pose.pose.orientation = tf2::toMsg(q);
  pub_ENU_->publish(msg);
}

// Point2D 거리 제곱 함수
double TruckController::distSq(Point2D p1, Point2D p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return dx * dx + dy * dy;
}