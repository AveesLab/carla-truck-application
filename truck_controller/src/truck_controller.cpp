#include "truck_controller.hpp"
#include <fstream>
#include <sstream>
#include <cmath>      // atan2, sqrt, isnan, M_PI 등
#include <limits>     // quiet_NaN, infinity
#include <algorithm>  // clamp, max, min
#include <stdexcept>
#include <string>
#include <array>

TruckController::TruckController(int argu_id)
: Node("truck_controller_node_" + std::to_string(argu_id)),
  current_wp_idx_(0), // 요청대로 시작 인덱스 초기화
  actor_id_(argu_id),
  truck_positions_(), // array는 기본 생성자로 초기화
  current_state_(ControllerState::INITIALIZING_HEADING),  // 상태 초기화 추가
  current_velocity_(0.0),  // 현재 속도 초기화
  init_speed_(10.0),
  dist_threshold_(5.0),    // 웨이포인트 탐색 거리 임계값
  init_dist_(2.0),         // 초기 직진 주행 거리
  dt_(0.02),               // 제어 주기 (50Hz)
  integral_(0.0),          // PID 적분항
  prev_error_(0.0),         // PID 이전 오차
  lane_number_(0)
{

  // Formation ID 설정
  if(actor_id_ == 0) formation_id_ = 0;
  else if(actor_id_ == 1) formation_id_ = 1;
  else if(actor_id_ == 2) formation_id_ = 2;
  else {
    //RCLCPP_ERROR(this->get_logger(), "Invalid actor_id: %d", actor_id_);
    throw std::runtime_error("Invalid actor_id");
  }
  
  if(actor_id_ == 1) current_wp_idx_ = 0;
  if(actor_id_ == 2) current_wp_idx_ = 0;

  // --- 파라미터 선언 및 값 가져오기 ---
  this->declare_parameter<std::string>("csv_path", "");
  this->declare_parameter<double>("lookahead_dist"); 
  //this->declare_parameter<double>("max_speed");     
  this->declare_parameter<double>("wheel_base");    

  // 군집 주행 관련 파라미터 선언
  //this->declare_parameter<double>("desired_gap", 20.0);  // 기본 20m
  //this->declare_parameter<double>("min_gap", 10.0);      // 최소 안전거리 10m
  this->declare_parameter<double>("time_gap", 2.0);      // 시간 간격 2초
  //this->declare_parameter<double>("max_accel", 2.0);     // 최대 가속도 2m/s^2

  std::string csv_path;

  this->get_parameter("csv_path", csv_path);
  this->get_parameter("lookahead_dist", lookahead_dist_);
  this->get_parameter("max_speed", max_speed_);
  this->get_parameter("wheel_base", wheel_base_);

  // 파라미터 가져오기
  //this->get_parameter("desired_gap", desired_gap_);
  //this->get_parameter("min_gap", min_gap_);
  this->get_parameter("time_gap", time_gap_);
  //this->get_parameter("max_accel", max_accel_);

  // --- 상태 변수 초기화 ---
  prev_x_ = std::numeric_limits<double>::quiet_NaN(); // 이전 위치 없음 표시
  prev_y_ = std::numeric_limits<double>::quiet_NaN();
  start_x_ = std::numeric_limits<double>::quiet_NaN(); // 시작 위치는 아직 모름
  start_y_ = std::numeric_limits<double>::quiet_NaN();
  current_yaw_ = 0.0; // 초기 Yaw는 우선 0 또는 경로 방향으로 설정 (직진 유도용)

  // 웨이포인트 로드 (이 함수는 waypoints_ 멤버 변수를 ENU 좌표로 채운다고 가정)
  load_waypoints_0(csv_path);
  load_waypoints_1(csv_path);
  waypoints_ = _waypoints_0;

  // --- 초기 Yaw 설정 (경로 0->1 방향): 초기 직진 방향 가이드용 ---
  if (waypoints_.size() >= 2) {
    size_t start_idx = 0; size_t next_idx = start_idx + 1;
    if (start_idx < waypoints_.size() && next_idx < waypoints_.size()) {
        double dx_init = waypoints_[next_idx].x - waypoints_[start_idx].x;
        double dy_init = waypoints_[next_idx].y - waypoints_[start_idx].y;
        if (std::abs(dx_init) > 1e-6 || std::abs(dy_init) > 1e-6) {
            current_yaw_ = std::atan2(dy_init, dx_init);
            //RCLCPP_INFO(this->get_logger(), "Set initial yaw guidance for straight driving: %.3f rad", current_yaw_);
        } else { /* zero length segment */ }
    } else { /* index out of bounds */ }
  } else { /* not enough waypoints */ }

  // 초기 Yaw 설정 로직은 compute_control 첫 호출 시 수행됨
  std::string ns = "/truck" + std::to_string(actor_id_);
  if(actor_id_ == 99) 
  {
    while(1)
    {
        //RCLCPP_INFO_STREAM(this->get_logger(),"No get NameSpace(argument) Please Re-start");
    }
  }

  // ROS 통신 설정 (namespace 덕분에 /truck0/gnss 등으로 구독합니다)

  pub_vel_ = this->create_publisher<std_msgs::msg::Float64>(ns + "/velocity_control", 10);
  pub_steer_ = this->create_publisher<std_msgs::msg::Float32>(ns + "/steer_control", 10);
  pub_ENU_ = this->create_publisher<nav_msgs::msg::Odometry>(ns + "/ENU", 10 );

  // 자신의 위치 업데이트를 위한 구독
  sub_server_enu_ = this->create_subscription<geometry_msgs::msg::Point>(
    ns + "/server/enu", 10,
    std::bind(&TruckController::server_enu_callback, this, std::placeholders::_1));

  // 모든 트럭의 위치 구독
  std::string truck0_topic = "/truck0/server/enu";
  std::string truck1_topic = "/truck1/server/enu";
  std::string truck2_topic = "/truck2/server/enu";
  
  sub_truck0_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
      truck0_topic, 10,
      std::bind(&TruckController::truck0_pos_callback, this, std::placeholders::_1));
  sub_truck1_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
      truck1_topic, 10,
      std::bind(&TruckController::truck1_pos_callback, this, std::placeholders::_1));
  sub_truck2_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
      truck2_topic, 10,
      std::bind(&TruckController::truck2_pos_callback, this, std::placeholders::_1));
      
  // 현재 속도 구독 추가
  sub_current_velocity_ = this->create_subscription<std_msgs::msg::Float32>(
      ns + "/velocity", 10,
      std::bind(&TruckController::current_velocity_callback, this, std::placeholders::_1));
      
  // Formation 변경 구독
  sub_formation_change_ = this->create_subscription<std_msgs::msg::Empty>(
      "/formation_change", 10,
      std::bind(&TruckController::formation_change_callback, this, std::placeholders::_1));
      
  //RCLCPP_INFO(this->get_logger(), "TruckController node initialized (IMU-less GPS Steer Mode). Yaw will be initialized on first control cycle.");



} // 생성자 끝

// ENU 위치 수신 콜백: 위치 업데이트 후 제어 함수 호출
void TruckController::server_enu_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    if (!msg || std::isnan(msg->x) || std::isnan(msg->y) || std::isnan(msg->z)) {
        //RCLCPP_ERROR(this->get_logger(), "Received invalid ENU position data");
        return;
    }

    // 급격한 위치 변화 감지
    if (!std::isnan(cur_x_)) {  // 첫 데이터가 아닌 경우
        double dx = msg->x - cur_x_;
        double dy = msg->y - cur_y_;
        double dist = std::sqrt(dx*dx + dy*dy);

    }

    // 현재 위치 업데이트
    cur_x_ = msg->x;
    cur_y_ = msg->y;
    cur_z_ = msg->z;

    // 최초 콜백 시 시작 위치 기록 및 prev_pos 초기화
    if (std::isnan(start_x_)) {
        start_x_ = cur_x_;
        start_y_ = cur_y_;
        prev_x_ = cur_x_;
        prev_y_ = cur_y_;
       // RCLCPP_INFO(this->get_logger(), "Stored starting position: (%.2f, %.2f)", start_x_, start_y_);
    }

    // 제어 로직 호출
    compute_control();
}




// // 조향각과 거리 기반 속도 계산
// double TruckController::calculate_target_velocity(double steer_angle) {
//     // km/h를 m/s로 변환
//     const double BASE_SPEED = 60.0 * (1000.0 / 3600.0);     // 60 km/h -> 16.67 m/s
//     const double SLOW_SPEED = 30.0 * (1000.0 / 3600.0);     // 30 km/h -> 8.33 m/s
//     const double CATCH_UP_SPEED = 70.0 * (1000.0 / 3600.0); // 70 km/h -> 19.44 m/s
//     const double STEERING_THRESHOLD = 0.04;

//     // 선행 차량과의 거리 계산
//     double distance_to_leader = 0.0;
//     int leader_id = formation_id_ - 1;  // 현재 차량의 formation_id에서 1을 뺀 값이 선행 차량의 ID
//     double target_velocity;
    
//     if (leader_id >= 0) {  // 선두 차량이 아닌 경우
//         const auto& leader_pos = truck_positions_[leader_id];
//         double dx = leader_pos.x - cur_x_;
//         double dy = leader_pos.y - cur_y_;
//         distance_to_leader = std::sqrt(dx*dx + dy*dy);

//         // 거리에 따른 속도 결정
//         if (distance_to_leader < min_gap_) {
//             target_velocity = SLOW_SPEED;  // 30 km/h
//             RCLCPP_DEBUG(this->get_logger(), "Close to leader (%.2f m), reducing speed to %.2f m/s", 
//                         distance_to_leader, SLOW_SPEED);
//         } else if (distance_to_leader > desired_gap_) {
//             target_velocity = CATCH_UP_SPEED;  // 70 km/h
//             RCLCPP_DEBUG(this->get_logger(), "Far from leader (%.2f m), increasing speed to %.2f m/s", 
//                         distance_to_leader, CATCH_UP_SPEED);
//         } else {
//             target_velocity = BASE_SPEED;  // 60 km/h
//             RCLCPP_DEBUG(this->get_logger(), "Maintaining base speed %.2f m/s", BASE_SPEED);
//         }
//     } else {
//         // 선두 차량은 기본 속도로 주행
//         target_velocity = BASE_SPEED;  // 60 km/h
//         RCLCPP_DEBUG(this->get_logger(), "Lead truck, maintaining base speed %.2f m/s", BASE_SPEED);
//     }
    
//     // 조향각이 임계값보다 크면 속도 감소
//     if (std::abs(steer_angle) > STEERING_THRESHOLD) {
//         target_velocity *= (1.0/3.0);  // 속도를 1/3로 감소
//         RCLCPP_DEBUG(this->get_logger(), "High steering angle (%.3f), reducing speed to %.2f m/s", 
//                     steer_angle, target_velocity);
//     }
    
//     // 최대 속도 제한
//     //target_velocity = std::min(target_velocity, max_speed_);
    

//     std::cout << "distance_to_leader: " << distance_to_leader << "m" << "  " << "speed : "<< target_velocity * 3.6 << "km/h" << std::endl;
    
//     return target_velocity;
// }

// Formation 변경 시 새로운 FID 계산
void TruckController::update_formation_id() {
    // Formation 변경 시 FID 순환: 0->2, 1->0, 2->1
    formation_id_ = (formation_id_ + 2) % 3;
    //RCLCPP_INFO(this->get_logger(), 
    //    "Formation changed (count: %d) - Actor %d new FID: %d",
    //    formation_change_count_, actor_id_, formation_id_);
}

// 현재 FID에 따른 선행 트럭 번호 계산
int TruckController::calculate_leader_truck_number() {
    if (formation_id_ == 0) {
        return -1;  // 선두 차량은 선행 차량 없음
    }
    
    // formation_change_count_와 actor_id_를 사용하여 선행 트럭 번호 계산
    int base_truck = (actor_id_ - formation_change_count_) % 3;
    if (base_truck < 0) base_truck += 3;
    
    // 현재 FID가 1이면 FID 0을, FID가 2면 FID 1을 추종
    int leader_fid = formation_id_ - 1;
    
    // leader_fid를 가진 트럭의 번호 계산
    int leader_truck = (base_truck - 1 + 3) % 3;
    
    return leader_truck;
}

// Formation 변경 콜백
void TruckController::formation_change_callback(const std_msgs::msg::Empty::SharedPtr msg) {
    formation_change_count_++;
    update_formation_id();
}

// // 군집 주행 시 속도 계산
// double TruckController::calculate_platoon_velocity() {
//     if (formation_id_ == 0) {
//         return max_speed_;
//     }

//     // 선행 트럭 번호 계산
//     int leader_truck = calculate_leader_truck_number();
//     if (leader_truck == -1 || leader_truck >= 3) {
//         RCLCPP_WARN(this->get_logger(), "Invalid leader truck number: %d", leader_truck);
//         return prev_velocity_;
//     }

//     // 선행 차량의 위치 유효성 검사
//     if (leader_truck < 0 || leader_truck >= static_cast<int>(truck_positions_.size())) {
//         RCLCPP_ERROR(this->get_logger(), "Leader truck index out of bounds: %d", leader_truck);
//         return prev_velocity_;
//     }

//     const auto& leader_pos = truck_positions_[leader_truck];
//     if (std::isnan(leader_pos.x) || std::isnan(leader_pos.y)) {
//         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//             "Leader truck %d position is invalid", leader_truck);
//         return prev_velocity_;
//     }

//     // 거리 계산 (트럭의 물리적 크기 고려)
//     double dx = leader_pos.x - cur_x_;
//     double dy = leader_pos.y - cur_y_;
//     double center_distance = std::sqrt(dx*dx + dy*dy);
    
//     // 트럭의 물리적 크기를 고려한 실제 거리 계산
//     double actual_distance = center_distance - TRUCK_LENGTH;  // 트럭 길이만큼 차감
    
//     // 실제 거리가 음수인 경우 (충돌 상태) 0으로 처리
//     if (actual_distance < 0) {
//         actual_distance = 0;
//     }
    
//     std::cout << "Center distance: " << center_distance << "m, Actual distance: " << actual_distance << "m" << std::endl;
    
//     // 거리에 따른 속도 조절 (부드러운 변화)
//     if (actual_distance < min_gap_) {
//         std::cout << "  ----------here we stop (actual distance: " << actual_distance << "m)  ";
//         return 0.0;
//     }
    
//     if (actual_distance > desired_gap_) {
//         double excess_ratio = (actual_distance - desired_gap_) / desired_gap_;
//         double speed_increase = std::min(excess_ratio, 0.5);  // 최대 50% 증가
//         std::cout << "----------here we go (actual distance: " << actual_distance << "m)  ";
//         return max_speed_ * (1.0 + speed_increase);
//     }
    
//     // min_gap과 desired_gap 사이에서 선형 보간
//     double ratio = (actual_distance - min_gap_) / (desired_gap_ - min_gap_);
//     return max_speed_ * (0.6 + (ratio * 0.4));  // 60%~100% 사이에서 선형 변화
// }

// 가속도 제한 적용
// double TruckController::apply_acceleration_limits(double target_velocity) {
//     double accel = (target_velocity - prev_velocity_) / control_dt_;
    
//     if (std::abs(accel) > max_accel_) {
//         target_velocity = prev_velocity_ + 
//             (accel > 0 ? max_accel_ : -max_accel_) * control_dt_;
//     }
    
//     return target_velocity;
// }

// 상태 기반 제어 로직
void TruckController::compute_control()
{
    if (std::isnan(cur_x_) || waypoints_.empty()) {
        //RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for valid position or waypoints.");
        return;
    }

    // === 상태 머신 ===
    if (current_state_ == ControllerState::INITIALIZING_HEADING)
    {
        // 시작 위치가 기록되었는지 확인
        if (std::isnan(start_x_)) {
            //RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for initial position record to start heading initialization.");
            // 안전 명령 발행
            std_msgs::msg::Float32 steer_msg;
            steer_msg.data = 0.0f;
            pub_steer_->publish(steer_msg);
            std_msgs::msg::Float64 vel_msg; 
            vel_msg.data = 0.0; 
            pub_vel_->publish(vel_msg);
            return;
        }

        // 시작점으로부터 실제 이동 거리 계산
        double dx_from_start = cur_x_ - start_x_;
        double dy_from_start = cur_y_ - start_y_;
        double dist_from_start = std::sqrt(dx_from_start * dx_from_start + dy_from_start * dy_from_start);

        //RCLCPP_DEBUG(this->get_logger(), "State: INITIALIZING_HEADING. Dist moved = %.3f m", dist_from_start);

        // 목표 초기 이동 거리(init_dist_) 미만이면 계속 직진
        if (dist_from_start < init_dist_)
        {
            // --- 직진 명령 발행 ---
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Driving straight for heading init (%.2f / %.2f m). Publishing vel=%.2f, steer=0.0",
                                 dist_from_start, init_dist_, 3.0);

            std_msgs::msg::Float32 steer_msg;
            steer_msg.data = 0.0f;
            pub_steer_->publish(steer_msg);
            
            // 초기 상태에서는 적절한 throttle 값으로 시작
            std_msgs::msg::Float64 vel_msg;
            vel_msg.data = 1.0;  // 초기 throttle 값 (0.3 = 30% 가속)
            pub_vel_->publish(vel_msg);

            // Odometry 발행 (생성자에서 설정된 초기 가이드 Yaw 사용)
            if (!std::isnan(current_yaw_))
            { 
                publish_odom(cur_x_, cur_y_, cur_z_, current_yaw_); 
            }

            // 다음 계산 위해 이전 위치 업데이트
            prev_x_ = cur_x_;
            prev_y_ = cur_y_;
            return; // 상태 유지하며 현재 제어 주기 종료

        } 
        else 
        {
            // --- 목표 거리 도달: 생성자에서 설정한 Yaw 값으로 RUNNING 상태 시작 ---
            //RCLCPP_INFO(this->get_logger(), "Initial distance %.2f m reached. Using constructor-initialized yaw (%.3f rad) to start RUNNING state.", init_dist_, current_yaw_);

            // 생성자에서 설정된 current_yaw_ 값이 유효한지 최종 확인
            if (std::isnan(current_yaw_)) {
                 //RCLCPP_ERROR(this->get_logger(), "Initial yaw from constructor was NaN! Setting to 0.");
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
        if (!std::isnan(prev_x_) && !std::isnan(prev_y_)) 
        { // 이전 위치 유효 확인


            double dx_move = cur_x_ - prev_x_;
            double dy_move = cur_y_ - prev_y_;
            double dist_moved_sq = dx_move * dx_move + dy_move * dy_move;

            if (dist_moved_sq > min_dist_threshold_sq)
            { // 충분히 움직였으면 업데이트
                current_yaw_ = std::atan2(dy_move, dx_move);
                heading_updated_this_cycle = true;
                //RCLCPP_DEBUG(this->get_logger(), "Yaw updated from movement: %.3f", current_yaw_);
            } 
            else 
            { // 움직임 작으면 이전 값 유지
                //RCLCPP_DEBUG(this->get_logger(), "Movement too small, keeping yaw: %.3f", current_yaw_);
                heading_updated_this_cycle = !std::isnan(current_yaw_); // 이전에 유효했으면 계속 유효
            }
        } 
        else 
        {

            // 이 상태에서는 prev_x_가 NaN이면 안 됨 (로직 오류 가능성)
            //RCLCPP_ERROR(this->get_logger(), "prev_x_ or prev_y_ is NaN in RUNNING state!");
            heading_updated_this_cycle = !std::isnan(current_yaw_); // 현재 yaw 유효성만 믿음
        }

        // 다음 계산 위해 이전 위치 업데이트 (여기서 업데이트해야 위에서 사용 가능)
        prev_x_ = cur_x_;
        prev_y_ = cur_y_;


        // 헤딩 유효성 최종 확인
        if (std::isnan(current_yaw_)) 
        {
             //RCLCPP_WARN_ONCE(this->get_logger(), "Current yaw is NaN in RUNNING state. Skipping control.");
             return;
        }

        double heading = current_yaw_;
        // --- 헤딩 추정 끝 ---

        // --- 2. Pure Pursuit 로직 (이전과 동일) ---
        int max_iterations = 100;  // 최대 반복 횟수 제한
        int iteration_count = 0;
        
        while(iteration_count < max_iterations)
        {
            // 가장 가까운 웨이포인트 찾기 (Window 사용)
            // ... (nearest_idx_pp 계산 로직 -) ...

            if(current_wp_idx_ >  (int)waypoints_.size() - 2)
            {
                current_wp_idx_ = 0; //bbang bbang

            }

            //간이 거리 탐색
            double temp_dx = waypoints_[current_wp_idx_].x - cur_x_;
            double temp_dy = waypoints_[current_wp_idx_].y - cur_y_;
            double temp_d2 = temp_dx * temp_dx + temp_dy * temp_dy;
            
            if(sqrt(temp_d2) > 200)
            {   
                current_wp_idx_ = (current_wp_idx_ + 100) % waypoints_.size();
                iteration_count++;
                continue;
            } 

            int window_pp = 10;
            int start_pp = std::max<int>(current_wp_idx_ - window_pp, 0); 
            int end_pp = std::min<int>(current_wp_idx_ + window_pp, (int)waypoints_.size() - 1);
            int nearest_idx_pp = current_wp_idx_;

            if (start_pp <= end_pp && start_pp < (int)waypoints_.size() && end_pp >= 0) 
            { 
                double min_d2_pp = std::numeric_limits<double>::infinity(); 
                for (int i = start_pp; i <= end_pp; ++i)
                { 
                    double dx_wp = waypoints_[i].x - cur_x_;
                    double dy_wp = waypoints_[i].y - cur_y_;
                    double d2 = dx_wp * dx_wp + dy_wp * dy_wp;

                    if (d2 < min_d2_pp)
                    { 
                        min_d2_pp = d2; 
                        nearest_idx_pp = i;
                    }
                } 

                if(sqrt(min_d2_pp) > dist_threshold_)
                {
                    current_wp_idx_ = (current_wp_idx_ + 10) % waypoints_.size();
                    iteration_count++;
                    continue;
                }  
                else 
                {
                    current_wp_idx_ = nearest_idx_pp;
                    break;
                }
            }
            iteration_count++;
        }

        if (iteration_count >= max_iterations) {

            RCLCPP_WARN(this->get_logger(), "Failed to find suitable waypoint after %d iterations", max_iterations);
            // 현재 위치에서 가장 가까운 웨이포인트를 찾아서 설정
            double min_dist = std::numeric_limits<double>::infinity();
            int closest_idx = 0;
            
            for (size_t i = 0; i < waypoints_.size(); ++i) {
                double dx = waypoints_[i].x - cur_x_;
                double dy = waypoints_[i].y - cur_y_;
                double dist = dx * dx + dy * dy;
                
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_idx = i;
                }
            }
            current_wp_idx_ = closest_idx;
        }
        //for lane change
         if(current_wp_idx_ > 160 && current_wp_idx_<180)
         {  
            lane_number_=1;
            waypoints_=_waypoints_1;
         }

        // --- [수정됨] 목표점 찾기 (Lookahead Distance 기반 기하학적 탐색) ---
        Point2D lookahead_point = {0, 0}; // 최종 목표 지점 좌표 (x, y)
        bool found_lookahead = false;    // 목표점 찾기 성공 여부
        double lookahead_dist_sq = lookahead_dist_ * lookahead_dist_; // 목표 거리 제곱 (계산 최적화)

        // 가장 가까운 웨이포인트 인덱스(nearest_idx_pp)부터 경로를 따라 탐색 시작
        size_t search_idx = current_wp_idx_;

        for (size_t i = 0; i < waypoints_.size(); ++i) 
        { // 경로 전체를 최대 한 바퀴 탐색
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
                }
                else if (t1 >= 0.0 && t1 <= 1.0) {
                    t_intersect = t1;
                }

                if (t_intersect >= 0.0) { // 유효한 교점 찾음
                    // 교점의 실제 좌표 계산
                lookahead_point.x = p1.x + t_intersect * segment_vec.x;
                lookahead_point.y = p1.y + t_intersect * segment_vec.y;
                found_lookahead = true; // 찾기 성공 플래그 설정
                //RCLCPP_DEBUG(this->get_logger(), "Found lookahead point on segment %zu -> %zu at t=%.3f", p1_idx, p2_idx, t_intersect);
                    break; // 찾았으면 탐색 루프 종료
                }
            }
            // 다음 선분으로 이동
            search_idx = p2_idx;
        } // for 루프 끝

        if (!found_lookahead) 
        { // 모든 선분을 탐색했는데도 교점을 못 찾은 경우
            //RCLCPP_WARN(this->get_logger(), "Lookahead point not found via intersection, using fallback logic.");
            // 대체 로직 사용 (예: 가장 가까운 경로 선분 방향으로 lookahead_dist 만큼 떨어진 점)
            size_t next_idx_fb = (current_wp_idx_ + 1) % waypoints_.size();
            Point2D seg_start_fb = { waypoints_[current_wp_idx_].x, waypoints_[current_wp_idx_].y };
            Point2D seg_end_fb = { waypoints_[next_idx_fb].x, waypoints_[next_idx_fb].y };
            Point2D seg_dir_fb = { seg_end_fb.x - seg_start_fb.x, seg_end_fb.y - seg_start_fb.y };
            double seg_mag_fb = std::sqrt(distSq(seg_start_fb, seg_end_fb)); // 이전 답변의 distSq 함수 필요
            if (seg_mag_fb > 1e-6)
            {
                seg_dir_fb.x /= seg_mag_fb; 
                seg_dir_fb.y /= seg_mag_fb;
            } // 단위 벡터화
            else 
            {
                seg_dir_fb = { std::cos(heading), std::sin(heading) };
            } // 방향 모르면 현재 헤딩 방향 사용
            // 현재 위치에서 경로 방향으로 lookahead_dist 만큼 떨어진 점 (근사치)
            lookahead_point = { cur_x_ + lookahead_dist_ * seg_dir_fb.x, cur_y_ + lookahead_dist_ * seg_dir_fb.y };
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
        if (actual_lookahead_dist > 0.1) 
        { // 0 나누기 방지
            kappa = 2.0 * std::sin(alpha) / actual_lookahead_dist; // 실제 거리 사용
            // 또는 파라미터 사용: kappa = 2.0 * std::sin(alpha) / lookahead_dist_;
        }


        double raw_steer = std::atan(wheel_base_ * kappa); // 라디안

        // 조향각 스무딩
        // ... (now_steer 계산 로직 - 이전 답변 참고) ...
        double smooth_factor = 0.8;
        double now_steer = smooth_factor * prev_steer_ + (1.0 - smooth_factor) * raw_steer;
        prev_steer_ = now_steer;


        // 명령 발행 (부호 확인!)
        // ... (steer_msg, vel_msg 발행 로직 - 이전 답변 참고) ...
        std_msgs::msg::Float32 steer_msg;
        steer_msg.data = static_cast<float>(now_steer * -30); /* <<< 부호 확인! */
        // 속도 제어 로직
        // 조향각 기반 기본 속도
        //double target_velocity = calculate_target_velocity(steer_msg.data);

        double distance_to_leader = get_distance_to_leader();

        double throttle_value = calculate_platoon_velocity( formation_id_, 
                                                            distance_to_leader,
                                                            current_velocity_, 
                                                            steer_msg.data );
        // double TruckController::calculate_platoon_velocity(
        //                                             double current_fid_, 
        //                                             double distance_to_leader_,
        //                                             double current_velocity_,
        //                                             double current_steering_) 

        std::stringstream ss;
        ss<<"formation_id_ : "<<formation_id_;
        std::cout<<ss.str()<<std::endl;

        std::stringstream sss;
        sss<<"Current Waypoint Num : "<<  current_wp_idx_;
        std::cout<<sss.str()<<std::endl;

        std::stringstream ssss;
        ssss<<"distance_to_leader : "<<distance_to_leader;
        std::cout<<ssss.str()<<std::endl;


        pub_steer_->publish(steer_msg);


        // // 군집 주행 속도 계산 및 적용
        // if (formation_id_ > 0) {
        //     double platoon_velocity = calculate_platoon_velocity();
        //     // 급격한 속도 변화 방지를 위한 보간
        //     double alpha = 0.7;  // 보간 계수 (0.0~1.0)
        //     target_velocity = (alpha * platoon_velocity) + ((1.0 - alpha) * target_velocity);
        // }
        
        
        // 가속도 제한 적용
        //target_velocity = apply_acceleration_limits(target_velocity);

        
        // throttle_value 명령 발행
        std_msgs::msg::Float64 vel_msg;
        vel_msg.data = throttle_value ;
        pub_vel_->publish(vel_msg);

        // Odometry 발행 및 로깅
        publish_odom(cur_x_, cur_y_, cur_z_, heading);


        // 다음 계산을 위해 현재 속도 저장
        //prev_velocity_ = target_velocity;


        
        
    }
     // RUNNING 상태 끝

    std::stringstream ss;
    ss<<"-------------------------------------------------------------------------";
    std::cout<<ss.str()<<std::endl;

} // compute_control 끝

void TruckController::load_waypoints_0(const std::string &csv_path)
{
    std::ifstream ifs(csv_path+"0.csv");
    if (!ifs.is_open()) 
    {
        //RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file: %s0.csv", csv_path.c_str());
        rclcpp::shutdown(); // 또는 다른 오류 처리 로직
        return;
    }

    _waypoints_0.clear(); // 기존 웨이포인트 비우기
    std::string line;
    std::getline(ifs, line); // 헤더 라인 스킵

    Waypoint wp;
    int line_count = 1;
    while (std::getline(ifs, line))
    {
        line_count++;
        std::istringstream ss(line);
        char comma; // 콤마 무시용

        // CSV 형식이 "x,y,z" 라고 가정 (쉼표로 구분)
        if (ss >> wp.x >> comma >> wp.y >> comma >> wp.z ) {
            _waypoints_0.push_back(wp);
        } 
        else 
        {
            //RCLCPP_WARN(this->get_logger(), "Failed to parse waypoint line %d: %s", line_count, line.c_str());
        }
    }
}

void TruckController::load_waypoints_1(const std::string &csv_path)
{
    std::ifstream ifs(csv_path+"1.csv");
    if (!ifs.is_open()) 
    {
        //RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file: %s", csv_path.c_str());
        rclcpp::shutdown(); // 또는 다른 오류 처리 로직
        return;
    }

    _waypoints_1.clear(); // 기존 웨이포인트 비우기
    std::string line;
    std::getline(ifs, line); // 헤더 라인 스킵

    Waypoint wp;
    int line_count = 1;
    while (std::getline(ifs, line))
    {
        line_count++;
        std::istringstream ss(line);
        char comma; // 콤마 무시용

        // CSV 형식이 "x,y,z" 라고 가정 (쉼표로 구분)
        if (ss >> wp.x >> comma >> wp.y >> comma >> wp.z )
        {
            _waypoints_1.push_back(wp);
        } 
        else 
        {
            //RCLCPP_WARN(this->get_logger(), "Failed to parse waypoint line %d: %s", line_count, line.c_str());
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

// 각 트럭의 위치 업데이트 콜백
void TruckController::truck0_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    if (!msg || std::isnan(msg->x) || std::isnan(msg->y) || std::isnan(msg->z)) {
        //RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
         //   "Received invalid position data for truck0");
        return;
    }
    
    // 급격한 위치 변화 감지 (예: 100m 이상의 순간 이동)
    if (truck_positions_[0].x != 0.0) {  // 첫 데이터가 아닌 경우
        double dx = msg->x - truck_positions_[0].x;
        double dy = msg->y - truck_positions_[0].y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 100.0) {
           // RCLCPP_WARN(this->get_logger(), 
               // "Detected suspicious position jump for truck0: %.2f meters", dist);
            return;
        }
    }
    
    truck_positions_[0].x = msg->x;
    truck_positions_[0].y = msg->y;
    truck_positions_[0].z = msg->z;
}

void TruckController::truck1_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    if (!msg || std::isnan(msg->x) || std::isnan(msg->y) || std::isnan(msg->z)) {
       // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
          //  "Received invalid position data for truck1");
        return;
    }
    
    // 급격한 위치 변화 감지
    if (truck_positions_[1].x != 0.0) {
        double dx = msg->x - truck_positions_[1].x;
        double dy = msg->y - truck_positions_[1].y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 100.0) {
          //  RCLCPP_WARN(this->get_logger(), 
               // "Detected suspicious position jump for truck1: %.2f meters", dist);
            return;
        }
    }
    
    truck_positions_[1].x = msg->x;
    truck_positions_[1].y = msg->y;
    truck_positions_[1].z = msg->z;
}

void TruckController::truck2_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    if (!msg || std::isnan(msg->x) || std::isnan(msg->y) || std::isnan(msg->z)) {
      //  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
           // "Received invalid position data for truck2");
        return;
    }
    
    // 급격한 위치 변화 감지
    if (truck_positions_[2].x != 0.0) {
        double dx = msg->x - truck_positions_[2].x;
        double dy = msg->y - truck_positions_[2].y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 100.0) {
          //  RCLCPP_WARN(this->get_logger(), 
              //  "Detected suspicious position jump for truck2: %.2f meters", dist);
            return;
        }
    }
    
    truck_positions_[2].x = msg->x;
    truck_positions_[2].y = msg->y;
    truck_positions_[2].z = msg->z;
}

// 현재 포메이션 기준 선행 차량과의 거리 계산
double TruckController::get_distance_to_leader() {
    if (formation_id_ <= 0) {
        return std::numeric_limits<double>::infinity();  // 선두 차량은 무한대 거리 반환
    }

    // 선행 차량의 actor_id 찾기 (formation_id가 1 작은 차량)
    int leader_formation_id = calculate_leader_truck_number();

    if (!std::isnan(truck_positions_[leader_formation_id].x) && !std::isnan(truck_positions_[leader_formation_id].y))
    {
        const auto& leader_pos = truck_positions_[leader_formation_id];
        
        double dx = truck_positions_[leader_formation_id].x - cur_x_;
        double dy = truck_positions_[leader_formation_id].y - cur_y_;

        double distance_from_sensor = std::sqrt(dx*dx + dy*dy);
        double distance_to_leader = distance_from_sensor - TRUCK_LENGTH;

        return distance_to_leader;

    }

    
    // 선행 차량을 찾지 못하거나 위치가 유효하지 않은 경우
    return std::numeric_limits<double>::infinity();
}

void TruckController::current_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
{   
    current_velocity_ = msg->data * 3.6;  // m/s를 km/h로 변환
}

// double TruckController::calculate_acceleration_command(double target_velocity, double current_velocity)
// {
//     const double dt = 0.02;  // 제어 주기 (50Hz 가정)
    
//     double velocity_error = target_velocity - current_velocity;
    
//     integral_velocity_error_ += velocity_error * dt;
//     const double integral_limit = 1.0;
//     integral_velocity_error_ = std::clamp(integral_velocity_error_, -integral_limit, integral_limit);
    
//     double derivative_error = (velocity_error - prev_velocity_error_) / dt;
//     prev_velocity_error_ = velocity_error;
    
//     double control_output = 
//         kp_velocity_ * velocity_error +
//         ki_velocity_ * integral_velocity_error_ +
//         kd_velocity_ * derivative_error;
    
//     // 제어 출력을 throttle/brake 명령으로 변환 (-1.0: full brake, 1.0: full throttle)
//     return std::clamp(control_output, -1.0, 1.0);
// }

// 1. FID 업데이트 함수
// void TruckController::update_current_fid() {
//     current_fid_ = calculate_formation_id();  // 기존 함수 활용
// }

// 2. FID와 거리에 따른 기준 속도 계산 (km/h)
double TruckController::get_reference_velocity(int fid, double distance_to_leader) {
    if (fid == 0) {
        std::stringstream ss;
        ss<<"leader speed";
        std::cout<<ss.str()<<std::endl;
        return STABLE_SPEED;  // 선두차량은 70km/h
    } 
    // FID 1,2인 경우 (후미 차량)
    
    if(distance_to_leader < emergency_gap_) 
    {   
        std::stringstream ss;
        ss<<"emergency stop";
        std::cout<<ss.str()<<std::endl;
        return -100.0;
    }


   
    if (distance_to_leader < min_gap_) 
    {

        std::stringstream ss;
        ss<<"slow down";
        std::cout<<ss.str()<<std::endl;

        return SLOW_SPEED;  // 최소 간격보다 가까우면 30km/h
    } 
    else if (distance_to_leader < desired_gap_)
    {
        std::stringstream ss;
        ss<<"stable speed";
        std::cout<<ss.str()<<std::endl;
        return STABLE_SPEED;  // 희망 간격 이내면 70km/h
    } 
    else 
    {   
        std::stringstream ss;
        ss<<"accelerate";
        std::cout<<ss.str()<<std::endl;
        return ACC_SPEED;  // 희망 간격보다 멀면 100km/h
    }
}

// 3. 조향각에 따른 속도 감소
double TruckController::adjust_velocity_for_steering(double base_velocity, double steering) {
    double abs_steering = std::abs(steering);
    if(base_velocity < -10.0) return -100.0;
    
    if (abs_steering >= 0.06) {
        return base_velocity / 2.0;  // 조향각 0.06 이상
    } else if (abs_steering >= 0.04) {
        return base_velocity / 1.0;  // 조향각 0.04 이상
    }
    
    return base_velocity;  // 조향각 작은 경우 원래 속도
}

// 4. PID 제어
double TruckController::calculate_pid_output(double current_vel, double target_vel,
                                           double kp, double ki, double kd) {

    if(target_vel < -10.0) return -100.0;

    double error = target_vel - current_vel;
    integral_ += error * dt_;
    
    // 적분항 제한
    const double integral_limit = 10.0;  // 적분항 제한
    integral_ = std::clamp(integral_, -integral_limit, integral_limit);
    
    double derivative = (error - prev_error_) / dt_;
    prev_error_ = error;
    
    // PID 출력 계산
    double output = kp * error + ki * integral_ + kd * derivative;
    

    return output;
}

// 5. 제어값 정규화 (-1 ~ 1)
double TruckController::normalize_control_output(double pid_output) {
    if(pid_output < -10.0) return -100.0;
    // -1.0 ~ 1.0 사이로 클리핑
    
    
    double clamped_output = std::clamp(pid_output, -1.0, 1.0);

    // std::stringstream ss;
    // ss << "pid_output: " << pid_output << " clamped_output: " << clamped_output ;
    // std::cout<<ss.str()<<std::endl;


    return clamped_output;
}

// 메인 제어 함수 (기존 함수 수정)
double TruckController::calculate_platoon_velocity(
                                                    double current_fid_, 
                                                    double distance_to_leader_,
                                                    double current_velocity_,
                                                    double current_steering_) 
    {
    // 1. FID 업데이트
    //update_current_fid();
    
    // 2. 기준 속도 계산
    double ref_velocity = get_reference_velocity(current_fid_, distance_to_leader_);
    
    // 3. 조향각 기반 속도 조정
    double adjusted_velocity = adjust_velocity_for_steering(ref_velocity, current_steering_);
    
    // 4. PID 제어 (현재 속도는 m/s를 km/h로 변환)
    double pid_output = calculate_pid_output(
        current_velocity_,
        adjusted_velocity,
        0.3,    // kp
        0.1,    // ki
        0.2     // kd
    );
    
    // 4. PID 출력을 throttle 값으로 변환
    double throttle = normalize_control_output(pid_output);
    
    // 5. 로깅
    std::stringstream ss;
    ss<< "Current velocity (km/h): " << current_velocity_ 
              << ", Target velocity (km/h): " << adjusted_velocity << std::endl<<"throttle: "<<throttle<<std::endl<<"lane_number: "<<lane_number_;
    std::cout<<ss.str()<<std::endl;

    
    return throttle;
}

