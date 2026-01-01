#include "truck_controller.hpp"
#include <fstream>
#include <sstream>
#include <cmath>      // atan2, sqrt, isnan, M_PI 등
#include <limits>     // quiet_NaN, infinity
#include <algorithm>  // clamp, max, min
#include <stdexcept>
#include <string>
#include <array>

#include <unistd.h>   // getpid()를 위해 추가

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
    dt_(0.01),               // 제어 주기 Hz
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
    this->declare_parameter<double>("curve_lookahead_dist"); 
    this->declare_parameter<double>("straight_lookahead_dist"); 

    this->declare_parameter<double>("ACC_SPEED");     
    this->declare_parameter<double>("SLOW_SPEED");     
    this->declare_parameter<double>("STABLE_SPEED");     

    this->declare_parameter<double>("WHEEL_BASE");    
    this->declare_parameter<double>("MIN_GAP");
    this->declare_parameter<double>("DESIRED_GAP");
    this->declare_parameter<double>("EMERGENCY_GAP");

    // 군집 주행 관련 파라미터 선언
    std::string csv_path;

    this->get_parameter("csv_path", csv_path);
    this->get_parameter("curve_lookahead_dist", curve_lookahead_dist_);
    this->get_parameter("straight_lookahead_dist", straight_lookahead_dist_);

    this->get_parameter("ACC_SPEED", ACC_SPEED_);
    this->get_parameter("SLOW_SPEED", SLOW_SPEED_);
    this->get_parameter("STABLE_SPEED", STABLE_SPEED_);

    this->get_parameter("WHEEL_BASE", WHEEL_BASE_);
    this->get_parameter("MIN_GAP", MIN_GAP_);
    this->get_parameter("DESIRED_GAP", DESIRED_GAP_);
    this->get_parameter("EMERGENCY_GAP", EMERGENCY_GAP_);

    min_gap_ = MIN_GAP_;
    desired_gap_ = DESIRED_GAP_;
    emergency_gap_ = EMERGENCY_GAP_;

    acc_speed_ = ACC_SPEED_;
    slow_speed_ = SLOW_SPEED_;
    stable_speed_ = STABLE_SPEED_;

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
            } else { /* zero length segment */ }
        } else { /* index out of bounds */ }
    } else { /* not enough waypoints */ }

    // 초기 Yaw 설정 로직은 compute_control 첫 호출 시 수행됨
    std::string ns = "/truck" + std::to_string(actor_id_);

    // ROS 통신 설정 (namespace 덕분에 /truck0/gnss 등으로 구독합니다)
    pub_vel_ = this->create_publisher<std_msgs::msg::Float64>(ns + "/velocity_control", 10);
    pub_steer_ = this->create_publisher<std_msgs::msg::Float32>(ns + "/steer_control", 10);
    pub_ENU_ = this->create_publisher<nav_msgs::msg::Odometry>(ns + "/ENU", 10 );
    pub_lane_change_end_flag_ = this->create_publisher<std_msgs::msg::Bool>("/lane_change_end_flag", 10);
    pub_reference_velocity_ = this->create_publisher<std_msgs::msg::Float64>(ns + "/reference_velocity", 10);

    // *changed
    pub_formation_change_flag_ = this->create_publisher<std_msgs::msg::Int32>(ns + "/formation_change", 10);

    // frame by Tick 구독
    sub_frame_ = this->create_subscription<std_msgs::msg::UInt32>("/sim/frame_id", 10, std::bind(&TruckController::on_frame_tick, this, std::placeholders::_1));

    // 자신의 위치 업데이트를 위한 구독 (sensor data)
    sub_server_enu_ = this->create_subscription<geometry_msgs::msg::Point>(ns + "/server/enu", 10, std::bind(&TruckController::server_enu_callback, this, std::placeholders::_1));

    sub_truck0_pos_ = this->create_subscription<geometry_msgs::msg::Point>("/truck0/server/enu", 10, std::bind(&TruckController::truck0_pos_callback, this, std::placeholders::_1));
    sub_truck1_pos_ = this->create_subscription<geometry_msgs::msg::Point>("/truck1/server/enu", 10, std::bind(&TruckController::truck1_pos_callback, this, std::placeholders::_1));
    sub_truck2_pos_ = this->create_subscription<geometry_msgs::msg::Point>("/truck2/server/enu", 10, std::bind(&TruckController::truck2_pos_callback, this, std::placeholders::_1));
    
    // 현재 속도 구독 (sensor data)
    sub_current_velocity_ = this->create_subscription<std_msgs::msg::Float32>(ns + "/velocity", 10, std::bind(&TruckController::current_velocity_callback, this, std::placeholders::_1));

    sub_truck0_velocity_ = this->create_subscription<std_msgs::msg::Float32>("/truck0/velocity", 10, std::bind(&TruckController::truck0_velocity_callback, this, std::placeholders::_1));
    sub_truck1_velocity_ = this->create_subscription<std_msgs::msg::Float32>("/truck1/velocity", 10, std::bind(&TruckController::truck1_velocity_callback, this, std::placeholders::_1));
    sub_truck2_velocity_ = this->create_subscription<std_msgs::msg::Float32>("/truck2/velocity", 10, std::bind(&TruckController::truck2_velocity_callback, this, std::placeholders::_1));

    // Formation 변경 구독
    sub_formation_command_ = this->create_subscription<ros2_msg::msg::TruckCommand>(ns + "/command", 10, std::bind(&TruckController::formation_command_callback, this, std::placeholders::_1));
    sub_formation_end_change_ = this->create_subscription<std_msgs::msg::Bool>("/lane_change_end_flag", 10, std::bind(&TruckController::formation_change_end_callback, this, std::placeholders::_1));

    // set-scenario
    sub_simple_lane_change_flag_ = this->create_subscription<std_msgs::msg::Bool>("/simple_lane_change_flag", 10, std::bind(&TruckController::simple_lane_change_flag_callback, this, std::placeholders::_1));
    sub_emergency_stop_scenario_flag_ = this->create_subscription<std_msgs::msg::Bool>("/emergency_stop_scenario_flag", 10, std::bind(&TruckController::emergency_stop_scenario_flag_callback, this, std::placeholders::_1));
    sub_cut_in_scenario_flag_ = this->create_subscription<std_msgs::msg::Bool>("/cut_in_scenario_flag", 10, std::bind(&TruckController::cut_in_scenario_flag_callback, this, std::placeholders::_1));
    sub_traffic_jam_scenario_flag_ = this->create_subscription<std_msgs::msg::Bool>("/traffic_jam_scenario_flag", 10, std::bind(&TruckController::traffic_jam_scenario_flag_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("log_dir", "/home/avees/ros2_ws/src/test_truck");
    std::string log_dir;
    this->get_parameter("log_dir", log_dir);

    if (!log_dir.empty() && log_dir.back() != '/' && log_dir.back() != '\\') {
        log_dir += "/";
    }

    std::stringstream log_name;
    log_name << log_dir << "truck" << actor_id_ << "_control_log.csv";

    log_file_.open(log_name.str(), std::ios::out | std::ios::trunc);

    if (log_file_.is_open()) {
        log_file_ << "frame_id,actor_id,formation_id,x,y,z,vel_kmh\n";
    } else {
        RCLCPP_WARN(this->get_logger(),"Failed to open log file: %s", log_name.str().c_str());
    }

} // 생성자 끝

void TruckController::on_frame_tick(const std_msgs::msg::UInt32::SharedPtr msg)
{
    const uint32_t frame_id = msg->data;
    {
        std::lock_guard<std::mutex> lock(ready_mtx_);
        ready_.frame_id = frame_id;
        ready_.computed = false;   // 이건 안 써도 되지만, 쓰면 나쁠 건 없음
    }
    compute_control();
}

bool TruckController::all_inputs_ready() const
{
    std::lock_guard<std::mutex> lock(ready_mtx_);
    const uint32_t frame = ready_.frame_id;

    if (frame == 0) {
        return false;
    }

    for (uint32_t pos : ready_.enu) {
        if (pos != frame) return false;
    }
    for (uint32_t vel : ready_.vel) {
        if (vel != frame) return false;
    }

    if (ready_.computed) {
        return false;
    }

    return true;
}

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
        //RCLCPP_INFO(this->get_logger(), "Stored starting position: (%.2f, %.2f)", start_x_, start_y_);
    }

    // 제어 로직은 별도 타이머에서 50Hz로 실행됨
}


// Formation 변경 시 새로운 FID 계산
void TruckController::update_formation_id() {
    if(actor_id_ == 0){
        std::cout<<"update_formation_id 호출"<<std::endl;
    }
    formation_change_count_++;
    // Formation 변경 시 FID 순환: 0->2, 1->0, 2->1
    formation_id_ = (formation_id_ + 2) % 3;

    LV_aid = (LV_aid + 1) % 3;
    FV1_aid = (FV1_aid + 1) % 3;
    FV2_aid = (FV2_aid + 1) % 3;
    if(formation_id_ == 0)
    {
        //herecout
        std::cout<<"LV_aid : "<<LV_aid<<std::endl;
        std::cout<<"FV1_aid : "<<FV1_aid<<std::endl;
        std::cout<<"FV2_aid : "<<FV2_aid<<std::endl;
    }
}

// 현재 FID에 따른 선행 트럭 번호 계산
int TruckController::calculate_leader_truck_number() {
    if (formation_id_ == 0) {
        return -1;  // 선두 차량은 선행 차량 없음
    }
    
    // formation_change_count_와 actor_id_를 사용하여 선행 트럭 번호 계산
    int base_truck = (actor_id_ - formation_change_count_) % 3;
    if (base_truck < 0) base_truck += 3;
    
    // leader_fid를 가진 트럭의 번호 계산
    int leader_truck = (base_truck - 1 + 3) % 3;
    
    return leader_truck;
}

// Formation 변경 콜백
void TruckController::formation_change_end_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    //true면 차선변경 끝났다는 뜻 -> check_overrrun 실행
    //false면 차선변경 중이라는 뜻
    if(lane_change_flag_ == true){
        if(formation_id_ != 0){
            formation_change_end_flag_ = msg->data;
        }
    }
}

void TruckController::formation_command_callback(const ros2_msg::msg::TruckCommand::SharedPtr msg) 
{
    if (!msg->lane_change_flag) {
        return;
    }

    if (any_scenario_flag_) {
        std::cout << "======= Blocking formation change: another scenario is active =======" << std::endl;
        return;
    }

    if (!check_stable_gaps()) {
        std::cerr << "======= Ignore formation change by unstable gap condition =======" << std::endl;
        return;
    }

    lane_change_flag_ = true;
    any_scenario_flag_ = true;   // ★ 회전 시나리오도 any_scenario의 한 종류

    std::cerr << "======= Start FORMATION_ROTATION scenario =======" << std::endl;
}


// set-scenario
void TruckController::simple_lane_change_flag_callback(const std_msgs::msg::Bool::SharedPtr msg) 
{
    if (msg->data) 
    {
        if (any_scenario_flag_) {
            std::cout << "blocking simple_lane_change_flag_: another scenario is active" << std::endl;
            return;
        }
        simple_lane_change_flag_ = true;
        any_scenario_flag_ = true;
        simple_lane_change_start_frame_ = -1;

        std::cout << "simple_lane_change_flag_ : " << simple_lane_change_flag_ << std::endl;
    } 
}

void TruckController::emergency_stop_scenario_flag_callback(const std_msgs::msg::Bool::SharedPtr msg) 
{
    if (msg->data) 
    {
        if (any_scenario_flag_) {
            std::cout << "blocking emergency_stop_scenario_flag: another scenario is active" << std::endl;
            return;
        }
        emergency_stop_scenario_flag_ = true;
        any_scenario_flag_ = true;
        std::cout << "emergency_stop_scenario_flag_ : " << emergency_stop_scenario_flag_ << std::endl;
    } else {
        if (emergency_stop_scenario_flag_) {
            emergency_stop_scenario_flag_ = false;
            any_scenario_flag_ = false;
        }
    }
}

void TruckController::cut_in_scenario_flag_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        if (any_scenario_flag_) {
            std::cout << "blocking cut_in_scenario_flag: another scenario is active" << std::endl;
            return;
        }
        cut_in_scenario_flag_ = true;
        any_scenario_flag_ = true;
        std::cout << "cut_in_scenario_flag_ : " << cut_in_scenario_flag_ << std::endl;
    } else {
        if (cut_in_scenario_flag_) {
            cut_in_scenario_flag_ = false;
            any_scenario_flag_ = false;
        }
    }
}

void TruckController::traffic_jam_scenario_flag_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        if (any_scenario_flag_) {
            std::cout << "blocking traffic_jam_scenario_flag: another scenario is active" << std::endl;
            return;
        }
        traffic_jam_scenario_flag_ = true;
        any_scenario_flag_ = true;
        std::cout << "traffic_jam_scenario_flag_ : " << traffic_jam_scenario_flag_ << std::endl;
    } else {
        if (traffic_jam_scenario_flag_) {
            traffic_jam_scenario_flag_ = false;
            any_scenario_flag_ = false;
        }
    }
}


// 상태 기반 제어 로직
void TruckController::compute_control()
{

    if (std::isnan(cur_x_) || waypoints_.empty()) {
        //RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for valid position or waypoints.");
        return;
    }

    // 지역 변수 선언 (함수 전체에서 사용)
    double distance_to_leader = 0.0;
    double throttle_value = 0.0;

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
            // {
            //     std::lock_guard<std::mutex> lock(ready_mtx_);
            //     ready_.computed = true;
            // }
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
                                 dist_from_start, init_dist_, 1.0);

            std_msgs::msg::Float32 steer_msg;
            steer_msg.data = 0.0f;
            pub_steer_->publish(steer_msg);
            
            // 초기 상태에서는 적절한 throttle 값으로 시작
            std_msgs::msg::Float64 vel_msg;
            vel_msg.data = 1.0;  // 초기 throttle 값 (0.3 = 30% 가속)
            pub_vel_->publish(vel_msg);

            // {
            //     std::lock_guard<std::mutex> lock(ready_mtx_);
            //     ready_.computed = true;
            // }

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

        
        int max_iterations = 10;  // 최대 반복 횟수 제한
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

        if (iteration_count >= max_iterations) 
        {
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
        
        check_mission_state_(current_wp_idx_);

        //for lane_change flag test
        if(deadband_flag_ == true)
        {
            // *changed
            //if(current_wp_idx_ > 3700 && current_wp_idx_<3800) lane_change_flag_=true;
            // if(current_wp_idx_ > 6200 && current_wp_idx_<6300) lane_change_flag_=true;
            //if(current_wp_idx_ > 7700 && current_wp_idx_<7800) simple_lane_change_flag_=true;
            //if(current_wp_idx_ > 9200 && current_wp_idx_<9300) lane_change_flag_=true;
            if(current_wp_idx_ > 10700 && current_wp_idx_<10800) lane_change_flag_=true;
            // if(current_wp_idx_ > 12200 && current_wp_idx_<12300) lane_change_flag_=true;
            //if(current_wp_idx_ > 13700 && current_wp_idx_<13800) lane_change_flag_=true;
            // if(current_wp_idx_ > 15200 && current_wp_idx_<15300) lane_change_flag_=true;
            // if(current_wp_idx_ > 16700 && current_wp_idx_<16800) lane_change_flag_=true;
            // if(current_wp_idx_ > 18200 && current_wp_idx_<18300) lane_change_flag_=true;
            // if(current_wp_idx_ > 19700 && current_wp_idx_<19800) lane_change_flag_=true;
            // if(current_wp_idx_ > 21200 && current_wp_idx_<21300) lane_change_flag_=true;

            // set-scenario
            mission_taken_on_EMERGENCY_STOP();
            mission_taken_on_TRAFFIC_JAM();
            mission_taken_on_CUT_IN();
            mission_taken_on_SIMPLE_LANE_CHANGE();
            
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

            if (discriminant >= 0 && a > 1e-9) 
            { // 실근이 존재하고 선분 길이가 0이 아닐 때
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

                if (t_intersect >= 0.0) 
                { // 유효한 교점 찾음
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

        if (!found_lookahead) // 모든 선분을 탐색했는데도 교점을 못 찾은 경우
        { 
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


        double raw_steer = std::atan(WHEEL_BASE_ * kappa); // 라디안

        // 조향각 스무딩
        // ... (now_steer 계산 로직 - 이전 답변 참고) ...
        double smooth_factor = 0.8;
        double now_steer = smooth_factor * prev_steer_ + (1.0 - smooth_factor) * raw_steer;
        prev_steer_ = now_steer;


        // 명령 발행 (부호 확인!)
        // ... (steer_msg, vel_msg 발행 로직 - 이전 답변 참고) ...
        std_msgs::msg::Float32 steer_msg;
        steer_msg.data = static_cast<float>(now_steer * -30); /* <<< 부호 확인! */
        //steer_msg.data = static_cast<float>(now_steer * -0.7);
        if(deadband_flag_ == true)
        {
            if(steer_msg.data < 0.05 && steer_msg.data > -0.05)  steer_msg.data = 0.00;
        }
        else
        {
            if(steer_msg.data < 0.02 && steer_msg.data > -0.02)  steer_msg.data = 0.00;
        }
  



        if(lane_change_flag_ == false)
        {
            //basic control
            //std::cout<<"basic control"<<std::endl;
            distance_to_leader = get_distance_to_leader();
            throttle_value = calculate_platoon_velocity( formation_id_, distance_to_leader, current_velocity_, steer_msg.data );

            //*changed
            if(formation_change_flag_ != 2) formation_change_flag_=0; //not changing

        }
        else
        {
            std::cout<<"lane change control"<<std::endl;
            //lane change control ~ front first
            if(formation_id_ == 0)
            {

                // changed *
                formation_change_flag_=1; //changing
                
                if(start_lane_change_flag_ == true)
                { 
                   // std::cout<<"start lane change"<<std::endl;
                    //웨이 포인트 200개 동안 차선변경 + 정착
                    lane_number_=1;
                    waypoints_=_waypoints_1;
                    start_lane_change_flag_=false;
                    start_lane_change_idx_=current_wp_idx_;
                    // lane 변경 + 감속 플래그 on
                    doing_lane_change_flag_=true; 


                }
                if((current_wp_idx_ - start_lane_change_idx_ > 200) && overrun_lane_change_flag_ == false && (doing_lane_change_flag_ == true ||decrease_speed_flag_ == true))
                {
                    
                   // std::cout<<"decrease speed : "<<start_lane_change_flag_<<std::endl;
                    // 레인 1번 진행중 중 -> 감속
                    doing_lane_change_flag_=false;
                    decrease_speed_flag_=true;
                    check_overrun();
                    
                }
                if(overrun_lane_change_flag_ == true)
                {
                    //std::cout<<"overrun"<<std::endl;
                    // 2번 차량이 넘어선 경우 -> 정상 속도 + 차선변경 *overrun 플래그는 직접계산
                    overrun_lane_change_flag_=false;
                    decrease_speed_flag_=false;
                    lane_number_=0;
                    waypoints_=_waypoints_0;
                    overrun_lane_change_idx_ = current_wp_idx_;

                    doing_lane_change_flag_=true;
                    start_lane_change_idx_ = 999999;
                }
                if((current_wp_idx_ - overrun_lane_change_idx_ > 200) && lane_number_== 0 &&doing_lane_change_flag_ == true)
                {
                    
                    std::cout<<"lane change end"<<std::endl;
                    // 차선 변경 및 정착 -> lane_change 종료 
                    doing_lane_change_flag_=false;

                    lane_change_flag_=false;
                    any_scenario_flag_ = false; 
                    start_lane_change_flag_ = true;
                    update_formation_id(); //fid 변경은 한번에 진행
                    std_msgs::msg::Bool lane_change_end_flag;
                    lane_change_end_flag.data = true;
                    pub_lane_change_end_flag_->publish(lane_change_end_flag);
                    

                    overrun_lane_change_idx_ = 999999;
                    formation_change_end_flag_=false;
                    formation_change_flag_=2; //done changing
                }



                //제어부

                if(start_lane_change_flag_ == true && lane_number_ == 1)
                {
                    //std::cout<<"doing_lane_change_flag_ 1: "<<doing_lane_change_flag_<<std::endl;
                    //웨이포인트 200개 동안 차선변경 + 정착 중에 종방향제어
                    distance_to_leader =  std::numeric_limits<double>::infinity(); //fid 0 은 상관 무
                    throttle_value =  1;
                }
                
                
                if(doing_lane_change_flag_ == true)
                {
                    //std::cout<<"doing_lane_change_flag_ 2: "<<doing_lane_change_flag_<<std::endl;
                    //웨이포인트 200개 동안 차선변경 + 정착 중에 종방향제어
                    distance_to_leader =  std::numeric_limits<double>::infinity(); //fid 0 은 상관 무
                    throttle_value =  -0.1;
                    if(lane_number_ == 0)
                    {
                        throttle_value = 1;
                    }     
                }

                if(decrease_speed_flag_ == true)
                {
                    //std::cout<<"decrease_speed_flag_ : "<<decrease_speed_flag_<<std::endl;
                    // 레인 1번 진행중 중  -> 감속  종방향제어
                    distance_to_leader = std::numeric_limits<double>::infinity(); //fid 0 은 상관 무
                    throttle_value =  calculate_platoon_velocity( formation_id_, 
                                                                            distance_to_leader,
                                                                            current_velocity_ * 1.6,
                                                                            steer_msg.data );
                }


                if(overrun_lane_change_flag_ == true)
                {
                    //std::cout<<"overrun_lane_change_flag_ : "<<overrun_lane_change_flag_<<std::endl;
                    // 2번 차량이 넘어선 경우 -> 정상 속도
                    distance_to_leader = std::numeric_limits<double>::infinity(); //fid 0 은 상관 무
                    throttle_value = 1;
                }
                if(overrun_lane_change_flag_ == false && lane_change_flag_==false)
                {
                    // 2번 차량이 넘어선 경우 -> 정상 속도 -> 정상궤도용
                    distance_to_leader = std::numeric_limits<double>::infinity(); //fid 0 은 상관 무
                    throttle_value =  calculate_platoon_velocity( formation_id_, distance_to_leader, current_velocity_, steer_msg.data );
                }
            }
            else if(formation_id_ == 1)
            {
                // 1번은 0번인 것처럼 주행
                distance_to_leader = get_distance_to_leader(); 
                throttle_value =  calculate_platoon_velocity( formation_id_ - 1, distance_to_leader, current_velocity_*1.2, steer_msg.data );

                //lane_change_flag_ 를 바꾸어 주는 코드 
                if(formation_change_end_flag_==true)
                {
                    //차선변경 끝났다는 뜻 -> check_overrrun 실행

                    // 차선 변경 및 정착 -> lane_change 종료 
                    lane_change_flag_ = false;
                    any_scenario_flag_ = false; 
                    //fid 변경 코드 
                    update_formation_id();
                    formation_change_end_flag_=false;
                    
                }
            }
            else if(formation_id_ == 2)
            {
                // 2번은 1번인 것처럼 주행
                distance_to_leader = get_distance_to_leader(); 
                throttle_value = calculate_platoon_velocity( formation_id_ , distance_to_leader*1.5, current_velocity_, steer_msg.data );

                //lane_change_flag_ 를 바꾸어 주는 코드 
                if(formation_change_end_flag_==true)
                {
                    //차선변경 끝났다는 뜻 -> check_overrrun 실행

                    // 차선 변경 및 정착 -> lane_change 종료 
                    lane_change_flag_ = false;
                    any_scenario_flag_ = false; 

                    //fid 변경 코드 
                    update_formation_id();
                    formation_change_end_flag_=false;

                }
            }
        }


        //publish control
        pub_steer_->publish(steer_msg);

        // throttle_value 명령 발행
        std_msgs::msg::Float64 vel_msg;
        vel_msg.data = throttle_value ;
        pub_vel_->publish(vel_msg);

        {
            std::lock_guard<std::mutex> lock(ready_mtx_);
            ready_.computed = true;
        }

        // Odometry 발행 및 로깅
        publish_odom(cur_x_, cur_y_, cur_z_, heading);

        //save csv
        if (log_file_.is_open()) {
            log_file_
                << ready_.frame_id << ","   // frame_id
                << actor_id_ << ","         // truck id
                << formation_id_ << ","     // LV/FV1/FV2
                << cur_x_ << ","            // x
                << cur_y_ << ","            // y
                << cur_z_ << ","            // z
                << current_velocity_        // km/h
                << "\n";
        }

        // *changed            
        if(formation_change_flag_ == 3) formation_change_flag_=0;
        //if(formation_change_flag_ == 2 && get_distance_to_leader() < desired_gap_ ) formation_change_flag_=3;
        if(formation_change_flag_ == 2 && check_stable_speeds() ) formation_change_flag_ = 3;
        //if(formation_change_flag_ == 2 && check_stable_speeds() && get_distance_to_leader() < desired_gap_ ) formation_change_flag_=3;   

        std_msgs::msg::Int32 formation_change_flag_msg;
        formation_change_flag_msg.data = formation_change_flag_;
        pub_formation_change_flag_->publish(formation_change_flag_msg);

    }
    // RUNNING 상태 끝

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
void TruckController::truck0_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg) 
{

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
    // {
    //     std::lock_guard<std::mutex> lock(ready_mtx_);
    //     ready_.enu[0] = ready_.frame_id;
    // }
    // if (all_inputs_ready()) {
    //     compute_control();
    // }

}

void TruckController::truck1_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg) 
{

    if (!msg || std::isnan(msg->x) || std::isnan(msg->y) || std::isnan(msg->z)) {
        return;
    }
    
    // 급격한 위치 변화 감지
    if (truck_positions_[1].x != 0.0) {
        double dx = msg->x - truck_positions_[1].x;
        double dy = msg->y - truck_positions_[1].y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 100.0) {
            return;
        }
    }
    
    truck_positions_[1].x = msg->x;
    truck_positions_[1].y = msg->y;
    truck_positions_[1].z = msg->z;
    // {
    //     std::lock_guard<std::mutex> lock(ready_mtx_);
    //     ready_.enu[1] = ready_.frame_id;
    // }
    // if (all_inputs_ready()) {
    //     compute_control();
    // }
}

void TruckController::truck2_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg) 
{

    if (!msg || std::isnan(msg->x) || std::isnan(msg->y) || std::isnan(msg->z)) {
        return;
    }
    
    // 급격한 위치 변화 감지
    if (truck_positions_[2].x != 0.0) {
        double dx = msg->x - truck_positions_[2].x;
        double dy = msg->y - truck_positions_[2].y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist > 100.0) {
            return;
        }
    }
    
    truck_positions_[2].x = msg->x;
    truck_positions_[2].y = msg->y;
    truck_positions_[2].z = msg->z;
    // {
    //     std::lock_guard<std::mutex> lock(ready_mtx_);
    //     ready_.enu[2] = ready_.frame_id;
    // }
    //     if (all_inputs_ready()) {
    //     compute_control();
    // }
}

// 현재 포메이션 기준 선행 차량과의 거리 계산
double TruckController::get_distance_to_leader() 
{
    if (formation_id_ == 0) {
        return std::numeric_limits<double>::infinity();  // 선두 차량은 무한대 거리 반환
    }

    // 선행 차량의 actor_id 찾기 (formation_id가 1 작은 차량)
    int leader_formation_id_actor_id = ( formation_id_ + 2 + formation_change_count_ ) % 3;
    
    // 유효하지 않은 leader_formation_id 체크
    if (leader_formation_id_actor_id < 0 || leader_formation_id_actor_id >= 3) {
        std::stringstream ss;
        ss<<"invalid leader_formation_id_actor_id: " << leader_formation_id_actor_id;
        std::cout<<ss.str()<<std::endl;
        return std::numeric_limits<double>::infinity();
    }

    if (!std::isnan(truck_positions_[leader_formation_id_actor_id].x) && !std::isnan(truck_positions_[leader_formation_id_actor_id].y))
    {
        const auto& leader_pos = truck_positions_[leader_formation_id_actor_id];
        
        double dx = truck_positions_[leader_formation_id_actor_id].x - cur_x_;
        double dy = truck_positions_[leader_formation_id_actor_id].y - cur_y_;

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

void TruckController::truck0_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
{   
    truck0_velocity_ = msg->data * 3.6;  // m/s를 km/h로 변환
    if(truck0_velocity_ > 95) truck0_overspeed_flag_ = true;

    // {
    //     std::lock_guard<std::mutex> lock(ready_mtx_);
    //     ready_.vel[0] = ready_.frame_id;
    // }

    // if (all_inputs_ready()) {
    //     compute_control();
    // }
}


void TruckController::truck1_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
{   
    truck1_velocity_ = msg->data * 3.6;  // m/s를 km/h로 변환
    if(truck1_velocity_ > 95) truck1_overspeed_flag_ = true;
    // {
    //     std::lock_guard<std::mutex> lock(ready_mtx_);
    //     ready_.vel[1] = ready_.frame_id;
    // }

    // if (all_inputs_ready()) {
    //     compute_control();
    // }
}

void TruckController::truck2_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
{   
    truck2_velocity_ = msg->data * 3.6;  // m/s를 km/h로 변환
    if(truck2_velocity_ > 95) truck2_overspeed_flag_ = true;
    // {
    //     std::lock_guard<std::mutex> lock(ready_mtx_);
    //     ready_.vel[2] = ready_.frame_id;
    // }

    // if (all_inputs_ready()) {
    //     compute_control();
    // }
}

bool TruckController::check_stable_speeds()
{
    double under_diff = 0.18;
    double over_diff = 0.18;

    if(truck0_velocity_ >stable_speed_ - under_diff  && truck0_velocity_ < stable_speed_ + over_diff &&
       truck1_velocity_ >stable_speed_ - under_diff && truck1_velocity_ < stable_speed_ + over_diff &&
       truck2_velocity_ >stable_speed_ - under_diff && truck2_velocity_ < stable_speed_ + over_diff)
    {
        std::cout<<"stable speeds"<<std::endl;
        return true;
    }
    else
    {
        //std::cout<<"unstable speeds"<<std::endl;
        return false;
    }
}

bool TruckController::check_stable_gaps()
{
    if (!std::isnan(truck_positions_[LV_aid].x) && !std::isnan(truck_positions_[LV_aid].y)
     && !std::isnan(truck_positions_[FV1_aid].x) && !std::isnan(truck_positions_[FV1_aid].y)
     && !std::isnan(truck_positions_[FV2_aid].x) && !std::isnan(truck_positions_[FV2_aid].y))
    {

        auto LV_pos = truck_positions_[LV_aid];
        auto FV1_pos = truck_positions_[FV1_aid];
        auto FV2_pos = truck_positions_[FV2_aid];
        
        double gap_01 = sqrt(distSq({LV_pos.x, LV_pos.y}, {FV1_pos.x, FV1_pos.y}))- TRUCK_LENGTH; //sensor distance - truck length
        double gap_12 = sqrt(distSq({FV1_pos.x, FV1_pos.y}, {FV2_pos.x, FV2_pos.y}))- TRUCK_LENGTH;

        if(formation_id_== 0)
        {
            std::cout<<"gap_01 : "<<gap_01<<std::endl;
            std::cout<<"gap_12 : "<<gap_12<<std::endl;
            
        } 

        if (gap_01 < DESIRED_GAP_ && gap_01 > MIN_GAP_)
        {
            if(gap_12 < DESIRED_GAP_ && gap_12 > MIN_GAP_)
            {
                return true;
            }
        }
    }

    return false;
}


bool TruckController::check_overspeed()
{
    if(truck0_overspeed_flag_ == true || truck1_overspeed_flag_ == true || truck2_overspeed_flag_ == true)
    {
        if(check_stable_speeds())
        {
            truck0_overspeed_flag_=false;
            truck1_overspeed_flag_=false;
            truck2_overspeed_flag_=false;
            return false;
        } 
        return true;
    }   
    else
    {
        return false;
    }
}



// 2. FID와 거리에 따른 기준 속도 계산 (km/h)
double TruckController::get_reference_velocity(int fid, double distance_to_leader) 
{
    std_msgs::msg::Float64 msg;

    if (fid == 0) {
        return stable_speed_;  
    } 
    
    if (std::isinf(distance_to_leader) || std::isnan(distance_to_leader)) {
        return stable_speed_;
    }
    
    if(distance_to_leader < emergency_gap_) 
    {   
        return -100.0;
    }

    if (distance_to_leader < min_gap_) 
    {
        return  slow_speed_;  
    } 
    else if (distance_to_leader < desired_gap_)  
    {
        return stable_speed_;  
    } 
    else 
    {   
        return acc_speed_;  
    }
}

// 3. 조향각에 따른 속도 감소
double TruckController::adjust_velocity_for_steering(double base_velocity, double steering) 
{
    std_msgs::msg::Float64 msg;
    double abs_steering = std::abs(steering);
    double adjusted_velocity = base_velocity;

    // 1️⃣ 비상정지 신호 유지
    if (base_velocity < -50.0) 
    {
        adjusted_velocity = -100.0;
        msg.data = adjusted_velocity;
        pub_reference_velocity_->publish(msg);

        if (formation_id_ == 0)
            std::cout << "[Leader Emergency Stop Detected @ adjust_velocity_for_steering]" << std::endl;

        return adjusted_velocity;
    }

    // 2️⃣ Deadband(직선 또는 안정 구간) → 감속 적용 안 함
    if (deadband_flag_ == true)
    {
        adjusted_velocity = base_velocity;
        msg.data = adjusted_velocity;
        pub_reference_velocity_->publish(msg);
        return adjusted_velocity;
    }

    // 3️⃣ 조향각 기반 감속
    if (abs_steering >= 0.15)
    {
        adjusted_velocity = base_velocity * 0.7;
        msg.data = adjusted_velocity;
        pub_reference_velocity_->publish(msg);
        return adjusted_velocity;
    }
    else if (abs_steering >= 0.10)
    {
        adjusted_velocity = base_velocity * 0.8;
        msg.data = adjusted_velocity;
        pub_reference_velocity_->publish(msg);
        return adjusted_velocity;
    }
    else if (abs_steering >= 0.05)
    {
        adjusted_velocity = base_velocity * 0.9;
        msg.data = adjusted_velocity;
        pub_reference_velocity_->publish(msg);
        return adjusted_velocity;
    }
    else
    {
        adjusted_velocity = base_velocity;
        msg.data = adjusted_velocity;
        pub_reference_velocity_->publish(msg);
        return adjusted_velocity;
    }
}

// 4. PID 제어
double TruckController::calculate_pid_output(double current_vel, double target_vel, double kp, double ki, double kd) 
{
    // emergency stop 신호인 경우 그대로 유지 (범위 확인으로 정밀도 문제 해결)
    if(target_vel < -50.0) {
        return -100.0;
    }

    double error = target_vel - current_vel;
    integral_ += error * dt_;
    
    // 적분항 제한
    const double integral_limit = 10.0;  // 적분항 제한
    integral_ = std::clamp(integral_, -integral_limit, integral_limit);
    
    double derivative = (error - prev_error_) / dt_;
    prev_error_ = error;
    
    // PID 출력 계산
    double output = kp * error + ki * integral_ + kd * derivative;
    if(output > 10.0) output = 10.0;
    if(output < -10.0) output = -10.0;

    return output;
}

// 5. 제어값 정규화 (-1 ~ 1)
double TruckController::normalize_control_output(double pid_output) 
{
    // emergency stop 신호인 경우 그대로 유지 (범위 확인으로 정밀도 문제 해결)
    if(pid_output < -50.0){
        return -100.0;
    }
    
    // -1.0 ~ 1.0 사이로 클리핑
    double clamped_output = std::clamp(pid_output, -1.0, 1.0);

    return clamped_output;
}

// 메인 제어 함수 (기존 함수 수정)
double TruckController::calculate_platoon_velocity(int current_fid_,  double distance_to_leader_, double current_velocity_, double current_steering_) 
{
    // 2. 기준 속도 계산
    double ref_velocity = get_reference_velocity(current_fid_, distance_to_leader_);
    
    // 3. 조향각 기반 속도 조정
    double adjusted_velocity = adjust_velocity_for_steering(ref_velocity, current_steering_);
    
    // 4. PID 제어 (현재 속도는 m/s를 km/h로 변환)
    double pid_output = calculate_pid_output(current_velocity_, adjusted_velocity,
        1.0,    // kp
        0.1,    // ki
        0.2     // kd
    );
    
    // 4. PID 출력을 throttle 값으로 변환
    double throttle = normalize_control_output(pid_output);

    return throttle;
}

void TruckController::check_mission_state_(int cur_idx_)
{
    if(cur_idx_ > Curve_idx_1.start_idx && cur_idx_ < Curve_idx_1.finish_idx-300)
    {
        lookahead_dist_ = curve_lookahead_dist_;
        deadband_flag_ = false;
    }
    else if(cur_idx_ > Curve_idx_2.start_idx+300 && cur_idx_ < Curve_idx_2.finish_idx-300)
    {
        lookahead_dist_ = curve_lookahead_dist_;
        deadband_flag_ = false;
    }
    else if(cur_idx_ > Curve_idx_3.start_idx+300 && cur_idx_ < Curve_idx_3.finish_idx-300)
    {
        lookahead_dist_ = curve_lookahead_dist_;
        deadband_flag_ = false;
    }
    else if(cur_idx_ > Curve_idx_4.start_idx+300 && cur_idx_ < Curve_idx_4.finish_idx-300)
    {   
        lookahead_dist_ = curve_lookahead_dist_;
        deadband_flag_ = false;
    }
    else
    {
        //straigt zone
        deadband_flag_ = true;
        lookahead_dist_ = straight_lookahead_dist_;
    }
}


void TruckController::check_overrun()
{

    int leader_formation_id = (1+ formation_change_count_) % 3;
    int front_first_id = (0+ formation_change_count_) % 3;

    double distance_overrun = 0;
    if (!std::isnan(truck_positions_[leader_formation_id].x) && !std::isnan(truck_positions_[leader_formation_id].y))
    {
        const auto& leader_pos = truck_positions_[leader_formation_id];
        
        double dx = truck_positions_[leader_formation_id].x - truck_positions_[front_first_id].x;
        double dy = truck_positions_[leader_formation_id].y - truck_positions_[front_first_id].y;

        double distance_from_sensor = std::sqrt(dx*dx + dy*dy);
        distance_overrun = distance_from_sensor - TRUCK_LENGTH;

        //std::cout<<"distance_overrun : "<<distance_overrun<<std::endl;
    }
    if(formation_id_ == 0)
    {
        //std::cout<<"distance_overrun : "<<distance_overrun<<std::endl;
    }
    if(distance_overrun > desired_gap_   + TRUCK_LENGTH   )
    {
        //std::cout<<"overrun computed"<<std::endl;
        overrun_lane_change_flag_=true;
    }
    else
    {
        overrun_lane_change_flag_=false;
    }

}



void TruckController::mission_taken_on_SIMPLE_LANE_CHANGE()
{
    if (!simple_lane_change_flag_) {
        return;
    }

    if (simple_lane_change_end_) {
        return;
    }

    // 1) 내 기준 simple LC 전체 시작 프레임 세팅 (처음 한 번만)
    const int32_t frame = static_cast<int32_t>(ready_.frame_id);
    if (simple_lane_change_start_frame_ < 0) {
        simple_lane_change_start_frame_ = frame;
    }

    // 2) formation_id에 따른 딜레이 프레임 결정
    int32_t my_delay_frames = 0;
    switch (formation_id_) {
        case 0: // LV
            my_delay_frames = 0;
            break;
        case 1: // FV1
            my_delay_frames = simple_lane_change_delay_frames_;      // 1단계 딜레이
            break;
        case 2: // FV2
            my_delay_frames = 2 * simple_lane_change_delay_frames_;  // 2단계 딜레이
            break;
        default:
            my_delay_frames = 0;
            break;
    }

    // 3) 아직 내 차례가 아니면 → 그냥 platoon 제어만 유지 (lane change X)
    if (frame < simple_lane_change_start_frame_ + my_delay_frames) {
        // 여기서는 lane_number_ / waypoints_ 건드리지 않음
        // 종방향은 그대로 calculate_platoon_velocity()가 처리
        return;
    }


    if (!simple_lane_change_start_) 
    {
        simple_lane_change_start_   = true;
       
        simple_lane_change_start_wp_ = current_wp_idx_;

        // lane1(두 번째 차선)용 웨이포인트로 교체
        lane_number_ = 1;
        waypoints_   = _waypoints_1;

        if (actor_id_ == 0) {
            std::cout << "[SIMPLE_LANE_CHANGE] start at wp = " << simple_lane_change_start_wp_ << std::endl;
        }
        return;
    }

    if (simple_lane_change_start_wp_ >= 0 && !waypoints_.empty()) {
        int diff = current_wp_idx_ - simple_lane_change_start_wp_;

        // wp 인덱스가 순환 구조일 경우를 대비한 보정
        if (diff < 0) {
            diff += static_cast<int>(waypoints_.size());
        }

        if (diff > simple_lane_change_duration_wp_) 
        {
            simple_lane_change_start_ = false;
            simple_lane_change_end_   = true;
            simple_lane_change_flag_  = false;
            any_scenario_flag_        = false;

            if (actor_id_ == 0) {
                std::cout << "[SIMPLE_LANE_CHANGE] Finish at wp = " << current_wp_idx_ << std::endl;
            }
        }
    }
}

void TruckController::mission_taken_on_EMERGENCY_STOP()
{   
    if (!emergency_stop_scenario_flag_) 
    {
        acc_speed_   = ACC_SPEED_;
        slow_speed_  = SLOW_SPEED_;
        stable_speed_= STABLE_SPEED_;
        return;
    }

    acc_speed_   = -100.0;
    slow_speed_  = -100.0;
    stable_speed_= -100.0;
}

void TruckController::mission_taken_on_CUT_IN()
{
    if (!cut_in_scenario_flag_) {
        // 기본 gap
        desired_gap_ = DESIRED_GAP_;
        min_gap_     = MIN_GAP_;
        return;
    }

    // CUT_IN active → formation_id == 1인 차량만 간격 벌리기
    if (formation_id_ == 1) {
        desired_gap_ = 42.0;
        min_gap_     = 41.0;
    }
}


void TruckController::mission_taken_on_TRAFFIC_JAM()
{
    if (!traffic_jam_scenario_flag_) {
        // no traffic jam → 기본 속도로
        acc_speed_   = ACC_SPEED_;
        slow_speed_  = SLOW_SPEED_;
        stable_speed_= STABLE_SPEED_;
        return;
    }

    // traffic jam active → 속도를 전체적으로 줄임
    const double scale = 0.6667;
    acc_speed_   = ACC_SPEED_  * scale;
    slow_speed_  = SLOW_SPEED_ * scale;
    stable_speed_= STABLE_SPEED_* scale;
}
