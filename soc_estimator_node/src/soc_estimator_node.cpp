#include "soc_estimator_node.hpp"
#include "ros2_msg/msg/truck_status.hpp"
#include "ros2_msg/msg/truck_command.hpp"

using ros2_msg::msg::TruckStatus;
using ros2_msg::msg::TruckCommand;

SOCEstimatorNode::SOCEstimatorNode()
: Node("soc_estimator_node")
{
    this->declare_parameter<std::string>("truck_id", "truck0");
    this->declare_parameter<std::string>("mode",     "LV");
    this->get_parameter("truck_id", truck_id_);
    this->get_parameter("mode",     mode_);

    RCLCPP_INFO(get_logger(), "🛠️ Initialized soc_estimator for %s (mode=%s)", truck_id_.c_str(), mode_.c_str());

    // publish status only
    status_pub_ = create_publisher<TruckStatus>("/" + truck_id_ + "/status", 10);
    // subscribe velocity
    speed_sub_   = create_subscription<std_msgs::msg::Float32>("/" + truck_id_ + "/velocity", 10, std::bind(&SOCEstimatorNode::currentspeed_callback, this, std::placeholders::_1));
    // subscribe mode commands
    command_sub_ = create_subscription<TruckCommand>("/" + truck_id_ + "/command", 10, std::bind(&SOCEstimatorNode::command_callback, this, std::placeholders::_1));
    // subscribe lane_change end flag
    lane_change_end_sub_ = create_subscription<std_msgs::msg::Int32>("/" + truck_id_ + "/formation_flag", 10, std::bind(&SOCEstimatorNode::formation_end_callback, this, std::placeholders::_1));
    // subscribe predecessor status
    int id      = std::stoi(truck_id_.substr(5));
    pred_id_    = (id - 1 + max_trucks_) % max_trucks_;
    predecessor_sub_ = create_subscription<TruckStatus>("/truck" + std::to_string(pred_id_) + "/status", 10, std::bind(&SOCEstimatorNode::predecessor_status_callback, this, std::placeholders::_1));

    // Simulink 모델 초기화
    BMSObj.initialize();

    // 타이머 세팅
    start_time_ = now();
    timer_      = create_wall_timer(std::chrono::milliseconds(100), std::bind(&SOCEstimatorNode::timer_callback, this));

    // 초기값
    pos_front_      = 0.0;
    vel_front_      = 0.0;
    lv_curr_speed_  = 0.0;
    last_soc_       = 100.0;
    last_pos_       = 0.0;
    last_vel_       = 0.0;
    mode_changed_   = false;
}

void SOCEstimatorNode::currentspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    lv_curr_speed_ = msg->data;
}

void SOCEstimatorNode::formation_end_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (last_flag != 0 && msg->data == 0)
    {
            rclcpp::Time end_time = this->now();
            double duration = (end_time - change_start_time_).seconds();
            double soc_drop = soc_at_change_start_ - last_soc_;
            RCLCPP_INFO(this->get_logger(), "✅ [Formation Change Complete] Duration = %.2fs, SOC Consumed = %.2f%%", duration, soc_drop);
        
    }

    last_flag = msg->data;
}


void SOCEstimatorNode::command_callback(const TruckCommand::SharedPtr msg)
{
    int id = std::stoi(truck_id_.substr(5));
    if (msg->truck_id == id && msg->mode != mode_) 
    {
        // 모드 바뀔 때만 플래그 세우기
        mode_         = msg->mode;
        mode_changed_ = true;

        // 위치 변경 시작 시간 기록
        change_start_time_     = this->now();
        soc_at_change_start_   = last_soc_;
        is_measuring_change_   = true;

        RCLCPP_WARN(get_logger(), "📩 Mode changed → %s | ⏱ Start time = %.2fs, SOC = %.2f%%", mode_.c_str(), change_start_time_.seconds(), soc_at_change_start_);
    }
}

void SOCEstimatorNode::predecessor_status_callback(const TruckStatus::SharedPtr msg)
{
    pos_front_ = msg->position;
    vel_front_ = msg->speed;
}

void SOCEstimatorNode::timer_callback()
{
    rclcpp::Time now_sim_time = this->now();  // 현재 시뮬레이션 시간

    // 초기화 시 한 번만 last_sim_time_ 설정
    if (last_sim_time_.nanoseconds() == 0) 
    {
        last_sim_time_ = now_sim_time;
        return;
    }

    double dt = (now_sim_time - last_sim_time_).seconds();

    if (dt <= 0.0)
        return;

    int num_steps = static_cast<int>(dt / step_interval_sec_);
	double consumed_time = num_steps * step_interval_sec_;
	
    for (int i = 0; i < num_steps; ++i) 
    {
        // 1) BMS 입력 공통 할당
        BMSObj.rtU.Targetspeed = lv_curr_speed_ * 3.6; //LV 
        BMSObj.rtU.pos_ego     = last_pos_; //LV,FV
        BMSObj.rtU.vel_ego     = last_vel_; //LV,FV
        BMSObj.rtU.soc_ego     = last_soc_; 
        //BMSObj.rtU.pos_front   = last_pos_ + 30.0; //FV
        BMSObj.rtU.pos_front   = pos_front_;
        //BMSObj.rtU.vel_front   = 25 ;
        BMSObj.rtU.vel_front   = vel_front_; //FV
        BMSObj.rtU.pos_follow  = last_pos_ - 30.0;
        BMSObj.rtU.Mode        = (mode_ == "LV") ? 0.0 : 1.0;

        /*std::cout << "BMS Input: TargetSpeed=" << BMSObj.rtU.Targetspeed
                  << ", pos_ego=" << BMSObj.rtU.pos_ego
                  << ", vel_ego=" << BMSObj.rtU.vel_ego
                  << ", soc_ego=" << BMSObj.rtU.soc_ego
                  << ", pos_front=" << BMSObj.rtU.pos_front
                  << ", vel_front=" << BMSObj.rtU.vel_front
                  << ", pos_follow=" << BMSObj.rtU.pos_follow
                  << ", Mode=" << BMSObj.rtU.Mode 
                  << std::endl;*/

        // 2) 모델 한 스텝 실행
        BMSObj.step();

        // 3) 출력 읽기 및 상태 업데이트
        last_soc_ = BMSObj.rtY.Battery_SOC;
        last_pos_ = BMSObj.rtY.Positionm;
        last_vel_ = BMSObj.rtY.Speedms;

        if (mode_changed_) 
        {
            mode_changed_ = false;
        }

        // 4) 상태 메시지 발행
        TruckStatus status_msg;
        status_msg.truck_id = std::stoi(truck_id_.substr(5));
        status_msg.soc      = last_soc_;
        status_msg.speed    = last_vel_;
        status_msg.position = last_pos_;
        status_msg.mode     = mode_;
        status_pub_->publish(status_msg);

	rclcpp::Time wall_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();  // 실시간 기준 시간

	RCLCPP_INFO(get_logger(),"[Sim: %.2fs | Real: %.2fs][%s:%s] SOC: %.2f%%  POS: %.2fm  VEL: %.2fkm/h / pred:%d ",now_sim_time.seconds(),wall_time.seconds(),truck_id_.c_str(), mode_.c_str(),
			last_soc_, last_pos_, last_vel_ * 3.6, pred_id_);
    }
    last_sim_time_ += rclcpp::Duration::from_seconds(consumed_time);
}

