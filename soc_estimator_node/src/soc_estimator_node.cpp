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
    last_flag.resize(max_trucks_, 0);

    RCLCPP_INFO(get_logger(), "🛠️ Initialized soc_estimator for %s (mode=%s)", truck_id_.c_str(), mode_.c_str());

    // publish status only
    status_pub_ = create_publisher<TruckStatus>("/" + truck_id_ + "/status", 10);
    // subscribe velocity
    speed_sub_ = create_subscription<std_msgs::msg::Float32>("/" + truck_id_ + "/velocity", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                current_speed_ = msg->data;
            });
    //subscribe velocity control
    control_sub_ = create_subscription<std_msgs::msg::Float64>("/" + truck_id_ + "/velocity_control", 10, 
            [this](const std_msgs::msg::Float64::SharedPtr msg)
            {
                throttle_control_ = msg->data;
            });
    // subscribe mode commands
    command_sub_ = create_subscription<TruckCommand>("/" + truck_id_ + "/command", 10, std::bind(&SOCEstimatorNode::formation_start_callback, this, std::placeholders::_1));
    // subscribe lane_change end flag
    for (int id = 0; id < max_trucks_; ++id) 
    {
        auto topic = "/truck" + std::to_string(id) + "/formation_change";
        auto sub = create_subscription<std_msgs::msg::Int32>(topic, 10,
        [this, id](const std_msgs::msg::Int32::SharedPtr msg)
        {
            this->formation_end_callback(msg, id);
        });

        formation_subs_.push_back(sub);  // vector에 저장해두면 나중에 소멸 방지됨
    }
    // subscribe predecessor status
    int id      = std::stoi(truck_id_.substr(5));
    pred_id_    = (id - 1 + max_trucks_) % max_trucks_;
    ego_sub_ = create_subscription<nav_msgs::msg::Odometry>("/truck" + std::to_string(id) + "/ENU", 10,
                    [this](const nav_msgs::msg::Odometry::SharedPtr msg) 
                    {
                        ego_pos = msg->pose.pose.position;
                    });
    target_sub_ = create_subscription<nav_msgs::msg::Odometry>("/truck" + std::to_string(pred_id_) + "/ENU", 10, 
                    [this](const nav_msgs::msg::Odometry::SharedPtr msg) 
                    {
                        target_pos = msg->pose.pose.position;
                    });

    // Simulink 모델 초기화
    BMSObj.initialize();

    // 타이머 세팅
    start_time_ = now();
    timer_      = create_wall_timer(std::chrono::milliseconds(100), std::bind(&SOCEstimatorNode::timer_callback, this));

    // 초기값
    last_soc_       = 100.0;
    last_pos_       = 0.0;
    last_vel_       = 0.0;
    mode_changed_   = false;

    std::string path = std::string("/home/avees/ros2_ws/src/logs/soc_estimator_") + truck_id_ + ".csv";
    csv_file_.open(path);
    csv_file_ << "sim_time,real_time,truck_id,mode,SOC(%),position(m),velocity(km/h)\n";

}

void SOCEstimatorNode::formation_end_callback(const std_msgs::msg::Int32::SharedPtr msg, int id)
{

    if (last_flag[id] != 0 && msg->data == 0)
    {
        rclcpp::Time end_time = this->now();
        double duration = (end_time - time_at_change_start_).seconds();
        double distance = compute_distance(ego_pos, pos_at_change_start_);
        double soc_drop = soc_at_change_start_ - last_soc_;
        RCLCPP_INFO(this->get_logger(), "✅ [Formation Change Complete] Duration = %.2fs | Distance = %.2fm | SOC Consumed = %.2f%%", duration, distance, soc_drop);
        baseline_target_distance_ = distance;
        start_baseline_measurement();  // ⬅️ 아래 함수 호출
    }

    last_flag[id] = msg->data;
}


void SOCEstimatorNode::formation_start_callback(const TruckCommand::SharedPtr msg)
{
    int id = std::stoi(truck_id_.substr(5));
    if (msg->truck_id == id && msg->mode != mode_) 
    {
        // 모드 바뀔 때만 플래그 세우기
        mode_         = msg->mode;
        mode_changed_ = true;

        // 위치 변경 시작 시간 기록
        pos_at_change_start_ = ego_pos;
        time_at_change_start_     = this->now();
        soc_at_change_start_   = last_soc_;
        is_measuring_change_   = true;

        RCLCPP_WARN(get_logger(), "📩 Mode changed → %s | ⏱ Start time = %.2fs, SOC = %.2f%%", mode_.c_str(), time_at_change_start_.seconds(), soc_at_change_start_);
    }
}

void SOCEstimatorNode::start_baseline_measurement()
{
    measuring_baseline_ = true;
    pos_at_baseline_start_ = ego_pos;
    soc_at_baseline_start_ = last_soc_;
    time_at_baseline_start_ = this->now();

    RCLCPP_WARN(this->get_logger(),
        "🚀 [Baseline] Measurement started | Target distance = %.2fm | Start SOC = %.2f%%",
        baseline_target_distance_, soc_at_baseline_start_);
}

double SOCEstimatorNode::compute_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
    return std::sqrt(
        std::pow(b.x - a.x, 2) +
        std::pow(b.y - a.y, 2) +
        std::pow(b.z - a.z, 2));
}

void SOCEstimatorNode::timer_callback()
{

    if (measuring_baseline_)
    {
        double dist = compute_distance(ego_pos, pos_at_baseline_start_);

        if (dist >= baseline_target_distance_)
        {
            rclcpp::Time end_time = this->now();
            double duration = (end_time - time_at_baseline_start_).seconds();
            double soc_drop = soc_at_baseline_start_ - last_soc_;

            RCLCPP_INFO(this->get_logger(),
                "✅ [Baseline Complete] Distance = %.2fm | Duration = %.2fs | SOC Δ = %.2f%%",
                dist, duration, soc_drop);

            // 종료 후 상태 초기화
            measuring_baseline_ = false;
        }
    }


    rclcpp::Time now_sim_time = this->now();  // 현재 시뮬레이션 시간

    // 초기화 시 한 번만 last_sim_time_ 설정
    if (last_sim_time_.nanoseconds() == 0) 
    {
        last_sim_time_ = now_sim_time;
        return;
    }

    double dt = (now_sim_time - last_sim_time_).seconds();

    int num_steps = static_cast<int>(dt / step_interval_sec_);
	double consumed_time = num_steps * step_interval_sec_;
	
    for (int i = 0; i < num_steps; ++i) 
    {
        // 1) BMS 입력 공통 할당
        BMSObj.rtU.velocity_control = throttle_control_;
        BMSObj.rtU.velocity = current_speed_;
        if(mode_ != "LV") {
            BMSObj.rtU.IVD = compute_distance(ego_pos, target_pos);
            RCLCPP_INFO(get_logger(), "Distance to front truck: %.2f m", BMSObj.rtU.IVD);
        }
        else {
            BMSObj.rtU.IVD = 30.0;
            RCLCPP_INFO(get_logger(), "In LV mode, set IVD to 30.0 m");
        }
        BMSObj.rtU.Mode = (mode_ == "LV") ? 0.0 : 1.0;
        BMSObj.rtU.Mass_kg = 40000.0;


        // 2) 모델 한 스텝 실행
        BMSObj.step();

        // 3) 출력 읽기 및 상태 업데이트
        last_soc_ = BMSObj.rtY.SOC;
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
    csv_file_ << now_sim_time.seconds() << ","
          << wall_time.seconds() << ","
          << truck_id_ << ","
          << mode_ << ","
          << last_soc_ << ","
          << last_pos_ << ","
          << last_vel_ * 3.6 << "\n";

    csv_file_.flush();
	
    RCLCPP_INFO(get_logger(),"[Sim: %.2fs | Real: %.2fs][%s:%s] SOC: %.2f%%  POS: %.2fm  VEL: %.2fkm/h / pred:%d ",now_sim_time.seconds(),wall_time.seconds(),truck_id_.c_str(), mode_.c_str(),last_soc_, last_pos_, last_vel_ * 3.6, pred_id_);
    }
    last_sim_time_ += rclcpp::Duration::from_seconds(consumed_time);
}

