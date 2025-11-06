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

    int id      = std::stoi(truck_id_.substr(5));
    pred_id_    = (id - 1 + max_trucks_) % max_trucks_;

    // publish status only
    status_pub_ = create_publisher<TruckStatus>("/" + truck_id_ + "/status", 10);
    aero_pub_   = create_publisher<std_msgs::msg::Float32>("/" + truck_id_ + "/drag", 10);
    sync_soc_pub_   = create_publisher<std_msgs::msg::Int32>("/sync_soc", 10);

    // subscribe velocity
    vel_sub_ = create_subscription<std_msgs::msg::Float32>("/" + truck_id_ + "/velocity", 10, [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                current_speed_ = msg->data;

                {
                    std::lock_guard<std::mutex> lock(ready_mtx_);
                    ready_.ego_vel = ready_.frame_id;
                }
                if (all_inputs_ready()) { compute_SOC(); }
            });

    //subscribe velocity control
    reference_vel_sub_ = create_subscription<std_msgs::msg::Float64>("/" + truck_id_ + "/reference_velocity", 10, [this](const std_msgs::msg::Float64::SharedPtr msg)
            {
                reference_velocity_ = msg->data;
                {
                    std::lock_guard<std::mutex> lock(ready_mtx_);
                    ready_.refer_vel = ready_.frame_id;
                }
                if (all_inputs_ready()) { compute_SOC(); }
            });

    pos_sub_ = create_subscription<nav_msgs::msg::Odometry>("/truck" + std::to_string(id) + "/ENU", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) 
            {   
                ego_pos = msg->pose.pose.position;
                {
                    std::lock_guard<std::mutex> lock(ready_mtx_);
                    ready_.ego_enu = ready_.frame_id;
                }
                if (all_inputs_ready()) { compute_SOC(); } 
            });

    target_pos_sub_ = create_subscription<nav_msgs::msg::Odometry>("/truck" + std::to_string(pred_id_) + "/ENU", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) 
            {   
                target_pos = msg->pose.pose.position;
                {
                    std::lock_guard<std::mutex> lock(ready_mtx_);
                    ready_.target_enu = ready_.frame_id;
                }
                if (all_inputs_ready()) { compute_SOC(); } 
            });

    // subscribe frame tick
    frame_sub_ = create_subscription<std_msgs::msg::UInt32>("/sim/frame_id", 10, std::bind(&SOCEstimatorNode::on_frame_tick, this, std::placeholders::_1));   
    
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

        formation_subs_.push_back(sub);
    }

    // Simulink 모델 초기화
    BMSObj.initialize();

    RCLCPP_INFO(get_logger(), "Initialized soc_estimator for %s (mode=%s)", truck_id_.c_str(), mode_.c_str());

    std::string path = std::string("/home/avees/ros2_ws/src/test_soc/soc_estimator_") + truck_id_ + ".csv";
    csv_file_.open(path);
    csv_file_ << "truck_id,frame,simulation_time,mode,SOC(%),distance(m),velocity(km/h)\n";

}

void SOCEstimatorNode::on_frame_tick(const std_msgs::msg::UInt32::SharedPtr msg)
{
    const uint32_t frame_id = msg->data;
    std::lock_guard<std::mutex> lock(ready_mtx_);

    // 1) 이번 프레임으로 업데이트
    uint32_t prev_frame = ready_.frame_id;
    ready_.frame_id = frame_id;
    ready_.computed = false;
}

bool SOCEstimatorNode::all_inputs_ready() const
{
    std::lock_guard<std::mutex> lock(ready_mtx_);
    const uint32_t f = ready_.frame_id;

    // 아직 한 번도 frame을 못 받았으면 아무것도 하지 말자
    if (f == 0) return false;

    bool ok =
        ready_.ego_vel == f &&
        ready_.refer_vel == f &&
        ready_.ego_enu == f &&
        ready_.target_enu == f &&
        !ready_.computed;

    return ok;
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
        soc_at_change_start_   = output_soc_;

        RCLCPP_INFO(get_logger(), "Mode changed → %s | Start time = %.2fs, SOC = %.2f%%", mode_.c_str(), time_at_change_start_.seconds(), soc_at_change_start_);
    }
}

void SOCEstimatorNode::formation_end_callback(const std_msgs::msg::Int32::SharedPtr msg, int id)
{
    if (last_flag[id] != 0 && msg->data == 0)
    {
        mode_changed_ = false;

        time_at_change_end_ = this->now();
        pos_at_change_end_ = ego_pos;
        soc_at_change_end_ = output_soc_;
        double duration = (time_at_change_end_ - time_at_change_start_).seconds();
        double distance = compute_distance(pos_at_change_end_, pos_at_change_start_);
        double soc_drop = soc_at_change_start_ - soc_at_change_end_;

        RCLCPP_INFO(this->get_logger(), "[Formation Change Complete] Duration = %.2fs | Distance = %.2fm | SOC Consumed = %.2f%%", duration, distance, soc_drop);

        baseline_target_distance_ = distance;

        start_baseline_measurement();  // ⬅️ 아래 함수 호출
    }

    last_flag[id] = msg->data;
}

void SOCEstimatorNode::start_baseline_measurement()
{
    measuring_baseline_ = true;
    pos_at_baseline_start_ = ego_pos;
    soc_at_baseline_start_ = output_soc_;
    time_at_baseline_start_ = this->now();

    RCLCPP_WARN(this->get_logger(),"[Baseline] Measurement started | Target distance = %.2fm | Start SOC = %.2f%%", baseline_target_distance_, soc_at_baseline_start_);
}

double SOCEstimatorNode::compute_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{

    return std::sqrt(
        std::pow(b.x - a.x, 2) +
        std::pow(b.y - a.y, 2) +
        std::pow(b.z - a.z, 2));
}

void SOCEstimatorNode::compute_SOC()
{
    {
        std::lock_guard<std::mutex> lock(ready_mtx_);
        ready_.computed = true;  
    }
    //RCLCPP_INFO(this->get_logger(), "Start computing SOC estimation for frame %d", ready_.frame_id);

    if (measuring_baseline_)
    {
        double dist = compute_distance(ego_pos, pos_at_baseline_start_);

        if (dist >= baseline_target_distance_)
        {
            rclcpp::Time end_time = this->now();
            double duration = (end_time - time_at_baseline_start_).seconds();
            double soc_drop = soc_at_baseline_start_ - output_soc_;

            RCLCPP_INFO(this->get_logger(), "✅ [Baseline Complete] Distance = %.2fm | Duration = %.2fs | SOC Δ = %.2f%%", dist, duration, soc_drop);

            measuring_baseline_ = false;
        }
    }

    rclcpp::Time now_sim_time = this->now();  // 현재 시뮬레이션 시간

    int num_steps = 4;
	
    for (int i = 0; i < num_steps; ++i) 
    {
        //RCLCPP_INFO(this->get_logger(), "Step %d", i);
        // 1) BMS 입력 공통 할당
        BMSObj.rtU.target_velocity = current_speed_ * 3.6; //kph
        BMSObj.rtU.ego_velocity = output_speed_; // mps
        BMSObj.rtU.Mass_kg = 40000.0;
        if(mode_changed_ && mode_ == "FV2")
        { 
            BMSObj.rtU.Mode = 2.0;
        }
        else{
            BMSObj.rtU.Mode = (mode_ == "LV") ? 0.0 : 1.0;
        }
        
        if(mode_ != "LV") {
            BMSObj.rtU.IVD = compute_distance(ego_pos, target_pos);
        }
        else {
            BMSObj.rtU.IVD = 30.0;
        }
        
        // 2) 모델 한 스텝 실행
        BMSObj.step();

        // 3) 출력 읽기 및 상태 업데이트
        output_distance_ = BMSObj.rtY.Positionm;
        output_speed_    = BMSObj.rtY.Speedms;
        output_soc_= BMSObj.rtY.SOC;
        output_aerodrag_ = BMSObj.rtY.Aero;
        
        if (has_prev_pos_) 
        {
            double delta_dist = compute_distance(ego_pos, last_pos_);
            total_distance_m_ += delta_dist;
        }
        last_pos_ = ego_pos;
        has_prev_pos_ = true;

        // 4) 상태 메시지 발행
        TruckStatus status_msg;
        status_msg.truck_id = std::stoi(truck_id_.substr(5));
        status_msg.soc      = output_soc_;
        status_msg.speed    = current_speed_;
        status_msg.position = total_distance_m_;
        status_msg.mode     = mode_;
        status_pub_->publish(status_msg);

        std_msgs::msg::Float32 aero_drag_;

        if ( i == 3 ) 
        {
            if(mode_changed_) {  
                aero_drag_.data = 0.6441;
            } else { 
                aero_drag_.data = output_aerodrag_; 
            }

            aero_pub_->publish(aero_drag_);
            std_msgs::msg::Int32 index;
            index.data = std::stoi(truck_id_.substr(5));
            sync_soc_pub_->publish(index);

            rclcpp::Time wall_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();  // 실시간 기준 시간
            csv_file_ << truck_id_ << ","
                << ready_.frame_id << ","
                << ready_.frame_id * 0.04 << ","
                << mode_ << ","
                << output_soc_ << ","
                << total_distance_m_ << ","
                << current_speed_ * 3.6 << "\n";

            csv_file_.flush();
            
            //RCLCPP_INFO(get_logger(),"[Frame: %.d][%s:%s] SOC: %.2f%%  POS: %.2fm  VEL: %.2fkm/h  TARGET_VEL: %.2fkm/h  Aero: %.2f ",ready_.frame_id,truck_id_.c_str(), mode_.c_str(),output_soc_, total_distance_m_, output_speed_ * 3.6, current_speed_ * 3.6, aero_drag_.data);
        }
        
        // rclcpp::Time wall_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();  // 실시간 기준 시간
        // csv_file_ << truck_id_ << ","
        //     << ready_.frame_id << ","
        //     << ready_.frame_id * 0.04 << ","
        //     << mode_ << ","
        //     << output_soc_ << ","
        //     << total_distance_m_ << ","
        //     << current_speed_ * 3.6 << "\n";

        // csv_file_.flush();
        
        // RCLCPP_INFO(get_logger(),"[Frame: %.d][%s:%s] SOC: %.2f%%  POS: %.2fm  VEL: %.2fkm/h Aero: %.2f ",ready_.frame_id,truck_id_.c_str(), mode_.c_str(),output_soc_, total_distance_m_, current_speed_ * 3.6, aero_drag_.data);
    }

}

