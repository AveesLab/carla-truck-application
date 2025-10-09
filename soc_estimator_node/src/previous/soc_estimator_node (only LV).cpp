#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <fstream>
#include <chrono>

// Simulink 생성 코드 헤더파일 include
#include "LV.h"

class SOCEstimatorNode : public rclcpp::Node
{
public:
    SOCEstimatorNode() : Node("soc_estimator_node")
    {
        // Publisher: SOC 값 publish
        soc_publisher_ = this->create_publisher<std_msgs::msg::Float64>("battery_soc", 10);
        // Publisher: Targetspeed 
        targetspeed_publisher_ = this->create_publisher<std_msgs::msg::Float32>("target_speed",10);
        throttle_publisher_= this->create_publisher<std_msgs::msg::Float64>("/truck0/throttle_control",10);
        brake_publisher_= this->create_publisher<std_msgs::msg::Float64>("/truck0/brake_control",10);
        
        // Subscriber: /truck0/velocity (Float32) → Currentspeed
        currentspeed_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/truck0/velocity_3D", 10, std::bind(&SOCEstimatorNode::currentspeed_callback, this, std::placeholders::_1));        
        // Subscriber: 위치 구독 (예: /truck0/pos_follow)
        pos_follow_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "pos_follow", 10, std::bind(&SOCEstimatorNode::pos_follow_callback, this, std::placeholders::_1));
        // Timer: 100ms 간격으로 step() 실행
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SOCEstimatorNode::timer_callback, this));
	
        // Simulink 모델 초기화
        rtObj.initialize();
        soc_log_file_.open("soc_log.csv", std::ios::out | std::ios::trunc);
	soc_log_file_ << "Time(s),SOC(%)\n"; 

    }
    ~SOCEstimatorNode()
	{
		if (soc_log_file_.is_open()) 
		{
		    soc_log_file_.close();
		    RCLCPP_INFO(this->get_logger(), "SOC log file closed.");
		}
	}

private:
    LV rtObj;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr soc_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr targetspeed_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr currentspeed_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pos_follow_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::ofstream soc_log_file_;

    double target_speed_ = 0.0; // 초기값
    double current_speed_ = 0;
    double current_velocity_ = 0.0;
    double pos_follow_ = 0.0;
    double throttle_ = 0.0;
    double brake_ = 0.0;
    rclcpp::Time start_time_ = this->now();

    // 속도(m/s -> km/h) 콜백 (double)
    void currentspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
		
		target_speed_ = 15.0;
		current_velocity_ = static_cast<double>(msg->data);
		
		// 현재 시간 기록
		rclcpp::Time now = this->now();
		static rclcpp::Time last_time = now;  // 첫 콜백 시 초기화
		double delta_t = (now - last_time).seconds();
		last_time = now;

		// 현재 SOC 값 확인
		double current_soc = rtObj.rtY.Battery_SOC;

	}	
	
    // 위치 콜백
    void pos_follow_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        pos_follow_ = msg->data;
    }

    // 주기적 추론 수행
	void timer_callback()
	{
		// ① 입력 업데이트
		rtObj.rtU.Targetspeed = target_speed_;   // (km/h)
		rtObj.rtU.pos_follow  = pos_follow_;
		//rtObj.rtU.Currentspeed = rtObj.rtY.Speedms;
		rtObj.rtU.Currentspeed = current_velocity_; // (m/s)
		// ② Simulink 한-스텝 실행
		rtObj.step();
		
		auto targetvel_msg = std_msgs::msg::Float32();
		auto throttle_ = std_msgs::msg::Float64();
		auto brake_ = std_msgs::msg::Float64();
		targetvel_msg.data = rtObj.rtU.Targetspeed; 
		throttle_.data = rtObj.rtY.Throttle;
		brake_.data = rtObj.rtY.Brake;
		targetspeed_publisher_->publish(targetvel_msg);
		throttle_publisher_->publish(throttle_);
		brake_publisher_->publish(brake_);

		
		// ③ 결과 Publish
		auto soc_msg   = std_msgs::msg::Float64();
		soc_msg.data   = rtObj.rtY.Battery_SOC;
		soc_publisher_->publish(soc_msg);
		current_speed_ = rtObj.rtY.Speedms;
		
		// ④ 100 ms 주기 디버그 로그  -----------------------------▼
		static rclcpp::Time last_time = this->now();
		rclcpp::Time now   = this->now();
		double delta_t     = (now - last_time).seconds();   // ≈ 0.10 s 기대
		last_time          = now;
		
		double elapsed_time = (now - start_time_).seconds();  // 누적 시간
		
		RCLCPP_INFO(this->get_logger(),
			"TargetSpeed %.2f km/h | CurrentSpeed %2f km/h | CalculatedSpeed %.2f km/h | SOC %.2f %% | t %.2f s| Throttle %.2f %% | Brake %.2f %%",
			target_speed_, current_velocity_*3.6, current_speed_*3.6, soc_msg.data, elapsed_time, throttle_, brake_);
	}
};

int main(int argc, char *argv[])
{
    std::cout << "START\n";
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SOCEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}

