#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <fstream>
#include <chrono>

// Simulink 생성 코드 헤더파일 include
#include "LV.h"
#include "FV1.h"

class SOCEstimatorNode : public rclcpp::Node
{
public:
    SOCEstimatorNode() : Node("soc_estimator_node")
    {
        this->declare_parameter<std::string>("mode", "LV");
        this->declare_parameter<std::string>("truck_id", "truck0");
        
        
		this->get_parameter("mode", mode_);
		this->get_parameter("truck_id", truck_id_);
        

        RCLCPP_INFO(this->get_logger(), "🛠️ Initialized with truck_id: %s, Mode: %s", truck_id_.c_str(), mode_.c_str());

        // Publisher & Subscriber 설정
        if (mode_ == "LV")
        {
            lv_soc_pub_      = this->create_publisher<std_msgs::msg::Float64>("/" + truck_id_ + "/battery_soc", 10);
            lv_throttle_pub_ = this->create_publisher<std_msgs::msg::Float64>("/" + truck_id_ + "/throttle_control", 10);
            lv_brake_pub_    = this->create_publisher<std_msgs::msg::Float64>("/" + truck_id_ + "/brake_control", 10);

            curr_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                "/" + truck_id_ + "/velocity_3D", 10,
                std::bind(&SOCEstimatorNode::currentspeed_callback, this, std::placeholders::_1));

            LVObj.initialize();
        }
        else if (mode_ == "FV")
        {
            fv_soc_pub_      = this->create_publisher<std_msgs::msg::Float64>("/" + truck_id_ + "/battery_soc", 10);
            fv_throttle_pub_ = this->create_publisher<std_msgs::msg::Float64>("/" + truck_id_ + "/throttle_control", 10);
            fv_brake_pub_    = this->create_publisher<std_msgs::msg::Float64>("/" + truck_id_ + "/brake_control", 10);

            // 전방 차량 정보는 중앙 컨트롤러 혹은 launch시 static하게 바인딩 필요
            FVObj.initialize();
        }

        // 타이머 콜백
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SOCEstimatorNode::timer_callback, this));

        start_time_ = this->now();
        
        // FV 위치 관련 변수 초기화
        pos_front_ = 0.0;
        vel_front_ = 20.8;
    }

private:
    std::string truck_id_;
    std::string mode_;
    LV LVObj;
    FV1 FVObj;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lv_soc_pub_, lv_throttle_pub_, lv_brake_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fv_soc_pub_, fv_throttle_pub_, fv_brake_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curr_speed_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double lv_curr_speed_ = 0.0;
    double pos_front_ = 0.0;
    double vel_front_ = 0;
    rclcpp::Time start_time_;

    void currentspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        lv_curr_speed_ = msg->data;
    }

    void timer_callback()
    {
        double elapsed = (this->now() - start_time_).seconds();

        if (mode_ == "LV")
        {
            LVObj.rtU.Targetspeed = 75.0;
            LVObj.rtU.Currentspeed = LVObj.rtY.Speedms;  // 혹은 lv_curr_speed_
            LVObj.step();

            publish_output(LVObj.rtY.Battery_SOC, LVObj.rtY.Throttle, LVObj.rtY.Brake,
                           lv_soc_pub_, lv_throttle_pub_, lv_brake_pub_);

            RCLCPP_INFO(this->get_logger(), "[%.2fs][%s] LV_SOC: %.2f%% POS: %.2fm VEL: %.2fkm/h",
                        elapsed, truck_id_.c_str(), LVObj.rtY.Battery_SOC, LVObj.rtY.Positionm, LVObj.rtY.Speedms * 3.6);

            if (LVObj.rtY.Battery_SOC <= 20.0) 
            {
                RCLCPP_WARN(this->get_logger(), "🔋 %s (LV) SOC dropped below 20%%. Shutting down.", truck_id_.c_str());
                rclcpp::shutdown();
            }
        }
        else if (mode_ == "FV")
        {
            // 향후에는 pos_front_, vel_front_를 topic 또는 service로 업데이트
            FVObj.rtU.pos_front = pos_front_;
            FVObj.rtU.vel_front = vel_front_;
            FVObj.step();

            publish_output(FVObj.rtY.Battery_SOC, FVObj.rtY.Throttle, FVObj.rtY.Brake,
                           fv_soc_pub_, fv_throttle_pub_, fv_brake_pub_);

            RCLCPP_INFO(this->get_logger(), "[%.2fs][%s] FV_SOC: %.2f%% POS: %.2fm VEL: %.2fkm/h",
                        elapsed, truck_id_.c_str(), FVObj.rtY.Battery_SOC, FVObj.rtY.Positionm, FVObj.rtY.Speedms * 3.6);
        }
    }

    void publish_output(double soc, double throttle, double brake,
                        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr soc_pub,
                        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_pub,
                        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub)
    {
        auto soc_msg = std_msgs::msg::Float64();
        auto throttle_msg = std_msgs::msg::Float64();
        auto brake_msg = std_msgs::msg::Float64();

        soc_msg.data = soc;
        throttle_msg.data = throttle;
        brake_msg.data = brake;

        soc_pub->publish(soc_msg);
        throttle_pub->publish(throttle_msg);
        brake_pub->publish(brake_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SOCEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}

