#include "TruckOBU.hpp"

TruckOBU::TruckOBU(boost::shared_ptr<carla::client::Vehicle> vehicle_, boost::shared_ptr<carla::client::Actor> actor, int truck_num)
    : Node("truck_obu_node", rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)) {

    truck_num_ = truck_num;
    Vehicle_ = vehicle_;

    v2xpublisher_ = this->create_publisher<ros2_msg::msg::V2XCAM>("v2xcam", 1);
    v2xcustom_publisher_ = this->create_publisher<ros2_msg::msg::V2XCUSTOM>("v2xcustom", 1);

    EmergencyFlagSubscriber_ = this->create_subscription<std_msgs::msg::Bool>("emergency_flag", 10, std::bind(&TruckOBU::EmergencyFlagSubCallback, this, std::placeholders::_1));
    caution1Subscriber_ = this->create_subscription<std_msgs::msg::Bool>("caution_mode_lane1", 10, std::bind(&TruckOBU::caution1SubCallback, this, std::placeholders::_1));
    caution2Subscriber_ = this->create_subscription<std_msgs::msg::Bool>("caution_mode_lane2", 10, std::bind(&TruckOBU::caution2SubCallback, this, std::placeholders::_1));
    LaneChangeSubscriber_ = this->create_subscription<std_msgs::msg::Bool>("lane_change_flag", 10, std::bind(&TruckOBU::LaneChangeSubCallback, this, std::placeholders::_1));
    if (truck_num == 0) {
        TimeGapSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("timegap", 10, std::bind(&TruckOBU::TimeGapSubCallback, this, std::placeholders::_1));
    }

    this->declare_parameter("add_sensor/obu", false);
    bool use_obu = this->get_parameter("add_sensor/obu").as_bool();

    if (use_obu) {
        auto obu_bp = blueprint_library->Find("sensor.other.obu");
        if (!obu_bp) {
            std::cerr << "[TruckOBU]  sensor.other.obu blueprint not found! Skipping OBU creation." << std::endl;
        } else {
            auto obu_bp_modifiable = *obu_bp;
            obu_bp_modifiable.SetAttribute("path_loss_model", "geometric");
            obu_bp_modifiable.SetAttribute("gen_cam_max", "0.1");
            obu_bp_modifiable.SetAttribute("scenario", "highway");

            obu_transform = cg::Transform{cg::Location{0, 0, 2}, cg::Rotation{0, 0, 0}};
            obu_actor = world->SpawnActor(obu_bp_modifiable, obu_transform, actor.get());
            obu = boost::static_pointer_cast<cc::Sensor>(obu_actor);

            if (obu) {
                msg_from += std::to_string(truck_num - 1);
                while (truck_num != 0 && !find_preceding) {
                    auto actor_list = world->GetActors();
                    for (const auto& actor : *actor_list) {
                        if (actor->GetTypeId().front() == 'v') {
                            for (auto&& attribute : actor->GetAttributes()) {
                                if (attribute.GetValue() == msg_from) {
                                    msg_from_id = actor->GetId();
                                    std::cerr << " found actor id: " << msg_from_id << std::endl;
                                    find_preceding = true;
                                    break;
                                }
                            }
                        }
                    }
                }

                obu->Listen([this](auto data) {
                    auto CAM = boost::static_pointer_cast<csd::CAMEvent>(data);
                    for (auto& data : *CAM) {
                        auto msg = data.Message;
                        if (msg.header.stationID == msg_from_id) {
                            ros2_msg::msg::V2XCAM msg_;
                            msg_.header.stamp = this->now();
                            msg_.stationid = msg.header.stationID;
                            msg_.latitude = msg.cam.camParameters.basicContainer.referencePosition.latitude;
                            msg_.longitude = msg.cam.camParameters.basicContainer.referencePosition.longitude;
                            msg_.heading = msg.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.heading.headingValue;
                            msg_.speed = msg.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.speed.speedValue;

                            v2xpublisher_->publish(msg_);
                        }
                    }
                });
            }
        }
    }

    auto v2x_custom_bp = blueprint_library->Find("sensor.other.v2x_custom");
    if (!v2x_custom_bp) {
        std::cerr << "[TruckOBU]  sensor.other.v2x_custom blueprint not found! Skipping V2X Custom Sensor creation." << std::endl;
        return;
    }

    auto v2x_custom_bp_modifiable = *v2x_custom_bp;
    v2x_custom_bp_modifiable.SetAttribute("path_loss_model", "geometric");
    v2x_custom_bp_modifiable.SetAttribute("scenario", "highway");

    v2x_custom_transform = cg::Transform{cg::Location{0, 0, 2}, cg::Rotation{0, 0, 0}};
    v2x_custom_actor = world->SpawnActor(v2x_custom_bp_modifiable, v2x_custom_transform, actor.get());
    obu_custom = boost::static_pointer_cast<cc::Sensor>(v2x_custom_actor);
    v2x_custom = boost::static_pointer_cast<cc::ServerSideSensor>(v2x_custom_actor);

    if (v2x_custom) {
        v2x_custom->Listen([this](auto data) {
            auto image = boost::static_pointer_cast<csd::CustomV2XEvent>(data);
            for (auto& data : *image) {
                auto msg = data.Message;

                ros2_msg::msg::V2XCUSTOM msg_;
                std::istringstream iss(msg.message);
                std::string token;

                getline(iss, token, ':'); getline(iss, token, ',');
                msg_.emergency_flag = (token.find("true") != std::string::npos);

                getline(iss, token, ':'); getline(iss, token, ',');
                msg_.caution_mode_lane1 = (token.find("true") != std::string::npos);

                getline(iss, token, ':'); getline(iss, token, ',');
                msg_.lane_change_flag = (token.find("true") != std::string::npos);

                getline(iss, token, ':');
                iss >> msg_.timegap;

                v2xcustom_publisher_->publish(msg_);
            }
        });
    }
}

void TruckOBU::EmergencyFlagSubCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    emergency_flag = msg->data;
}
void TruckOBU::caution1SubCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    caution_mode_lane1 = msg->data;
}
void TruckOBU::caution2SubCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    caution_mode_lane2 = msg->data;
}
void TruckOBU::LaneChangeSubCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    lane_change_flag = msg->data;
}
void TruckOBU::TimeGapSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    timegap = msg->data;

    ros2_msg::msg::V2XCUSTOM msg_;
    msg_.header.stamp = this->now();
    msg_.emergency_flag = emergency_flag;
    msg_.caution_mode_lane1 = caution_mode_lane1;
    msg_.lane_change_flag = lane_change_flag;
    msg_.timegap = timegap;

    v2xcustom_publisher_->publish(msg_);
}

void TruckOBU::GetVel() {
    vel_ = Vehicle_->GetVelocity();
    velocity_ = vel_.x;
}
void TruckOBU::PublishV2V() {
    ros2_msg::msg::V2XCAM msg_;
    msg_.header.stamp = this->now();
    msg_.stationid = 0;
    msg_.latitude = 0;
    msg_.longitude = 0;
    msg_.heading = 0;
    msg_.speed = velocity_;
    v2xpublisher_->publish(msg_);
}
