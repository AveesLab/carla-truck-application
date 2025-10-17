#include "platoon_coordinator/platoon_coordinator.hpp"
using ros2_msg::msg::TruckStatus;
using ros2_msg::msg::TruckCommand;


PlatoonCoordinator::PlatoonCoordinator()
: Node("platoon_coordinator")
{
  RCLCPP_INFO(this->get_logger(), "PlatoonCoordinator started");
  
  for (int id = 0; id < max_trucks_; ++id) {previous_soc_[id] = 0.0;}

  // 트럭 상태 수집용 Subscriber 설정
  for (int id = 0; id < max_trucks_; ++id) 
  {
    auto topic = "/truck" + std::to_string(id) + "/status";
    auto sub = this->create_subscription<TruckStatus>(topic, 10, [this](const TruckStatus::SharedPtr msg) { this->status_callback(msg); });
    truck_subscribers_.push_back(sub);
  }

  // 각 트럭에 명령 전송용 Publisher 설정
  for (int id = 0; id < max_trucks_; ++id) 
  {
    auto topic = "/truck" + std::to_string(id) + "/command";
    auto pub = this->create_publisher<TruckCommand>(topic, 10);
    command_publishers_.push_back(pub);
  }

  // 주기적 최적화 루프
  optimization_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&PlatoonCoordinator::optimization_timer_callback, this));
}

void PlatoonCoordinator::status_callback(const TruckStatus::SharedPtr msg) 
{
  TruckState &st = truck_states_[msg->truck_id];
  st.soc      = msg->soc;
  st.speed    = msg->speed;
  st.position = msg->position;
  st.mode     = msg->mode;
  std::cout << "[Truck" << msg->truck_id << ", Mode:" << st.mode << "] SOC:" << st.soc << ", Speed:" << st.speed << ", Position:" << st.position <<  std::endl;
}


std::string PlatoonCoordinator::get_next_mode(const std::string &current_mode) 
{
  // if (current_mode == "LV") return "FV1";
  // if (current_mode == "FV1") return "FV2";
  // if (current_mode == "FV2") return "LV";

  if (current_mode == "LV") return "FV2";
  if (current_mode == "FV1") return "LV";
  if (current_mode == "FV2") return "FV1";

  return current_mode; 
}

void PlatoonCoordinator::optimization_timer_callback() 
{
  command_buffer_.clear();

  bool need_rotate = false;
  for (const auto &kv : truck_states_) 
  {
    int id = kv.first;
    const TruckState &st = kv.second;
    double prev = previous_soc_[id];
    if (prev > 0.0 && (prev - st.soc) >= 3.0) 
    {
      need_rotate = true;
      break;
    }
  }

  if (need_rotate) 
  {
    for (auto &kv : truck_states_) 
    {
      int id = kv.first;
      const TruckState &st = kv.second;

      std::string next = get_next_mode(st.mode);
      TruckCommandData cmd;
      cmd.mode = next;
      cmd.change_flag = true;
      command_buffer_[id] = cmd;
      previous_soc_[id] = st.soc;

      RCLCPP_WARN(this->get_logger(), "🔁 Platoon rotate: Truck %d mode %s -> %s (flag: %d) ", id, st.mode.c_str(), next.c_str(), cmd.change_flag);
    }
  }
  else 
  {
    for (const auto &kv : truck_states_) 
    {
      int id = kv.first;
      TruckCommandData cmd;
      const TruckState &st = kv.second;
      if (previous_soc_[id] == 0.0) 
      {
        previous_soc_[id] = st.soc;
      }
    }
  }

  publish_commands();
}

void PlatoonCoordinator::publish_commands() 
{
  for (const auto &cmd : command_buffer_) 
  {
    int id = cmd.first;
    const TruckCommandData &data = cmd.second;

    TruckCommand msg;
    msg.truck_id        = id;
    msg.mode 			= data.mode;
    msg.lane_change_flag = data.change_flag;

    command_publishers_[id]->publish(msg);
  }
}


