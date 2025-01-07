#include "Camera.hpp"

CameraPublisher::CameraPublisher(boost::shared_ptr<carla::client::Actor> actor, int trucknum_)
    : Node("camera_node"+ std::to_string(actor->GetId()), rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)) {

    rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    custom_qos.keep_last(1);
    actor_ = actor;
    this->get_parameter_or("add_sensor/camera_number", num_cameras_, 2);
    this->get_parameter_or("carla/sync", sync_ , false);
    this->get_parameter_or("carla/sync_with_delay", sync_with_delay, false);


    //pubsub vs shared memory
    #ifdef IMAGE_FROM_ROS
    //shared memory false -> ros
        this->get_parameter_or("carla/shared_memory",shm_com,false);
    #elif IMAGE_FROM_SHARED_MEMORY
    //shared memory true -> shared memory
        this->get_parameter_or("carla/shared_memory",shm_com,true);
    #else
    //default-> ros 
        this->get_parameter_or("carla/shared_memory",shm_com,false);
    #endif
    
   
    CurImagePublisher_ = this->create_publisher<sensor_msgs::msg::Image>("cur_image1",custom_qos);
    if(sync_with_delay) {
        GetDelayParameter();
        LaneImagePublisher = this->create_publisher<std_msgs::msg::String>("lane_camera", custom_qos);
        WaitLanePublisher = this->create_publisher<std_msgs::msg::Bool>("wait_lane", custom_qos);
        WaitVelPublisher = this->create_publisher<std_msgs::msg::Bool>("wait_vel", custom_qos);
    }
    else if(sync_) {
        std::string topic_name_ = "/publish_image" + std::to_string(trucknum_);
        image_sync_sub_ = this->create_subscription<std_msgs::msg::Bool>(topic_name_, 10, std::bind(&CameraPublisher::SyncSubCallback, this, std::placeholders::_1));
    }

    velocity_image_queue.resize(num_cameras_);
    //publishers_.resize(num_cameras_);

    for(int i=0; i<num_cameras_; ++i){
        std::string index = std::to_string(i);

        this->get_parameter_or("camera" + index + "/x", rgbcam_x,2.0f);
        this->get_parameter_or("camera" + index + "/y", rgbcam_y,0.0f);
        this->get_parameter_or("camera" + index + "/z", rgbcam_z,3.5f);
        this->get_parameter_or("camera" + index + "/pitch", rgbcam_pitch, -15.0f);
        this->get_parameter_or("camera" + index + "/yaw", rgbcam_yaw,0.0f);
        this->get_parameter_or("camera" + index + "/roll", rgbcam_roll,0.0f);
        this->get_parameter_or("camera" + index + "/sensor_tick", rgbcam_sensor_tick,std::string("0.033333f"));
        this->get_parameter_or("camera" + index + "/image_size_x", rgbcam_image_size_x,std::string("1920"));
        this->get_parameter_or("camera" + index + "/image_size_y", rgbcam_image_size_y,std::string("1080"));
        this->get_parameter_or("camera" + index + "/fov", rgbcam_fov,std::string("90.0f"));
        this->get_parameter_or("camera" + index + "/topic_name", rgbcam_topic_name ,std::string("carla/image_raw" + index));
        if(sync_ || sync_with_delay) {
            rgbcam_sensor_tick = "0.0f";
        }
        //shm_com ==true ->shared memory mode
        if(shm_com) {
            std::string shared_memory_name = "/truck" + std::to_string(trucknum_)  +rgbcam_topic_name;
            void *shared_memory = initialize_shared_memory(shared_memory_name);
            if (shared_memory == nullptr) {
                std::cerr << "Failed to initialize shared memory for camera " << index << std::endl;
                continue;
            }
            shm_name.push_back(shared_memory_name);
            shared_memory_ptrs_.push_back(shared_memory);
            auto shm_publisher = this->create_publisher<std_msgs::msg::String>(rgbcam_topic_name,custom_qos);
            shm_publishers_.push_back(shm_publisher);
        }
        //shm_com==false ->ros mode
        else {
            auto publisher = this->create_publisher<sensor_msgs::msg::Image>(rgbcam_topic_name, custom_qos);
            publishers_.push_back(publisher);
        }

        auto camera_bp = blueprint_library->Find("sensor.camera.rgb");
        assert(camera_bp != nullptr);

        // Create a modifiable copy of the camera blueprint
        auto camera_bp_modifiable = *camera_bp;

        camera_bp_modifiable.SetAttribute("sensor_tick", rgbcam_sensor_tick);
        camera_bp_modifiable.SetAttribute("image_size_x", rgbcam_image_size_x);
        camera_bp_modifiable.SetAttribute("image_size_y", rgbcam_image_size_y);
        camera_bp_modifiable.SetAttribute("fov", "90.0f");
        camera_bp_modifiable.SetAttribute("lens_flare_intensity", "0.0f");
        camera_bp_modifiable.SetAttribute("lens_k", "0.0f");

        auto camera_transform = cg::Transform{cg::Location{rgbcam_x, rgbcam_y, rgbcam_z}, cg::Rotation{rgbcam_pitch, rgbcam_yaw, rgbcam_roll}};
        auto camera_actor = world->SpawnActor(camera_bp_modifiable, camera_transform, actor.get());
        auto camera = boost::static_pointer_cast<cc::Sensor>(camera_actor);

        camera_sensors.push_back(camera);
        camera_actors.push_back(camera_actor);

        camera->Listen([this, i](auto data) {
            auto image = boost::static_pointer_cast<csd::Image>(data);
            assert(image != nullptr);
            static int prev_tick_cnt = 0;
            
            if(shm_com){
                publishImage(*image, shm_publishers_[i],shared_memory_ptrs_[i],shm_name[i]);
            }
            else {
                publishImage(*image, publishers_[i]);
            }
        });


    }
}

void CameraPublisher::wait_for_lane(bool ch_) {
    std_msgs::msg::Bool msg;
    msg.data = ch_;
    WaitLanePublisher->publish(msg);
}

void CameraPublisher::wait_for_velocity(bool ch_) {
    std_msgs::msg::Bool msg;
    msg.data = ch_;
    WaitVelPublisher->publish(msg);
}

void CameraPublisher::GetDelayParameter() {
    this->get_parameter_or("period/velocity_planner", velocity_planner_period, 30);
    this->get_parameter_or("delay/velocity_planner", velocity_planner_delay , 100);
    this->get_parameter_or("period/path_planner", path_planner_period, 30);
    this->get_parameter_or("delay/path_planner", path_planner_delay , 100);

    int lcm_velocity = lcm(velocity_planner_period, velocity_planner_delay);
    int lcm_path = lcm(path_planner_period, path_planner_delay);
    int lcm_all = lcm(lcm_velocity, lcm_path);

    lcm_period = lcm_all;
}

int CameraPublisher::gcd(int a, int b) {
    while (b != 0) {
        int temp = b;
        b = a % b;
        a = temp;
    }
    return a;
}

int CameraPublisher::lcm(int a, int b) {
    return (a * b) / gcd(a, b);
}

void CameraPublisher::publishImage(const csd::Image &carla_image, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher) {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    // Set the header
     msg->header.stamp = this->now();

    msg->header.frame_id = "1"; // Set appropriate frame ID

    // Set image properties
    msg->height = carla_image.GetHeight();
    msg->width = carla_image.GetWidth();
    msg->encoding = "bgra8"; // Assuming the image is in RGB8 format
    msg->is_bigendian = false;
    msg->step = msg->width * 4; // 3 bytes per pixel for RGB8 encoding

    // Allocate memory for ROS message data
    msg->data.resize(msg->step * msg->height);

    // Copy image data
    const auto* raw_data = reinterpret_cast<const uint8_t*>(carla_image.data());
    std::copy(raw_data, raw_data + (msg->step * msg->height), msg->data.begin());


    // Publish the message
    //RCLCPP_INFO(this->get_logger(), "cam pub2");
    publisher->publish(std::move(msg));

}

// publishImage 함수 수정 (덮어쓰기)
void CameraPublisher::publishImage(const csd::Image &carla_image, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher, void *shared_memory, std::string name) {
    //RCLCPP_INFO(this->get_logger(), "Publishing image and updating shared memory");
    auto msg = std_msgs::msg::String();
    const auto* raw_data = reinterpret_cast<const uint8_t*>(carla_image.data());
    msg.data = name;

    // 이미지 데이터를 공유 메모리에 덮어쓰기
    std::memcpy(shared_memory, raw_data, carla_image.GetWidth() * 4 * carla_image.GetHeight());
    msync(shared_memory, carla_image.GetWidth() * 4 * carla_image.GetHeight(), MS_SYNC);
    publisher->publish(msg);
}

void *CameraPublisher::initialize_shared_memory(const std::string &name) {
    int shm_fd = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "Failed to create shared memory: " << name << std::endl;
        return nullptr;
    }

    if (ftruncate(shm_fd, SHARED_MEMORY_SIZE) == -1) {
        std::cerr << "Failed to set size of shared memory: " << name << std::endl;
        close(shm_fd);
        return nullptr;
    }

    void *shared_memory = mmap(0, SHARED_MEMORY_SIZE, PROT_WRITE | PROT_READ, MAP_SHARED, shm_fd, 0);
    if (shared_memory == MAP_FAILED) {
        std::cerr << "Failed to map shared memory: " << name << std::endl;
        close(shm_fd);
        return nullptr;
    }

    close(shm_fd);
    return shared_memory;
}

void CameraPublisher::SyncSubCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    for(int i=0; i<num_cameras_; ++i){
        auto msg = std_msgs::msg::String();
        msg.data = shm_name[i];
        shm_publishers_[i]->publish(msg);
    }
}
