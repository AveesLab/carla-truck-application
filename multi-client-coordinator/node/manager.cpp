#include "manager.hpp"
const float epsilon = 0.0001f;
/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}

SyncManager::SyncManager(): Node("sync_manager_node"), registration_(10,false), sync_throttle(10,false), sync_steer(10,false) , sync_soc(10,false) 
{

	//------------Connect CARLA------------
    client = new cc::Client(host, port);
    world = new cc::World(client->GetWorld());
    client->SetTimeout(10s);
    
    //------------Parameters------------
    declare_parameter<double>("dt_second", 0.04);
    //declare_parameter<double>("sub_dt_second", 0.01);
    declare_parameter<int>("num_of_sub_dt", 4);
    const double dt_value  = get_parameter("dt_second").as_double();
    //const double sub_dt_value  = get_parameter("sub_dt_second").as_double();
    const int num_of_sub_dt  = get_parameter("num_of_sub_dt").as_int();

    const int    clamped_sub_dt = (num_of_sub_dt > 0) ? num_of_sub_dt : 1;
    const double sub_dt = dt_value / static_cast<double>(clamped_sub_dt);
    
    //------------Setting QOS------------
    rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    custom_qos.reliable();
	
	//------------ROS Topic ------------
    TruckSizeSubscriber_ = this->create_subscription<std_msgs::msg::Int32>("/numtruckss", 10, std::bind(&SyncManager::TruckSizeSubCallback, this, std::placeholders::_1));
    RegistrationSubscriber_ = this->create_subscription<std_msgs::msg::Int32>("/registration", 10, std::bind(&SyncManager::RegistrationSubCallback, this, std::placeholders::_1));
    SyncThrottleSubscriber_ = this->create_subscription<std_msgs::msg::Int32>("/sync_throttle", 10, std::bind(&SyncManager::SyncThrottleSubCallback, this, std::placeholders::_1));
    SyncSteerSubscriber_ = this->create_subscription<std_msgs::msg::Int32>("/sync_steer", 10, std::bind(&SyncManager::SyncSteerSubCallback, this, std::placeholders::_1));
    SyncSOCSubscriber_ = this->create_subscription<std_msgs::msg::Int32>("/sync_soc", 10, std::bind(&SyncManager::SyncSOCSubCallback, this, std::placeholders::_1));
    // SyncDragSubscriber_ = this->create_subscription<std_msgs::msg::Int32>("/sync_drag", 10, std::bind(&SyncManager::SyncDragSubCallback, this, std::placeholders::_1));
    SyncEnuSubscriber_ = this->create_subscription<geometry_msgs::msg::Point>("truck0/server/enu", 10, [&](const geometry_msgs::msg::Point::SharedPtr msg){
        enu = true;
    });

    //Steer
    LVSteerSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("/truck0/steer_control", custom_qos, std::bind(&SyncManager::LVSteerSubCallback, this, std::placeholders::_1));
    FV1SteerSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("/truck1/steer_control", custom_qos, std::bind(&SyncManager::FV1SteerSubCallback, this, std::placeholders::_1));
    FV2SteerSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("/truck2/steer_control", custom_qos, std::bind(&SyncManager::FV2SteerSubCallback, this, std::placeholders::_1));

    ShutdownPublisher_ = this->create_publisher<std_msgs::msg::String>("/shutdown_topic",10);
    isNodeRunning_ = true;
    TarPub_ = this->create_publisher<ros2_msg::msg::Target>("/truck0/target",10);
    FramePub_ = this->create_publisher<std_msgs::msg::UInt32>("/sim/frame_id", 10);
    
	//------------Setting Synchronous mode------------
    //world = new cc::World(client->ReloadWorld(true));
    settings = world->GetSettings();
    settings.synchronous_mode = true;
    // settings.fixed_delta_seconds = 0.04;
    // settings.max_substep_delta_time = 0.01;
    // settings.max_substeps = 4;
    settings.fixed_delta_seconds = static_cast<float>(dt_value);
    settings.max_substep_delta_time  = static_cast<float>(sub_dt);
    settings.max_substeps            = clamped_sub_dt;
    // settings.max_substep_delta_time = static_cast<float>(sub_dt_value);
    // settings.max_substeps = num_of_sub_dt;
    // settings.no_rendering_mode = true; 
    world->ApplySettings(settings,time_);
    
    //------------Save Delta t value(s/ms/ns)------------
    using ns = std::chrono::nanoseconds;
    const double dt_s = settings.fixed_delta_seconds.get_value_or(dt_value);
    period_ns_        = std::chrono::duration_cast<ns>(std::chrono::duration<double>(dt_s));
    period_ms_        = dt_s * 1000.0;

    if (period_ns_.count() <= 0) {
        RCLCPP_ERROR(get_logger(), "Invalid fixed_delta_seconds=%.6f", settings.fixed_delta_seconds);
    }
    
    
    //------------Open .csv------------
    prof_csv_.open(prof_path_, std::ios::out | std::ios::trunc);
	if (prof_csv_.is_open()) {
		prof_csv_ << "frame,gap_to_deadline_ms,wall_elapsed_ms,ideal_elapsed_ms,drift_ms,ctrl_latency_ms\n";
	}
    
    //------------Create Thread------------
    manager_thread_ = std::thread(&SyncManager::managerInThread, this);
	tick_thread_ = std::thread(&SyncManager::tickSchedulerThread, this);

    callback_id = world->OnTick([&](cc::WorldSnapshot snapshot) 
    {
		sim_time = snapshot.GetTimestamp().elapsed_seconds;
        if (!(registered && first && go_time && enu)) return;
        recordData();
    });
    
    RCLCPP_INFO(get_logger(),"Initialize Finish: fixed=%.3f s (%.1f ms), max_substep_dt=%.3f s (%.2f ms), max_substeps=%d", dt_value, dt_value * 1000.0, sub_dt,  sub_dt * 1000.0, clamped_sub_dt);
}


SyncManager::~SyncManager(void)
{
    isNodeRunning_ = false;
    
    if (manager_thread_.joinable()) manager_thread_.join();
    if (tick_thread_.joinable())    tick_thread_.join();
    
    //---------OnTick 콜백 제거---------
    if (world && callback_id) {
        try { world->RemoveOnTick(callback_id); }
        catch (...) { /* ignore */ }
    }
    
    //---------Sync 모드 종료---------
    try {
        settings.synchronous_mode = false;
        if (world) world->ApplySettings(settings, time_);
    } catch (...) {}

    ////--------Actor 정리---------
    try {
        auto actor_list = world ? world->GetActors() : nullptr;
        if (actor_list) {
            for (size_t i = 0; i < truck_ids.size(); ++i) {
                if (auto a = actor_list->Find(truck_ids[i])) {
                    if (auto v = boost::dynamic_pointer_cast<cc::Vehicle>(a)) v->Destroy();
                }
                if (auto t = actor_list->Find(trailer_ids[i])) {
                    if (auto v = boost::dynamic_pointer_cast<cc::Vehicle>(t)) v->Destroy();
                }
            }
        }
    } catch (...) {}

    ////---------World/Client 메모리 해제---------
    delete world;
    world = nullptr;
    delete client;
    client = nullptr;

    std::cerr << "pub shutdown" << std::endl;
}


void SyncManager::TruckSizeSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    std::cerr << "TruckSizeSubCallback : "<< msg->data << std::endl;
    unique_lock<mutex> lock(mutex_);
    this->size = msg->data;
    truck_ids.resize(this->size);
    trailer_ids.resize(this->size);
}

void SyncManager::RegistrationSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    unique_lock<mutex> lock(mutex_);
    std::cerr << "RegistrationSubCallback : "<< msg->data << std::endl;
    registration_[msg->data] = true;
}


void SyncManager::LVSteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    lv_steer = msg->data;
}

void SyncManager::FV1SteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    fv1_steer = msg->data;
}

void SyncManager::FV2SteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    fv2_steer = msg->data;
}


void SyncManager::SyncThrottleSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    unique_lock<mutex> lock(mutex_);
    sync_throttle[msg->data] = true;
}

void SyncManager::SyncSteerSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    unique_lock<mutex> lock(mutex_);
    sync_steer[msg->data] = true;
}

void SyncManager::SyncSOCSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    unique_lock<mutex> lock(mutex_);
    sync_soc[msg->data] = true;
}



bool SyncManager::check_register() 
{
    if(size == 0) return false;
    if(registered) return true;
    unique_lock<mutex> lock(mutex_);
    for(int i = 0; i<size; i++) 
    {
        if(registration_[i] == false) return false;
    }
    std::cerr << "tick for register " << std::endl;
    if(cnt == 0) 
    {
        
        std_msgs::msg::UInt32 frame_msg;
        frame_msg.data = frame_k.load();
        FramePub_->publish(frame_msg);
     
        world->Tick(time_);
        
        //FindAllTruck();
        cnt = 1;
        for(int i = 0; i<size; i++) {
            if(registration_[i] == true) registration_[i] = false;
        }
        return false;
    }
    else if(cnt == 1) 
    {
        registered = true;
        std::cerr << "All registered" << std::endl;
        FindAllTruck();

        std_msgs::msg::UInt32 frame_msg;
        //frame_msg.data = static_cast<uint32_t>(sim_time / 40.0f); // 또는 frame counter
        frame_msg.data = frame_k.load();
        FramePub_->publish(frame_msg);

        world->Tick(time_);

        return true;
    }
    return false;
}

void SyncManager::FindAllTruck() 
{
    vehicles_.clear();
    vehicles_.resize(this->size);    // size = number of trucks

    auto actor_list = world->GetActors();
    for (int i = 0; i < this->size; ++i) {
        const std::string truck_name   = "truck"   + std::to_string(i);
        const std::string trailer_name = "trailer" + std::to_string(i);
        for (auto it = actor_list->begin(); it != actor_list->end(); ++it) {
            ActorPtr actor = *it;
            if (actor->GetTypeId().front() != 'v') continue;
            for (auto&& attr : actor->GetAttributes()) {
                if (attr.GetValue() == truck_name) {
                    truck_ids[i] = actor->GetId();
                    vehicles_[i] = boost::dynamic_pointer_cast<cc::Vehicle>(actor);
                } else if (attr.GetValue() == trailer_name) {
                    trailer_ids[i] = actor->GetId();
                }
            }
        }
        if (!vehicles_[i]) {
            RCLCPP_WARN(this->get_logger(), "Vehicle cache miss for %s", truck_name.c_str());
        }
    }
}


bool SyncManager::sync_received() 
{
    unique_lock<mutex> lock(mutex_);
    // sync_throttle 배열의 모든 원소가 true인지 확인
    for (int i = 0; i < size; i++) 
    {
        if (sync_throttle[i] == false) 
        {
            return false; // 하나라도 false면 false 반환
        }
    }

    // sync_steer 배열의 모든 원소가 true인지 확인
    for (int i = 0; i < size; i++) 
    {
        if (sync_steer[i] == false) 
        {
            return false; // 하나라도 false면 false 반환
        }
    }

    //sync_steer 배열의 모든 원소가 true인지 확인
    for (int i = 0; i < size; i++) 
    {
        if (sync_soc[i] == false) 
        {
            return false; // 하나라도 false면 false 반환
        }
    }

    //all received
    for(int i = 0; i<size; i++) 
    {
        sync_throttle[i] = false;
    }
    for(int i = 0; i<size; i++) 
    {
        sync_steer[i] = false;
    }
    for(int i = 0; i<size; i++) 
    {
        sync_soc[i] = false;
    }
    
    RCLCPP_INFO(this->get_logger(), "All received");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return true;

}

void SyncManager::managerInThread() 
{
    int last_frame = -1;

    while (isNodeRunning_) 
    {
        if (check_register()) 
        {
            if (!first) 
            {
                FindAllTruck();
                first = true;
                RCLCPP_INFO(this->get_logger(), "Start");
            }

            if (sync_received()) 
            {
                auto current_frame = frame_k.load(std::memory_order_acquire);

                if (current_frame != last_frame) 
                {
                    // 지금 시각(ns)
                    using clock = std::chrono::steady_clock;
                    using ns    = std::chrono::nanoseconds;
                    auto now_tp = std::chrono::time_point_cast<ns>(clock::now());
                    int64_t now_ns = now_tp.time_since_epoch().count();
                    last_sync_finish_ns_.store(now_ns, std::memory_order_release);

                    // tick 시작 시각 읽기
                    int64_t tick_ns = last_tick_start_ns_.load(std::memory_order_acquire);
                    if (tick_ns != 0) 
                    {
                        double latency_ms = double(now_ns - tick_ns) / 1e6;  // ms로 변환
                        RCLCPP_INFO(this->get_logger(), "[SYNC : Frame %d] control path latency = %.3f ms", current_frame, latency_ms);
                    }
                    
                    go_time = true;
                    tick_request_ = true; // ✅ 별도 tick thread에서 처리할 flag
                    last_frame = current_frame;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 최소 부하
    }
}

void SyncManager::tickSchedulerThread()
{
    using clock = std::chrono::steady_clock;
    using ns    = std::chrono::nanoseconds;
    using TP    = std::chrono::time_point<clock, ns>;

    bool   t0_init = false;
    TP     t0;
    TP     first_deadline;               // 이벤트 모드에선 안 써도 됨
    uint32_t prev_frame = 0;
    TP     prev_t_start;
    TP     prev_deadline_tp;             // 이벤트 모드에선 "이전 tick 시각" 정도로 쓰면 됨

    if(!t0_init) {
        const uint32_t k = frame_k.fetch_add(1) + 1;
        std_msgs::msg::UInt32 msg;
        msg.data = k;
        FramePub_->publish(msg);
    }

    while (isNodeRunning_) {
        // manager 스레드가 tick_request_ 세워줄 때까지 잠깐 쉼
        if (!tick_request_.exchange(false, std::memory_order_acq_rel)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 여기 오면 "컨트롤 다 왔음, 이제 tick 하자" 상태
        TP t_start = std::chrono::time_point_cast<ns>(clock::now());
        last_tick_start_ns_.store(t_start.time_since_epoch().count(), std::memory_order_release);

        // 직전 프레임에 대한 timing 기록
        if (prev_frame != 0) {
            // 이벤트 모드니까 deadline 대신 "직전 tick 시각 + period"를 가짜 deadline 으로 넣어줄 수 있음
            TP fake_deadline = prev_t_start + period_ns_;
            recordTiming(prev_frame, prev_t_start, fake_deadline, period_ns_, t0_init ? t0 : t_start, t0_init ? first_deadline : t_start);
        }

        if (!t0_init) {
            t0 = t_start;
            first_deadline = t_start;  // 이벤트 모드라 그냥 시작시각 넣어둠
            t0_init = true;
        }


        // frame publish
        const uint32_t k = frame_k.fetch_add(1) + 1;
        std_msgs::msg::UInt32 msg;
        msg.data = k;
        FramePub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Tick for frame %u", k);

        RCLCPP_INFO(this->get_logger(), "Before Tick");
        // 실제 CARLA step
        world->Tick(time_);
        RCLCPP_INFO(this->get_logger(), "After Tick");


        // 다음 recordTiming을 위해 저장
        prev_frame       = k;
        prev_t_start     = t_start;
        prev_deadline_tp = t_start + period_ns_;
    }
}

void SyncManager::recordTiming(uint32_t frame, const std::chrono::steady_clock::time_point& t_start, const std::chrono::steady_clock::time_point& prev_deadline,
    const std::chrono::nanoseconds& period_ns, const std::chrono::steady_clock::time_point& t0, const std::chrono::steady_clock::time_point& first_deadline)
{
    using namespace std::chrono;

    // 1) deadline 대비 gap
    const auto gap_to_deadline = t_start - prev_deadline;
    const double gap_ms = double(duration_cast<nanoseconds>(gap_to_deadline).count()) / 1e6;
    
    // 2) 전체 경과
    const double wall_elapsed_ms = duration<double, std::milli>(t_start - t0).count();

    // 3) 이상적인 경과
    const double ideal_elapsed_ms = duration<double, std::milli>(prev_deadline - first_deadline).count();

    // 4) drift
    const double drift_ms = wall_elapsed_ms - ideal_elapsed_ms;

    // 5) control latency (이전 tick 시작 ~ sync 끝)
    double ctrl_latency_ms = 0.0;
    {
        // ✅ 여기서는 전역 말고, 이 프레임의 tick 시작시각(t_start)을 직접 쓴다
        const int64_t tick_ns = t_start.time_since_epoch().count();
        const int64_t sync_ns = last_sync_finish_ns_.load(std::memory_order_acquire);

        if (sync_ns > tick_ns && tick_ns != 0) {
            ctrl_latency_ms = double(sync_ns - tick_ns) / 1e6;
        }

        // 여기서 진짜 deadline 검사도 같이 해버릴 수 있음
        const int64_t deadline_ns = prev_deadline.time_since_epoch().count();
        const int64_t control_deadline_ns = deadline_ns + period_ns.count();


        if (frame > warmup_frames_ && sync_ns != 0 && sync_ns > control_deadline_ns) {
            RCLCPP_WARN(this->get_logger(), "[Tick] frame %u: No control-sync before deadline (ZOH).", frame);
        }

    }
    // 6) CSV 출력
    if (prof_csv_.is_open()) {
        prof_csv_ << frame << ","
                  << std::fixed << std::setprecision(6)
                  << gap_ms << ","
                  << wall_elapsed_ms << ","
                  << ideal_elapsed_ms << ","
                  << drift_ms << ","
                  << ctrl_latency_ms << "\n";
    }
    // 7) 통계
    if (frame > warmup_frames_) {
        gaps_ms_.push_back(gap_ms);
        drift_ms_.push_back(drift_ms);
        analyzed_frames_++;
        if (gap_to_deadline.count() > 0) {
            over_cnt_++;
        }
    }
}



void SyncManager::recordData() {
    static struct timeval prevTime = {0};
    struct timeval currentTime;
    gettimeofday(&currentTime, nullptr);

    // Δt (초) 계산
    double diff_time = 0.0;
    if (prevTime.tv_sec != 0) {
        diff_time = (currentTime.tv_sec - prevTime.tv_sec)
                  + (currentTime.tv_usec - prevTime.tv_usec) / 1e6;
    }
    prevTime = currentTime;

    // 파일 스트림 캐시는 그대로 OK (open/close 비용 제거)
    static std::vector<std::ofstream> file_streams;
    static size_t flush_every = 25;     // 25프레임마다 flush
    static size_t flush_count = 0;

    if (file_streams.empty()) {
        file_streams.resize(truck_ids.size());
        for (size_t i = 0; i < truck_ids.size(); ++i) {
            std::string path = "/home/avees/ros2_ws/src/test_truck/truck" + std::to_string(i+1) + ".csv";
            file_streams[i].open(path, std::ios::out | std::ios::app);
            if (file_streams[i].tellp() == 0) {
                file_streams[i] << "Time,ActorID,Velocity,Acceleration,Distance,"
                                 << "location_x,location_y,lateral_error,steer,WallTimeDiff\n";
            }
        }
    }

    for (size_t i = 0; i < vehicles_.size(); ++i) {
        auto veh = vehicles_[i];
        if (!veh) continue;  // 캐시 못했으면 skip

        auto vel = veh->GetVelocity();
        auto acc = veh->GetAcceleration();
        auto loc = veh->GetLocation();

        double v = std::hypot(std::hypot(vel.x, vel.y), vel.z);
        v = std::round(v * 100.0) / 100.0;
        double ax = acc.x;

        float dist = (i==0) ? lv_dist : (i==1) ? fv1_dist : fv2_dist;
        float err  = (i==0) ? lv_error: (i==1) ? fv1_error: fv2_error;
        float str  = (i==0) ? lv_steer: (i==1) ? fv1_steer: fv2_steer;

        auto& ofs = file_streams[i];
        ofs << std::fixed << std::setprecision(2)
            << sim_time << ","
            << veh->GetId() << ","
            << v << ","
            << ax << ","
            << dist << ","
            << loc.x << ","
            << loc.y << ","
            << err << ","
            << str << ","
            << std::setprecision(6) << diff_time << "\n";
    }

    // 즉시 flush 대신 **주기적 flush**로 I/O 지연 줄이기
    if (++flush_count % flush_every == 0) {
        for (auto& ofs : file_streams) ofs.flush();
        flush_count = 0;
    }
}
