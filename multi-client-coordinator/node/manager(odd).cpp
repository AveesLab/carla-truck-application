#include "manager.hpp"
const float epsilon = 0.0001f;
/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}

bool go_time = false;

// manager.cpp

SyncManager::SyncManager()
: Node("sync_manager_node"), registration_(10,false), sync_throttle(10,false), sync_steer(10,false)
{
    // 1) params
    declare_parameter<double>("dt_second", 0.04);
    declare_parameter<std::string>("prof_path", prof_path_);
    declare_parameter<int>("warmup_frames", static_cast<int>(warmup_frames_));

    const double dt_value  = get_parameter("dt_second").as_double();
    prof_path_             = get_parameter("prof_path").as_string();
    warmup_frames_         = static_cast<uint64_t>(get_parameter("warmup_frames").as_int());

    // 2) CARLA
    client = new cc::Client(host, port);
    client->SetTimeout(10s);
    world = new cc::World(client->GetWorld());

    // 3) CARLA settings
    settings = world->GetSettings();
    settings.synchronous_mode       = true;
    settings.fixed_delta_seconds    = static_cast<float>(dt_value);
    settings.max_substep_delta_time = 0.01f;
    settings.max_substeps           = 4;
    world->ApplySettings(settings, time_);

    // 4) period 계산을 멤버에 저장
    using ns = std::chrono::nanoseconds;
    const double dt_s = settings.fixed_delta_seconds.get_value_or(dt_value);
    period_ns_        = std::chrono::duration_cast<ns>(std::chrono::duration<double>(dt_s));
    period_ms_        = dt_s * 1000.0;

    if (period_ns_.count() <= 0) {
        RCLCPP_ERROR(get_logger(), "Invalid fixed_delta_seconds=%.6f", settings.fixed_delta_seconds);
    }

    // 5) CSV open (한 번만)
    prof_csv_.open(prof_path_, std::ios::out | std::ios::trunc);
    if (prof_csv_.is_open()) {
        prof_csv_ << "frame,gap_to_deadline_ms,wall_elapsed_ms,ideal_elapsed_ms,drift_ms\n";
        RCLCPP_INFO(get_logger(), "[E1] profiler csv opened: %s", prof_path_.c_str());
    } else {
        RCLCPP_WARN(get_logger(), "[E1] Cannot open profiler csv: %s", prof_path_.c_str());
    }

    // 6) 실행 플래그
    isNodeRunning_ = true;

    // 7) manager thread 먼저
    manager_thread_ = std::thread(&SyncManager::managerInThread, this);

    // 8) tick thread (멤버 period_ns_ 사용)
    tick_thread_ = std::thread(&SyncManager::tickSchedulerThread, this);

    // 9) OnTick 콜백
    callback_id = world->OnTick([&](cc::WorldSnapshot snapshot) {
        sim_time = snapshot.GetTimestamp().elapsed_seconds;
        if (!(registered && first && go_time && enu)) return;
        recordData();
    });

    RCLCPP_INFO(get_logger(), "Initialize Finish using dt_second = %.3f sec", dt_value);
}




SyncManager::~SyncManager() 
{
    // 1) 먼저 실행 정지 신호
    isNodeRunning_ = false;


    // 2) 스레드 join (둘 다!)
    if (manager_thread_.joinable()) manager_thread_.join();
    if (tick_thread_.joinable())    tick_thread_.join();

    // 3) OnTick 콜백 제거 (등록되어 있다면)
    if (world && callback_id) {
        try { world->RemoveOnTick(callback_id); }
        catch (...) { /* ignore */ }
    }

    // 4) (이제 아무도 쓰지 않음) 요약 통계 & CSV close
    auto pct = [](std::vector<double> v, double p)->double {
        if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
        std::sort(v.begin(), v.end());
        const double idx = std::clamp(p * (v.size() - 1), 0.0, double(v.size()-1));
        const size_t i = size_t(idx);
        const double frac = idx - i;
        return (i+1 < v.size()) ? v[i]*(1.0-frac) + v[i+1]*frac : v[i];
    };

    if (analyzed_frames_ > 0) {
        const double p50   = pct(gaps_ms_, 0.50);
        const double p90   = pct(gaps_ms_, 0.90);
        const double p99   = pct(gaps_ms_, 0.99);
        const double p999  = pct(gaps_ms_, 0.999);
        const double max_v = *std::max_element(gaps_ms_.begin(), gaps_ms_.end());
        const double overp = 100.0 * double(over_cnt_) / double(analyzed_frames_);
        const double drift_final = drift_ms_.empty() ? 0.0 : drift_ms_.back();

        RCLCPP_INFO(this->get_logger(),
            "[E1] Δt=%.1f ms | frames=%lu | Over%%=%.5f | P50=%.3f | P90=%.3f | P99=%.3f | P99.9=%.3f | Max=%.3f | Drift_final=%.3f ms",
            period_ms_, analyzed_frames_, overp, p50, p90, p99, p999, max_v, drift_final);

        // if (prof_csv_.is_open()) {
        //     prof_csv_ << "# SUMMARY,Dt_ms," << period_ms_
        //               << ",Frames," << analyzed_frames_
        //               << ",Over_pct," << overp
        //               << ",P50,"  << p50
        //               << ",P90,"  << p90
        //               << ",P99,"  << p99
        //               << ",P99.9,"<< p999
        //               << ",Max,"  << max_v
        //               << ",Drift_final_ms," << drift_final << "\n";
        // }
    }
    if (prof_csv_.is_open()) { prof_csv_.flush(); prof_csv_.close(); }

    // 5) CARLA를 비동기 모드로 되돌리기 (스레드 종료 후!!)
    try {
        settings.synchronous_mode = false;
        if (world) world->ApplySettings(settings, time_);
    } catch (...) {}

    // 6) 액터 정리
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

    // 7) world/client 메모리 해제 (또는 unique_ptr로 관리 권장)
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

bool SyncManager::check_register() 
{
    if(size == 0 ) return false;
    if(registered) return true;
    unique_lock<mutex> lock(mutex_);
    for(int i = 0; i<size; i++) 
    {
        if(registration_[i] == false) return false;
    }
    std::cerr << "tick for register " << std::endl;
    if(cnt == 0) 
    {
        //world->Tick(time_);
        
        std_msgs::msg::UInt32 frame_msg;
        //frame_msg.data = static_cast<uint32_t>(sim_time / 40.0f); // 또는 frame counter
        frame_msg.data = frame_k.load();
        FramePub_->publish(frame_msg);
        
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
        //world->Tick(time_);

        std_msgs::msg::UInt32 frame_msg;
        //frame_msg.data = static_cast<uint32_t>(sim_time / 40.0f); // 또는 frame counter
        frame_msg.data = frame_k.load();
        FramePub_->publish(frame_msg);

        return true;
    }
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

void SyncManager::managerInThread() 
{
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
                // 지금 시각(ns)
                using clock = std::chrono::steady_clock;
                using ns    = std::chrono::nanoseconds;
                auto now_tp = std::chrono::time_point_cast<ns>(clock::now());
                int64_t now_ns = now_tp.time_since_epoch().count();
                last_sync_finish_ns_.store(now_ns, std::memory_order_release);

                // tick 시작 시각 읽기
                int64_t tick_ns = last_tick_start_ns_.load(std::memory_order_acquire);
                if (tick_ns != 0) {
                    double latency_ms = double(now_ns - tick_ns) / 1e6;  // ms로 변환
                    RCLCPP_INFO(this->get_logger(),
                        "[SYNC] control path latency = %.3f ms", latency_ms);
                }
                
                go_time = true;
                tick_request_ = true; // ✅ 별도 tick thread에서 처리할 flag
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 최소 부하
    }
}

void SyncManager::tickSchedulerThread()
{
    try {
        using clock = std::chrono::steady_clock;
        using ns    = std::chrono::nanoseconds;
        using TP    = std::chrono::time_point<clock, ns>;

        TP next_deadline = std::chrono::time_point_cast<ns>(clock::now()) + period_ns_;

        bool t0_init = false;
        bool first_deadline_init = false;
        TP t0, first_deadline;

        while (isNodeRunning_) 
        {
            std::this_thread::sleep_until(next_deadline);
            const TP t_start = std::chrono::time_point_cast<ns>(clock::now());
            last_tick_start_ns_.store(t_start.time_since_epoch().count(), std::memory_order_release);

            if (!t0_init) {
                t0 = t_start;
                t0_init = true;
            }
            if (!first_deadline_init) {
                first_deadline = next_deadline - period_ns_;
                first_deadline_init = true;
            }

            const ns gap_to_deadline = t_start - next_deadline;
            const double gap_ms = static_cast<double>(gap_to_deadline.count()) / 1e6;

            const bool had_sync = tick_request_.exchange(false, std::memory_order_acq_rel);
            if (!had_sync) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,"[Tick] No control-sync before deadline (ZOH).");
            }

            // 실제 step
            world->Tick(time_);

            // frame publish
            const uint32_t k = frame_k.fetch_add(1) + 1;
            std_msgs::msg::UInt32 frame_msg;
            frame_msg.data = k;
            FramePub_->publish(frame_msg);


            // drift 계산
            const double wall_elapsed_ms = std::chrono::duration<double, std::milli>(t_start - t0).count();
            const TP prev_deadline = next_deadline - period_ns_;
            const double ideal_elapsed_ms = std::chrono::duration<double, std::milli>(prev_deadline - first_deadline).count();
            const double drift_ms = wall_elapsed_ms - ideal_elapsed_ms;
            double ctrl_latency_ms = 0.0;
            {
                int64_t sync_ns = last_sync_finish_ns_.load(std::memory_order_acquire);
                int64_t tick_ns = last_tick_start_ns_.load(std::memory_order_acquire);
                if (sync_ns > tick_ns && tick_ns != 0) {
                    ctrl_latency_ms = double(sync_ns - tick_ns) / 1e6;
                }
            }

            
            if (prof_csv_.is_open()) 
            {
                prof_csv_ << k << ","
                        << std::fixed << std::setprecision(6)
                        << gap_ms << ","
                        << wall_elapsed_ms << ","
                        << ideal_elapsed_ms << ","
                        << drift_ms << ","
                        << ctrl_latency_ms << "\n";   // ← 새 칼럼
            }

            // 통계 집계
            if (k > warmup_frames_) {
                gaps_ms_.push_back(gap_ms);
                drift_ms_.push_back(drift_ms);
                analyzed_frames_++;
                if (gap_to_deadline.count() > 0) {
                    over_cnt_++;
                }
            }

            next_deadline += period_ns_;
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "[TickThread] Exception: %s", e.what());
        isNodeRunning_ = false; 
    }
}


bool SyncManager::sync_received() {
    unique_lock<mutex> lock(mutex_);
    // sync_throttle 배열의 모든 원소가 true인지 확인
    for (int i = 0; i < size; i++) {
        if (sync_throttle[i] == false) {
            return false; // 하나라도 false면 false 반환
        }
    }

    // sync_steer 배열의 모든 원소가 true인지 확인
    for (int i = 0; i < size; i++) {
        if (sync_steer[i] == false) {
            return false; // 하나라도 false면 false 반환
        }
    }

    //all received
    for(int i = 0; i<size; i++) {
        sync_throttle[i] = false;
    }
    for(int i = 0; i<size; i++) {
        sync_steer[i] = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return true;

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