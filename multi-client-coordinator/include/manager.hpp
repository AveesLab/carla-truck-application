#include <iostream>
#include <fstream>
#include <string>
#include <sys/time.h>
#include <vector>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <thread>
#include <shared_carlalib.h>
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <sys/mman.h>
#include <sys/stat.h>  /* For mode constants */
#include <fcntl.h>     /* For O_* constants */
#include <unistd.h>    /* For ftruncate */
#include <cstring>     /* For memset */
#include <thread>
#include <string>
#include <ros2_msg/msg/target.hpp>
using namespace carla::traffic_manager;
#define SHARED_MEMORY_NAME "/sync_memory"
using namespace std;

class SyncManager : public rclcpp::Node {

public:
    SyncManager();
    ~SyncManager();
private:
    std::string host = "localhost";
    uint16_t port = 2000u;
    cc::Client* client;
    cc::World* world;
    carla::time_duration time_;
    carla::rpc::EpisodeSettings settings;
    bool first = false;
    bool first_check = false;
    mutex mutex_;
    bool isNodeRunning_ = false;
    bool registered = false;
    bool go_time = false;
    int size = 0;
    int cnt = 0;
    float sim_time = 0.0f;
    bool enu = false;
    std::vector<bool> registration_;
    std::vector<bool> sync_throttle;
    std::vector<bool> sync_steer;
    std::vector<bool> sync_drag;
    std::vector<bool> sync_soc;
    std::thread manager_thread_;
    std::thread tick_thread_;
    std::atomic<bool> tick_request_ = false;
    
    vector<unsigned int> truck_ids;
    vector<unsigned int> trailer_ids;
    void managerInThread();
    void tickSchedulerThread();
    void FindAllTruck();
    void recordData();
    void recordTiming(uint32_t frame, const std::chrono::steady_clock::time_point& t_start, const std::chrono::steady_clock::time_point& prev_deadline,
    const std::chrono::nanoseconds& period_ns, const std::chrono::steady_clock::time_point& t0, const std::chrono::steady_clock::time_point& first_deadline);
    
    size_t callback_id;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ShutdownPublisher_;
    rclcpp::Publisher<ros2_msg::msg::Target>::SharedPtr TarPub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr GapPub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr LC0Pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr LC1Pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr LC2Pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr FramePub_;
    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr TruckSizeSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr RegistrationSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr SyncSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr SyncThrottleSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr SyncSteerSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr SyncDragSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr SyncSOCSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr SyncEnuSubscriber_;

    //callback
    void TruckSizeSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void RegistrationSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void SyncThrottleSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void SyncSteerSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void SyncDragSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void SyncSOCSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    bool check_register();
    bool sync_received();

    //steer
    float lv_steer = 0.0f;
    float fv1_steer = 0.0f;
    float fv2_steer = 0.0f;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr LVSteerSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr FV1SteerSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr FV2SteerSubscriber_;
    void LVSteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void FV1SteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void FV2SteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg);

    //lateral_error
    float lv_error = 0.0f;
    float fv1_error =0.0f;
    float fv2_error=0.0f;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr LVErrorSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr FV1ErrorSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr FV2ErrorSubscriber_;
    void LVErrorSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void FV1ErrorSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void FV2ErrorSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    
    //min_distance
    float lv_dist = 0.0f;
    float fv1_dist = 0.0f;
    float fv2_dist = 0.0f;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr LVDistSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr FV1DistSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr FV2DistSubscriber_;
    void LVDistSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void FV1DistSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void FV2DistSubCallback(const std_msgs::msg::Float32::SharedPtr msg);

    int fd;
    int* shared_mem_ptr;

    std::vector<boost::shared_ptr<cc::Vehicle>> vehicles_;

    // ★ 추가: 프레임 카운터 & 갭 계측용
    std::atomic<uint32_t> frame_k{0};

    // --- E1 profiler additions ---
    std::ofstream prof_csv_;
    std::string prof_path_ = "/home/avees/ros2_ws/src/test_truck/e1_profiler.csv";
    uint64_t warmup_frames_ = 300;

    std::chrono::steady_clock::time_point t0_{};
    bool t0_init_ = false;

    double period_ms_ = 0.0;
    std::chrono::nanoseconds period_ns_{0};
    std::atomic<int64_t> last_tick_start_ns_{0};   // tick 시작 시각
    std::atomic<int64_t> last_sync_finish_ns_{0};  // sync 완료 시각

    uint64_t analyzed_frames_ = 0;
    uint64_t over_cnt_ = 0;

    std::vector<double> gaps_ms_;   // GapStartMs (warmup 제외)
    std::vector<double> drift_ms_;  // Drift trajectory (warmup 제외)
    // manager.hpp 안에
    std::condition_variable tick_cv_;
    std::mutex tick_mtx_;

};
