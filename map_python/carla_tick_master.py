
import os
import time
import math
import csv

import carla
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time as RosTime
from std_msgs.msg import UInt32, Int32


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def speed_of(v):  # carla.Vector3D -> float (m/s)
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


class CarlaTickMaster(Node):
    def __init__(self):
        super().__init__('carla_tick_master')

        # --- CARLA 연결 ---
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.02  # 50 Hz
        self.world.apply_settings(settings)

        applied = self.world.get_settings()
        self.get_logger().info(f"[SYNC MODE] {applied.synchronous_mode}, Δt={applied.fixed_delta_seconds:.3f}s")

        # --- Publisher ---
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.frame_pub = self.create_publisher(UInt32, '/sim/frame_id', 10)

        # --- Ready flags (3 trucks 예시) ---
        self.num_trucks = 3
        self.sync_throttle_flags = {i: False for i in range(self.num_trucks)}
        self.sync_steer_flags = {i: False for i in range(self.num_trucks)}
        self.sync_throttle_sub = self.create_subscription(Int32, '/sync_throttle', self.sync_throttle_callback, 10)
        self.sync_steer_sub = self.create_subscription(Int32, '/sync_steer', self.sync_steer_callback, 10)

        # --- CSV 로깅 설정 ---
        self.save_dir = os.path.expanduser('~/ros2_ws/src/test_truck')
        ensure_dir(self.save_dir)
        self.csv_files = {}
        self.csv_writers = {}
        self._setup_csv_loggers()

        # --- 차량 식별 ---
        self.vehicles = self.world.get_actors().filter('vehicle.*')
        self.truck_ids = []
        for i in range(self.num_trucks):
            role = f"truck{i}"
            for a in self.vehicles:
                if a.attributes.get('role_name') == role:
                    self.truck_ids.append(a.id)
                    break
        self.get_logger().info(f"[LOG] Tracking vehicles: {self.truck_ids}")

        # --- 내부 변수 ---
        self.current_frame = 0
        self.last_tick_time = time.time()

        self.get_logger().info("✅ CarlaTickMaster initialized. Waiting for control_ready...")

        # 첫 tick 시작
        self.publish_frame_and_tick()


    def _setup_csv_loggers(self):
        for i in range(self.num_trucks):
            path = os.path.join(self.save_dir, f"{i}_truck.csv")
            f = open(path, mode='w', newline='', encoding='utf-8')
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'velocity', 'pitch'])
            f.flush()
            self.csv_files[i] = f
            self.csv_writers[i] = writer
        self.get_logger().info(f"[LOG] CSV initialized under {self.save_dir}")


    # 각 트럭에서 control 완료 신호 수신
    def sync_throttle_callback(self, msg):
        truck_id = msg.data
        self.sync_throttle_flags[truck_id] = True
        #self.get_logger().info(f"[READY] truck{truck_id} throttle ready at frame {self.current_frame}")
        self.check_and_tick()


    def sync_steer_callback(self, msg):
        truck_id = msg.data
        self.sync_steer_flags[truck_id] = True
        #self.get_logger().info(f"[READY] truck{truck_id} steer ready at frame {self.current_frame}")
        self.check_and_tick()


    def check_and_tick(self):
        if all(self.sync_steer_flags.values()) and all(self.sync_throttle_flags.values()):
            self.sync_steer_flags = {i: False for i in range(self.num_trucks)}
            self.sync_throttle_flags = {i: False for i in range(self.num_trucks)}
            self.get_logger().info(f"[READY] All of trucks contol ready at frame {self.current_frame}")
            self.publish_frame_and_tick()


    def publish_frame_and_tick(self):
        # --- CARLA tick ---
        frame_id = self.world.tick()
        self.current_frame = frame_id
        snapshot = self.world.get_snapshot()
        sim_time = snapshot.timestamp.elapsed_seconds

        # --- Clock publish (/clock) ---
        clock_msg = Clock()
        clock_msg.clock = RosTime()
        clock_msg.clock.sec = int(sim_time)
        clock_msg.clock.nanosec = int((sim_time % 1.0) * 1e9)
        self.clock_pub.publish(clock_msg)

        # --- Frame publish (/sim/frame_id) ---
        frame_msg = UInt32()
        frame_msg.data = frame_id
        self.frame_pub.publish(frame_msg)

        # --- Snapshot 기반 CSV 저장 ---
        for i, actor_id in enumerate(self.truck_ids):
            actor_snapshot = snapshot.find(actor_id)
            if actor_snapshot:
                tf = actor_snapshot.get_transform()
                vel = actor_snapshot.get_velocity()
                x, y = tf.location.x, tf.location.y
                pitch = tf.rotation.pitch
                spd = speed_of(vel)
                self.csv_writers[i].writerow([f"{sim_time:.6f}", f"{x:.6f}", f"{y:.6f}", f"{spd:.6f}", f"{pitch:.6f}"])
                self.csv_files[i].flush()

        # --- Log ---
        dt_real = time.time() - self.last_tick_time
        self.last_tick_time = time.time()
        self.get_logger().info(f"[TICK {frame_id}] SimTime={sim_time:.3f}s | Real Δt={dt_real:.3f}s")


    def destroy_node(self):
        for f in self.csv_files.values():
            try:
                f.flush()
                f.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CarlaTickMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🔻 Stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

