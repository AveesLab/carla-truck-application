import carla
import time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time as RosTime

class CarlaTickNode(Node):
    def __init__(self):
        super().__init__('carla_tick_node')

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.02  # Setting Hz
        self.world.apply_settings(settings)

        applied = self.world.get_settings()
        print("[Check] Applied sync mode:", applied.synchronous_mode)
        print("[Check] Applied fixed_delta_seconds:", applied.fixed_delta_seconds)

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.get_logger().info("Tick node with /clock started.")

    def tick_and_publish(self):
        frame_id = self.world.tick()
        snapshot = self.world.get_snapshot()
        sim_time = snapshot.timestamp.elapsed_seconds

        clock_msg = Clock()
        clock_msg.clock = RosTime()
        clock_msg.clock.sec = int(sim_time)
        clock_msg.clock.nanosec = int((sim_time % 1.0) * 1e9)
        self.clock_pub.publish(clock_msg)

        self.get_logger().info(f"[TICK] Sim time: {sim_time:.3f}s")

def main():
    rclpy.init()
    node = CarlaTickNode()

    try:
        while rclpy.ok():
            node.tick_and_publish()
            time.sleep(0.02)  # Real time tick 호출 주기
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

