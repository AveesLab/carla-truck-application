import carla
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
import numpy as np
import math
import sys

class TruckPointPublisher(Node):
    def __init__(self, vehicle, namespace='truck0'):
        super().__init__('truck_Point_publisher', namespace=namespace)
        self.publisher_ = self.create_publisher(Point, 'server/enu', 10)
        self.vehicle = vehicle
        self.timer = self.create_timer(0.1, self.publish_Point)

    def publish_Point(self):
        transform = self.vehicle.get_transform()
        loc = transform.location
        #rot = transform.rotation
        
        msg = Point()
        msg.x = loc.x
        msg.y = loc.y
        msg.z = loc.z
        

        #msg = Pose()
        #msg.position.x = loc.x
        #msg.position.y = loc.y
        #msg.position.z = loc.z

        #cy = math.cos(math.radians(rot.yaw) * 0.5)
        #sy = math.sin(math.radians(rot.yaw) * 0.5)
        #cp = math.cos(math.radians(rot.pitch) * 0.5)
        #sp = math.sin(math.radians(rot.pitch) * 0.5)
        #cr = math.cos(math.radians(rot.roll) * 0.5)
        #sr = math.sin(math.radians(rot.roll) * 0.5)

        #msg.orientation.w = cr * cp * cy + sr * sp * sy
        #msg.orientation.x = sr * cp * cy - cr * sp * sy
        #msg.orientation.y = cr * sp * cy + sr * cp * sy
        #msg.orientation.z = cr * cp * sy - sr * sp * cy


        self.publisher_.publish(msg)
        self.get_logger().info(
            f"[{self.get_namespace()}] Point: x={loc.x:.2f}, y={loc.y:.2f}, z={loc.z:.2f}")

def main():
    if len(sys.argv) != 2:
        print("사용법: python3 enuconverter.py <트럭 개수 (1~3)>")
        return

    num_trucks = int(sys.argv[1])
    if not (1 <= num_trucks <= 3):
        print("트럭 개수는 1~3 사이여야 합니다.")
        return

    rclpy.init()
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    vehicles = world.get_actors().filter('vehicle.*')
    if len(vehicles) < num_trucks:
        print(f"❌ 차량 부족. 현재 {len(vehicles)}대 존재.")
        return

    executor = MultiThreadedExecutor()
    nodes = []

    for i in range(num_trucks):
        namespace = f"truck{i}"
        node = TruckPointPublisher(vehicles[i], namespace)
        nodes.append(node)
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

