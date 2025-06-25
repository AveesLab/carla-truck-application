import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import carla
import math
import sys


def quaternion_to_yaw(x, y, z, w):
    # ROS -> Euler 변환 (yaw만 추출)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi  # deg

class CameraFollower(Node):
    def __init__(self,argv_input):
        super().__init__('odom_camera_follower')

        # CARLA 클라이언트 연결
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.spectator = self.world.get_spectator()

        ns =  '/truck' + argv_input + '/ENU' 

        # Odometry 구독
        self.create_subscription(Odometry, ns, self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        x, y, z = pos.x, pos.y, pos.z
        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
        

        # 카메라를 차량 위에서 헤딩 방향으로 배치
        cam_tf = carla.Transform(
            carla.Location(x=x-75*math.cos(yaw*math.pi/180), y=y-75*math.sin(yaw*math.pi/180), z=z + 40),  # 위에서
            carla.Rotation(pitch=-25, yaw=yaw)  # 트럭 방향
        )
        self.spectator.set_transform(cam_tf)
        self.get_logger().info(f"Camera moved to x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraFollower(sys.argv[1])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

