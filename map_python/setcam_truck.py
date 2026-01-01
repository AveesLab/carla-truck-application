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
    def __init__(self, argv_input):
        super().__init__('odom_camera_follower')

        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.spectator = self.world.get_spectator()

        ns = '/truck' + argv_input + '/ENU'
        self.create_subscription(Odometry, ns, self.odom_callback, 10)

        # ===== smoothing state =====
        self.cam_x = None
        self.cam_y = None
        self.cam_z = None
        self.cam_yaw = None

        self.alpha_pos = 0.15   # 0~1 (작을수록 더 부드러움)
        self.alpha_yaw = 0.2

    def _lerp(self, a, b, alpha):
        return a + alpha * (b - a)

    def _lerp_angle_deg(self, a, b, alpha):
        # yaw가 359 -> 1 같은 상황에서 반대방향으로 크게 도는 문제 방지
        diff = (b - a + 180) % 360 - 180
        return a + alpha * diff

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        x, y, z = pos.x, pos.y, pos.z
        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

        # ===== 원하는 구도 파라미터 =====
        dist = 75
        height = 40
        pitch = -25

        target_x = x - dist * math.cos(math.radians(yaw))
        target_y = y - dist * math.sin(math.radians(yaw))
        target_z = z + height
        target_yaw = yaw

        # ===== init =====
        if self.cam_x is None:
            self.cam_x, self.cam_y, self.cam_z, self.cam_yaw = target_x, target_y, target_z, target_yaw

        # ===== smoothing =====
        self.cam_x = self._lerp(self.cam_x, target_x, self.alpha_pos)
        self.cam_y = self._lerp(self.cam_y, target_y, self.alpha_pos)
        self.cam_z = self._lerp(self.cam_z, target_z, self.alpha_pos)
        self.cam_yaw = self._lerp_angle_deg(self.cam_yaw, target_yaw, self.alpha_yaw)

        cam_tf = carla.Transform(
            carla.Location(x=self.cam_x, y=self.cam_y, z=self.cam_z),
            carla.Rotation(pitch=pitch, yaw=self.cam_yaw)
        )
        self.spectator.set_transform(cam_tf)


def main(args=None):
    rclpy.init(args=args)
    node = CameraFollower(sys.argv[1])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

