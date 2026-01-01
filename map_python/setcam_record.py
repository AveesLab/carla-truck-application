import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import carla
import math
import sys
import os

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi  # deg


class CameraFollower(Node):
    def __init__(self, argv_input):
        super().__init__('odom_camera_follower')

        # =========================
        # CARLA 연결
        # =========================
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.spectator = self.world.get_spectator()

        # =========================
        # 녹화 설정
        # =========================
        self.enable_record = True  # 필요하면 false로 꺼도 됨
        self.out_dir = f"record_truck{argv_input}"
        os.makedirs(self.out_dir, exist_ok=True)

        # 프레임 번호 저장용(ROS callback 기준 증가)
        self.local_frame = 0

        # =========================
        # RGB 카메라 센서 생성
        # =========================
        bp_lib = self.world.get_blueprint_library()
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", "1920")
        cam_bp.set_attribute("image_size_y", "1080")
        cam_bp.set_attribute("fov", "90")

        # 초기 위치는 임시
        init_tf = carla.Transform(
            carla.Location(x=0.0, y=0.0, z=50.0),
            carla.Rotation(pitch=-25, yaw=0.0)
        )

        self.camera = self.world.spawn_actor(cam_bp, init_tf)

        # 카메라 콜백
        self.camera.listen(self._on_image)

        # =========================
        # Odometry 구독
        # =========================
        ns = '/truck' + argv_input + '/ENU'
        self.create_subscription(Odometry, ns, self.odom_callback, 10)

        self.get_logger().info(f"Recording dir: {self.out_dir}")

    def _on_image(self, image: carla.Image):
        if not self.enable_record:
            return

        # image.frame을 쓰면 CARLA 프레임 기반 파일명
        # local_frame을 쓰면 ROS 콜백 기반 파일명
        # 둘 중 하나 선택하면 됨
        fname = os.path.join(self.out_dir, f"{image.frame:06d}.png")
        image.save_to_disk(fname)

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        x, y, z = pos.x, pos.y, pos.z
        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

        cam_tf = carla.Transform(
            carla.Location(
                x=x - 75 * math.cos(yaw * math.pi / 180),
                y=y - 75 * math.sin(yaw * math.pi / 180),
                z=z + 40
            ),
            carla.Rotation(pitch=-25, yaw=yaw)
        )

        # Spectator 이동
        self.spectator.set_transform(cam_tf)

        # ✅ 센서 카메라도 동일 위치로 이동
        if self.camera is not None:
            self.camera.set_transform(cam_tf)

        self.local_frame += 1

        # 로그 너무 과하면 성능 떨어질 수 있어서 필요시 주기 줄이기
        # self.get_logger().info(f"Camera moved to x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

    def destroy(self):
        try:
            if self.camera is not None:
                self.camera.stop()
                self.camera.destroy()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CameraFollower(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

