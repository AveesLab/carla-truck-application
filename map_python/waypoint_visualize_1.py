import rclpy
from rclpy.node import Node
from ros2_msg.msg import GNSS
import carla
import csv

class CarlaWaypointVisualizer(Node):
    def __init__(self):
        super().__init__('carla_waypoint_visualizer')

        # 경로 파일 경로
        self.csv_path = '/home/avees/ros2_ws/src/truck_controller/data/1.csv'
        
        # CARLA 클라이언트 연결
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # 경로 저장
        self.waypoints = []
        self.load_waypoints(self.csv_path)

        # **한 번만** 그리기
        self.visualize_waypoints()

        # GNSS 구독(시각화에는 사용 안 함)
        self.create_subscription(GNSS, '/truck0/gnss', self.gnss_callback, 10)

    def gnss_callback(self, msg: GNSS):
        # GNSS 메시지를 수신하면 로그만 찍고, 그리기는 하지 않습니다.
        self.get_logger().info(
            f'GNSS Data: Latitude: {msg.latitude:.6f}, Longitude: {msg.longitude:.6f}')

    def load_waypoints(self, csv_path):
        with open(csv_path, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # 헤더 스킵
            for row in reader:
                x = float(row[0]); y = float(row[1]); z = float(row[2])
                self.waypoints.append(carla.Transform(carla.Location(x, y, z)))

    def visualize_waypoints(self):


        LT = 1e6
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i].location
            wp2 = self.waypoints[i + 1].location

            # # 1800~1850 구간만 초록색, 나머지는 빨간색
            # if 1400 <= i <= 2000:
            #     color = carla.Color(255, 127, 0)  # orange
            # else:
            #     color = carla.Color(255, 127, 0)  # orange

            # self.world.debug.draw_line(
            #     wp1, wp2,
            #     thickness=0.1,
            #     color=color,
            #     life_time=LT
            # )
            self.world.debug.draw_string(wp1, 'O', draw_shadow=False,
            color=carla.Color(r=255, g=127, b=0), life_time=LT,
            persistent_lines=True)

def main(args=None):
    rclpy.init(args=args)
    node = CarlaWaypointVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

