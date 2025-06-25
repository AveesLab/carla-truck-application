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

            if 0 <= i <= 1800+300 :
            # 1 before curve
                self.world.debug.draw_string(self.waypoints[i].location, 'O', draw_shadow=False,
                color=carla.Color(r=0,  g=255, b=0), life_time=LT,
                persistent_lines=True)

            elif 78100-300 <= i <= 79600+300 :
            # 2 curve
                self.world.debug.draw_string(self.waypoints[i].location, 'O', draw_shadow=False,
                color=carla.Color(r=0,  g=255, b=0), life_time=LT,
                persistent_lines=True)


            elif 155900-300 <= i <= 157400+300 :
            # 3 curve
                self.world.debug.draw_string(self.waypoints[i].location, 'O', draw_shadow=False,
                color=carla.Color(r=0,  g=255, b=0), life_time=LT,
                persistent_lines=True)

            elif 233250-300 <= i <= 235750+300 :
            # 4 curve
                self.world.debug.draw_string(self.waypoints[i].location, 'O', draw_shadow=False,
                color=carla.Color(r=0,  g=255, b=0), life_time=LT,
                persistent_lines=True)

            # else:
            #     self.world.debug.draw_string(wp1, 'O', draw_shadow=False,
            #     color=carla.Color(r=127, g=127, b=0), life_time=LT,
            #     persistent_lines=True)

def main(args=None):
    rclpy.init(args=args)
    node = CarlaWaypointVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

