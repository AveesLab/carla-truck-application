'''
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

'''
import carla
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor # 또는 SingleThreadedExecutor
from geometry_msgs.msg import Point

import sys
import time # CARLA 액터 로드 대기 및 디버깅용
class TruckPointPublisher(Node):
    def __init__(self, vehicle_actor, namespace='truck0'): # 파라미터 이름 명확화
        # ROS2 네이밍 컨벤션에 따라 노드 이름은 소문자 스네이크 케이스 권장
        super().__init__('truck_point_publisher', namespace=namespace)
        self.publisher_ = self.create_publisher(Point, 'server/enu', 10)
        self.vehicle = vehicle_actor # 전달받은 carla.Actor 객체
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_point_callback)
        self.get_logger().info(
            f"Publisher for namespace '{self.get_namespace()}' initialized for CARLA vehicle ID {self.vehicle.id}."
        )
    def publish_point_callback(self):
        # 차량 액터가 유효하고 살아있는지 확인 (CARLA에서 액터가 파괴되었을 수 있음)
        if not self.vehicle or not self.vehicle.is_alive:
            self.get_logger().warn(
                f"[{self.get_namespace()}] Vehicle (ID: {self.vehicle.id if self.vehicle else 'N/A'}) is not valid or not alive. Skipping publish."
            )
            # 이런 경우 타이머를 멈추거나 노드를 정리하는 로직 추가 고려 가능
            return
        try:
            transform = self.vehicle.get_transform()
            loc = transform.location
            msg = Point()
            msg.x = loc.x
            msg.y = loc.y
            msg.z = loc.z
            self.publisher_.publish(msg)
            # 로그 레벨을 DEBUG로 낮추거나, 매우 짧은 주기로 로그를 남기지 않도록 주의
            # self.get_logger().debug( # DEBUG 레벨로 변경 또는 필요시 INFO 사용
            #     f"[{self.get_namespace()}] Publishing Point: x={loc.x:.2f}, y={loc.y:.2f}, z={loc.z:.2f}"
            # )
        except Exception as e:
            # CARLA API 호출 중 예외 발생 가능 (예: 시뮬레이터 연결 끊김)
            self.get_logger().error(f"[{self.get_namespace()}] Error in publish_point_callback: {e}")
def main():
    # 스크립트 인자 파싱 (트럭 개수)
    if len(sys.argv) != 2:
        script_name = sys.argv[0] if sys.argv else "your_script_name.py"
        print(f"Usage: python3 {script_name} <number_of_trucks (1-3)>")
        return
    try:
        num_trucks_to_configure = int(sys.argv[1])
    except ValueError:
        print("Error: Number of trucks must be an integer.")
        return
    # 현재 스크립트는 최대 3대까지만 명시적으로 role_name ('truck0', 'truck1', 'truck2')을 가정
    if not (1 <= num_trucks_to_configure <= 3):
        print("Error: Number of trucks must be between 1 and 3 for this script configuration.")
        return
    rclpy.init()
    carla_client = None
    try:
        # CARLA 서버에 연결
        carla_client = carla.Client('localhost', 2000)
        carla_client.set_timeout(10.0) # seconds
        world = carla_client.get_world()

    except Exception as e:
        print(f"Critical Error: Could not connect to CARLA simulator at localhos t:2000. Details: {e}")
        if rclpy.ok(): # rclpy가 초기화되었다면 종료
            rclpy.shutdown()
        return
    # MultiThreadedExecutor 또는 SingleThreadedExecutor 선택 가능
    # 현재 각 노드는 독립적인 타이머 콜백만 가지므로 SingleThreadedExecutor도 충분할 수 있음
    # 문제가 지속되면 executor 타입을 변경하며 테스트하는 것도 방법
    executor = MultiThreadedExecutor()
    created_nodes = [] # 생성된 노드 객체들을 저장할 리스트
    # CARLA 월드에서 모든 차량 액터 가져오기
    # 이 스크립트 실행 시점에 원하는 차량들이 CARLA 월드에 모두 스폰되어 있어야 함
    # CARLA 월드가 완전히 로드되고 액터들이 스폰될 시간을 약간 줄 수 있습니다.
    print("Waiting a moment for CARLA world and actors to settle...")
    time.sleep(2.0) # 2초 대기 (환경에 따라 조절)
    all_vehicle_actors_in_world = world.get_actors().filter('vehicle.*') # 'vehicle.*' 필터는 모든 종류의 차량을 가져옴
    print(f"Found {len(all_vehicle_actors_in_world)} vehicle actors in the CARLA world.")
    # 디버깅: 찾은 모든 차량 액터의 role_name과 ID 출력
    for actor_debug in all_vehicle_actors_in_world:
        role_name_debug = actor_debug.attributes.get('role_name', 'N/A')
        print(f"  - Found Actor ID: {actor_debug.id}, Type: {actor_debug.type_id}, Role Name: {role_name_debug}")
    # 각 트럭에 대한 퍼블리셔 노드 생성
    for i in range(num_trucks_to_configure):
        node_namespace = f"truck{i}"
        # 이 노드가 담당해야 할 차량의 role_name (CARLA에서 스폰 시 설정된 이름과 일치해야 함)
        target_vehicle_role_name = f"truck{i}"
        specific_vehicle_actor = None
        # CARLA 월드의 모든 차량 액터 중에서 해당 role_name을 가진 액터 검색
        for carla_actor in all_vehicle_actors_in_world:
            # 액터의 'attributes' 딕셔너리에서 'role_name' 키의 값을 가져옴
            # 해당 키가 없으면 기본값으로 'N/A' 또는 다른 값 사용 가능
            actor_role_name = carla_actor.attributes.get('role_name')
            if actor_role_name == target_vehicle_role_name:
                specific_vehicle_actor = carla_actor
                break # 해당 role_name을 가진 차량을 찾았으므로 내부 루프 종료
        if specific_vehicle_actor:
            print(f"Found vehicle for ROS namespace '{node_namespace}' with CARLA role_name '{target_vehicle_role_name}' (CARLA Actor ID: {specific_vehicle_actor.id}).")
            print(f"  Vehicle Type: {specific_vehicle_actor.type_id}")
            # 해당 차량 객체로 TruckPointPublisher 노드 생성
            node_instance = TruckPointPublisher(specific_vehicle_actor, node_namespace)
            created_nodes.append(node_instance)
            executor.add_node(node_instance)
        else:
            # 매우 중요한 오류: CARLA 월드에서 해당 role_name의 차량을 찾지 못함
            print(f":x: CRITICAL ERROR: Could not find any vehicle with role_name '{target_vehicle_role_name}' for ROS namespace '{node_namespace}' in the CARLA world.")
            print(f"   Please ensure a vehicle with this role_name is correctly spawned in CARLA before running this script.")
            print(f"   Skipping creation of publisher for {node_namespace}.")
            # 여기서 스크립트를 종료하거나, 계속 진행하되 해당 노드는 생성하지 않을 수 있음
            # 군집 주행에서는 모든 차량이 중요하므로, 하나라도 없으면 문제가 될 수 있어 종료 권장
            print("Exiting due to missing critical vehicle actor.")
            for node_to_clean in created_nodes: # 이미 생성된 노드가 있다면 정리
                if executor and not executor._is_shutdown : executor.remove_node(node_to_clean)
                if not node_to_clean._destroyed : node_to_clean.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            return # 스크립트 실행 중단
    # 생성된 노드가 하나도 없다면 스핀할 필요 없음
    if not created_nodes:
        print("No publisher nodes were successfully created. Exiting.")
        if rclpy.ok():
            rclpy.shutdown()
        return
    print(f"\nSuccessfully created and added {len(created_nodes)} publisher nodes to the executor.")
    print("Starting executor spin (Press Ctrl+C to exit)...")
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Shutting down publisher nodes and rclpy...")
    except Exception as e:
        # spin 중 다른 예외 발생 시
        print(f"\nAn exception occurred during executor spin: {e}")
    finally:
        # 프로그램 종료 시 생성된 노드들을 정리
        print("Cleaning up ROS2 nodes...")
        for node in created_nodes:
            # 타이머가 있다면 명시적으로 취소
            if node.timer and node.timer.is_active: # is_active (ROS2 Foxy 이후) 또는 is_ready (이전)
                node.timer.cancel()
            # Executor에서 노드 제거 (실행 중인 executor에서 제거해야 함)
            if executor and not executor._is_shutdown: # executor가 종료되지 않았다면
                 try:
                     executor.remove_node(node)
                 except Exception as ex_remove:
                     print(f"Error removing node from executor: {ex_remove}")
            # 노드 파괴
            if not node._destroyed: # 이미 파괴되지 않았다면
                 node.destroy_node()
        # rclpy 종료
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS2 shutdown complete. Exiting script.")
if __name__ == '__main__':
    main()