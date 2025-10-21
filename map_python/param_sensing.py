# -*- coding: utf-8 -*-
import sys
import time

import carla
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor  # 또는 SingleThreadedExecutor

from geometry_msgs.msg import Point
from std_msgs.msg import Float32

DEFAULT_DRAG = 0.6


class TruckPointPublisher(Node):
    def __init__(self, vehicle_actor, namespace='truck0'):
        # ROS2 네이밍 컨벤션에 따라 노드 이름은 소문자 스네이크 케이스 권장
        super().__init__('truck_param_publisher', namespace=namespace) #changed
        self.publisher_ = self.create_publisher(Point, 'server/enu', 10)
        self.vehicle = vehicle_actor
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_point_callback)

        # drag 토픽 수신 전 기본값 0.6을 보유. 수신 시 덮어씀.
        self.target_drag = float(DEFAULT_DRAG)
        self.last_applied_drag = None
        self.sub_drag = self.create_subscription(
            Float32, 'drag', self.on_drag, 10  # 네임스페이스 적용 → /truckX/drag
        )

        self.get_logger().info(
            f"Publisher for namespace '{self.get_namespace()}' initialized for CARLA vehicle ID {self.vehicle.id}."
        )

    def on_drag(self, msg: Float32):
        self.target_drag = float(msg.data)

    def publish_point_callback(self):
        # 차량 액터가 유효하고 살아있는지 확인
        if not self.vehicle or not self.vehicle.is_alive:
            self.get_logger().warn(
                f"[{self.get_namespace()}] Vehicle (ID: {self.vehicle.id if self.vehicle else 'N/A'}) is not valid or not alive. Skipping publish."
            )
            return

        try:
            # 위치 퍼블리시
            transform = self.vehicle.get_transform()
            loc = transform.location
            msg = Point()
            msg.x = loc.x
            msg.y = loc.y
            msg.z = loc.z
            self.publisher_.publish(msg)

            # drag 적용: 변경 시에만 ApplyPhysicsControl 수행
            want = float(self.target_drag)
            pc = self.vehicle.get_physics_control()
            cur = getattr(pc, 'drag_coefficient', None)
            if (cur is None) or (abs(cur - want) > 1e-4):
                pc.drag_coefficient = want
                self.vehicle.apply_physics_control(pc)
                self.last_applied_drag = want

        except Exception as e:
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

    if not (1 <= num_trucks_to_configure <= 3):
        print("Error: Number of trucks must be between 1 and 3 for this script configuration.")
        return

    rclpy.init()
    carla_client = None
    try:
        # CARLA 서버에 연결
        carla_client = carla.Client('localhost', 2000)
        carla_client.set_timeout(10.0)  # seconds
        world = carla_client.get_world()

    except Exception as e:
        print(f"Critical Error: Could not connect to CARLA simulator at localhost:2000. Details: {e}")
        if rclpy.ok():
            rclpy.shutdown()
        return

    # Executor 선택
    executor = MultiThreadedExecutor()
    created_nodes = []

    # CARLA 월드 준비 대기
    print("Waiting a moment for CARLA world and actors to settle...")
    time.sleep(2.0)
    all_vehicle_actors_in_world = world.get_actors().filter('vehicle.*')
    print(f"Found {len(all_vehicle_actors_in_world)} vehicle actors in the CARLA world.")

    # 디버깅: 모든 차량 액터의 role_name과 ID 출력
    for actor_debug in all_vehicle_actors_in_world:
        role_name_debug = actor_debug.attributes.get('role_name', 'N/A')
        print(f"  - Found Actor ID: {actor_debug.id}, Type: {actor_debug.type_id}, Role Name: {role_name_debug}")

    # 각 트럭에 대한 퍼블리셔 노드 생성
    for i in range(num_trucks_to_configure):
        node_namespace = f"truck{i}"
        target_vehicle_role_name = f"truck{i}"
        specific_vehicle_actor = None

        # 해당 role_name 차량 탐색
        for carla_actor in all_vehicle_actors_in_world:
            actor_role_name = carla_actor.attributes.get('role_name')
            if actor_role_name == target_vehicle_role_name:
                specific_vehicle_actor = carla_actor
                break

        if specific_vehicle_actor:
            print(f"Found vehicle for ROS namespace '{node_namespace}' with CARLA role_name '{target_vehicle_role_name}' (CARLA Actor ID: {specific_vehicle_actor.id}).")
            print(f"  Vehicle Type: {specific_vehicle_actor.type_id}")

            # 노드 생성 및 등록
            node_instance = TruckPointPublisher(specific_vehicle_actor, node_namespace)
            created_nodes.append(node_instance)
            executor.add_node(node_instance)
        else:
            print(f":x: CRITICAL ERROR: Could not find any vehicle with role_name '{target_vehicle_role_name}' for ROS namespace '{node_namespace}' in the CARLA world.")
            print("   Please ensure a vehicle with this role_name is correctly spawned in CARLA before running this script.")
            print("   Skipping creation of publisher for {node_namespace}.")
            print("Exiting due to missing critical vehicle actor.")

            # 생성된 노드 정리
            for node_to_clean in created_nodes:
                try:
                    if hasattr(executor, "_is_shutdown") and not executor._is_shutdown:
                        executor.remove_node(node_to_clean)
                except Exception:
                    pass
                try:
                    if hasattr(node_to_clean, "destroy_node"):
                        node_to_clean.destroy_node()
                except Exception:
                    pass

            if rclpy.ok():
                rclpy.shutdown()
            return

    # 생성된 노드가 하나도 없다면 종료
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
        print(f"\nAn exception occurred during executor spin: {e}")
    finally:
        print("Cleaning up ROS2 nodes...")
        for node in created_nodes:
            # 타이머 취소
            try:
                if hasattr(node, "timer") and node.timer:
                    # Foxy+: is_canceled(), Humble+: is_ready()/is_active() 차이 고려
                    try:
                        if hasattr(node.timer, "cancel"):
                            node.timer.cancel()
                    except Exception:
                        pass
            except Exception:
                pass

            # Executor에서 제거
            try:
                if hasattr(executor, "_is_shutdown") and not executor._is_shutdown:
                    executor.remove_node(node)
            except Exception:
                pass

            # 노드 파괴
            try:
                node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()
        print("ROS2 shutdown complete. Exiting script.")


if __name__ == '__main__':
    main()

