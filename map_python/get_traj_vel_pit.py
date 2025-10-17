#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import math
import csv

import carla
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int32
from rosgraph_msgs.msg import Clock


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def speed_of(v):  # carla.Vector3D -> float (m/s)
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


class TruckCSVLogger(Node):
    def __init__(self, vehicle_actor: carla.Actor, truck_index_zero_based: int, save_dir: str, namespace: str):
        super().__init__('truck_csv_logger', namespace=namespace)

        self.vehicle = vehicle_actor
        self.csv_logging_started = False
        self.formation_change_value = 0

        # Formation change 수신 (기존 그대로)
        formation_topic = f'/{namespace}/formation_change'
        self.formation_change_subscriber = self.create_subscription(
            Int32, formation_topic, self._on_formation_change, 10
        )

        # ✅ /clock 구독 추가
        self.clock_sub = self.create_subscription(Clock, '/clock', self._on_clock, 10)

        ensure_dir(save_dir)
        self.csv_path = os.path.join(save_dir, f"{truck_index_zero_based}_truck.csv")
        self._file = open(self.csv_path, mode='w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._file)
        self._writer.writerow(['timestamp', 'x', 'y', 'velocity', 'pitch', 'formation_change'])
        self._file.flush()

        self.get_logger().info(f"[{self.get_namespace()}] CSV ready -> {self.csv_path}")

    def _on_formation_change(self, msg: Int32):
        self.formation_change_value = msg.data
        if not self.csv_logging_started:
            self.csv_logging_started = True
            self.get_logger().info(f"[{self.get_namespace()}] CSV logging started!")

    # ✅ /clock 콜백: 시뮬레이션 tick마다 호출
    def _on_clock(self, msg: Clock):
        if not self.csv_logging_started:
            return

        sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        try:
            tf = self.vehicle.get_transform()
            loc = tf.location
            rot = tf.rotation
            vel = self.vehicle.get_velocity()

            x, y = loc.x, loc.y
            spd = speed_of(vel)
            pitch = rot.pitch
            formation_flag = self.formation_change_value

            self._writer.writerow([
                f"{sim_time:.6f}", f"{x:.6f}", f"{y:.6f}",
                f"{spd:.6f}", f"{pitch:.6f}", formation_flag
            ])
            self._file.flush()

        except Exception as e:
            self.get_logger().error(f"[{self.get_namespace()}] Log error: {e}")

    def destroy_node(self):
        try:
            if self.timer:
                self.timer.cancel()
        except Exception:
            pass
        try:
            if getattr(self, "_file", None):
                self._file.flush()
                self._file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    # 인자: 트럭 대수 (1~3)
    if len(sys.argv) != 2:
        script_name = sys.argv[0] if sys.argv else "truck_csv_logger.py"
        print(f"Usage: python3 {script_name} <number_of_trucks (1-3)>")
        return

    try:
        num_trucks = int(sys.argv[1])
    except ValueError:
        print("Error: Number of trucks must be an integer.")
        return

    if not (1 <= num_trucks <= 3):
        print("Error: Number of trucks must be between 1 and 3.")
        return

    # 저장 디렉터리 (요청 경로)
    save_dir = os.path.expanduser('~/ros2_ws/src/test_truck')
    ensure_dir(save_dir)

    rclpy.init()
    client = None
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
    except Exception as e:
        print(f"Critical Error: Could not connect to CARLA at localhost:2000. Details: {e}")
        if rclpy.ok():
            rclpy.shutdown()
        return

    # 액터 안정화 대기 (환경에 맞게 조절)
    print("Waiting for CARLA actors to settle...")
    time.sleep(2.0)

    vehicles = world.get_actors().filter('vehicle.*')
    print(f"Found {len(vehicles)} vehicle actors in CARLA world.")
    for a in vehicles:
        print(f"  - Actor ID: {a.id}, Type: {a.type_id}, role_name: {a.attributes.get('role_name', 'N/A')}")

    # 요청한 대수만큼 truck{i} 를 찾아 로거 노드 구성 (동시성 보장을 위해 SingleThreadedExecutor 사용)
    executor = SingleThreadedExecutor()
    nodes = []

    try:
        for i in range(num_trucks):
            role = f"truck{i}"
            target = None
            for a in vehicles:
                if a.attributes.get('role_name') == role:
                    target = a
                    break
            if target is None:
                print(f"❌ CRITICAL: CARLA vehicle with role_name '{role}' not found. Exiting.")
                for n in nodes:
                    try:
                        executor.remove_node(n)
                    except Exception:
                        pass
                    n.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                return

            ns = role  # 네임스페이스 동일하게
            node = TruckCSVLogger(target, i, save_dir, ns)
            nodes.append(node)
            executor.add_node(node)

        print(f"\n✅ Started {len(nodes)} CSV logger node(s) at 10 Hz.")
        print("Press Ctrl+C to stop.")
        try:
            executor.spin()
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt: stopping...")
        finally:
            for n in nodes:
                try:
                    executor.remove_node(n)
                except Exception:
                    pass
                n.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            print("ROS2 shutdown complete. CSV files saved at:", save_dir)

    except Exception as e:
        print(f"Runtime Error: {e}")
        for n in nodes:
            try:
                executor.remove_node(n)
            except Exception:
                pass
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

