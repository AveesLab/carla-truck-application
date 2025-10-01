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


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def speed_of(v):  # carla.Vector3D -> float (m/s)
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


class TruckCSVLogger(Node):
    """
    CARLA vehicle actor로부터 10Hz로
    timestamp(s), x, y, velocity(m/s), pitch(deg), formation_change 를 CSV로 기록
    """
    def __init__(self, vehicle_actor: carla.Actor, truck_index_zero_based: int, save_dir: str, namespace: str):
        # ROS2 네임스페이스: truck0/truck1/...
        super().__init__('truck_csv_logger', namespace=namespace)

        self.vehicle = vehicle_actor
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self._on_timer)

        # Formation change 상태 추적
        self.formation_change_active = False
        self.csv_logging_started = False  # CSV 기록 시작 플래그
        self.formation_change_value = 0  # 수신받은 formation_change 값
        # 각 트럭의 네임스페이스별 토픽 구독
        formation_topic = f'/{namespace}/formation_change'
        self.formation_change_subscriber = self.create_subscription(
            Int32,
            formation_topic,
            self._on_formation_change,
            10
        )

        # 파일명: 0_truck.csv, 1_truck.csv, 2_truck.csv (truck0 -> 0, truck1 -> 1, truck2 -> 2)
        ensure_dir(save_dir)
        self.csv_path = os.path.join(save_dir, f"{truck_index_zero_based}_truck.csv")

        # CSV 오픈 및 헤더 작성
        self._file = open(self.csv_path, mode='w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._file)
        self._writer.writerow(['timestamp', 'x', 'y', 'velocity', 'pitch', 'formation_change'])
        self._file.flush()

        self.get_logger().info(
            f"[{self.get_namespace()}] CSV file prepared -> {self.csv_path} (vehicle ID {self.vehicle.id})"
        )
        self.get_logger().info(
            f"[{self.get_namespace()}] Waiting for {formation_topic} signal to start logging..."
        )

    def _now_sec(self) -> float:
        # ROS 시계(시뮬레이션 시간 포함) 사용. 필요 시 실제 시간은 time.time()
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_formation_change(self, msg: Int32):
        """Formation change 메시지를 받으면 플래그 설정 및 CSV 기록 시작"""
        # 수신받은 값 저장
        self.formation_change_value = msg.data
        
        # 첫 번째 메시지를 받으면 CSV 기록 시작
        if not self.csv_logging_started:
            self.csv_logging_started = True
            self.get_logger().info(f"[{self.get_namespace()}] CSV logging started after receiving formation_change signal!")
        


    def _on_timer(self):
        # CSV 기록이 시작되지 않았으면 리턴
        if not self.csv_logging_started:
            return
            
        if not self.vehicle or not self.vehicle.is_alive:
            self.get_logger().warn(
                f"[{self.get_namespace()}] Vehicle (ID: {self.vehicle.id if self.vehicle else 'N/A'}) not alive. Skip."
            )
            return

        try:
            tf = self.vehicle.get_transform()
            loc = tf.location
            rot = tf.rotation
            vel = self.vehicle.get_velocity()

            ts = f"{self._now_sec():.9f}"          # 초 단위, 나노초 정밀도 문자열
            x = loc.x
            y = loc.y
            spd = speed_of(vel)                    # m/s
            pitch = rot.pitch                      # deg
            formation_flag = self.formation_change_value  # 수신받은 formation_change 값

            self._writer.writerow([ts, f"{x:.6f}", f"{y:.6f}", f"{spd:.6f}", f"{pitch:.6f}", formation_flag])
            # 디스크 안전성 위해 주기적으로 flush (10Hz라 부담 적음)
            self._file.flush()

        except Exception as e:
            self.get_logger().error(f"[{self.get_namespace()}] Timer error: {e}")

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

