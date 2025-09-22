#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import time
import carla

TOWN_NAME   = "Town04_Opt"
OUT_DIR     = os.path.expanduser("~/ros2_ws/src/town04_data")
OUT_CSV     = os.path.join(OUT_DIR, "town_waypoint_0.csv")

# 시작 기준점 (요청한 값)
START_LOC   = carla.Location(x=-270.990, y=27.0, z=2.0)

# 경로 추출 파라미터
STEP_METERS = 0.5      # waypoint 간격 (m)
TOL_DIST    = 0.05     # 시작점으로 되돌아왔는지 확인하는 거리 허용오차 (m)
TOL_YAW     = 1.0      # 시작 yaw와의 차이 허용오차 (deg)
MIN_POINTS  = 20       # 이 수 이상 포인트가 쌓인 후에만 루프 체크

def norm_deg_pm180(a: float) -> float:
    """각도를 [-180, 180) 범위로 정규화"""
    return ((a + 180.0) % 360.0) - 180.0

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    # 맵 로드 및 안정화
    world = client.load_world(TOWN_NAME)
    time.sleep(0.5)
    carla_map = world.get_map()

    # 시작 웨이포인트 (도로로 스냅)
    start_wp  = carla_map.get_waypoint(
        START_LOC,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    start_tf  = start_wp.transform
    start_loc = start_tf.location
    start_yaw = start_tf.rotation.yaw

    waypoints = []
    wp = start_wp

    print(f"[INFO] start at x={start_loc.x:.3f}, y={start_loc.y:.3f}, z={start_loc.z:.3f}, yaw={start_yaw:.2f}")

    while True:
        loc = wp.transform.location
        rot = wp.transform.rotation
        pitch_norm = norm_deg_pm180(rot.pitch)

        waypoints.append((loc.x, loc.y, loc.z, pitch_norm))

        # 닫힌 루프 감지
        if len(waypoints) > MIN_POINTS:
            d = loc.distance(start_loc)
            dyaw = abs(((rot.yaw - start_yaw + 180.0) % 360.0) - 180.0)
            if d < TOL_DIST and dyaw < TOL_YAW:
                print(f"[INFO] loop closed: dist={d:.3f} m, Δyaw={dyaw:.2f}° → stop")
                # 시작 위치로 한 점 더 찍어 경로 닫기 (pitch는 시작 pitch 기준)
                waypoints.append((start_loc.x, start_loc.y, start_loc.z, norm_deg_pm180(start_tf.rotation.pitch)))
                break

        nxt = wp.next(STEP_METERS)
        if not nxt:
            print("[WARN] no further waypoint found → stop")
            break

        wp = nxt[0]

    # 저장
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(OUT_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z", "pitch_deg"])
        w.writerows(waypoints)

    print(f"[SUCCESS] saved {len(waypoints)} points → {OUT_CSV}")

if __name__ == "__main__":
    main()
