import carla
import csv
import os
import math

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(20.0)
    world = client.load_world("mid_large_cycle")
    carla_map = world.get_map()

    # 시작점 지정
    truck_spawn = carla.Location(x=18980, y=19509, z=2.0)
    start_wp = carla_map.get_waypoint(truck_spawn, project_to_road=True, lane_type=carla.LaneType.Driving)

    # 오른쪽 차선으로 이동
    right_wp = start_wp.get_left_lane()
    if right_wp is None or right_wp.lane_type != carla.LaneType.Driving:
        print("[ERROR] 오른쪽 주행 차선을 찾을 수 없습니다.")
        return

    start_wp = right_wp
    start_loc = start_wp.transform.location
    start_yaw = start_wp.transform.rotation.yaw

    step = 0.5
    tol_dist = 0.05
    tol_yaw = 1.0
    min_pts = 20

    waypoints = []
    wp = start_wp

    while True:
        loc = wp.transform.location
        yaw = wp.transform.rotation.yaw
        waypoints.append([loc.x, loc.y, loc.z])

        if len(waypoints) > min_pts:
            d = loc.distance(start_loc)
            dy = abs(((yaw - start_yaw + 180) % 360) - 180)
            if d < tol_dist and dy < tol_yaw:
                print(f"[INFO] 루프 감지됨 → 종료")
                waypoints.append([start_loc.x, start_loc.y, start_loc.z])
                break

        next_wps = wp.next(step)
        if not next_wps:
            print("[WARNING] 더 이상 다음 웨이포인트가 없습니다.")
            break
        wp = next_wps[0]

    # CSV 저장
    save_dir = os.path.expanduser("~/ros2_ws/src/data")
    os.makedirs(save_dir, exist_ok=True)
    csv_path = os.path.join(save_dir, "1.csv")
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "z"])
        for p in waypoints:
            writer.writerow(p)
    print(f"[SUCCESS] 오른쪽 차선 기준 웨이포인트 {len(waypoints)}개 저장됨 → {csv_path}")

if __name__ == '__main__':
    main()

