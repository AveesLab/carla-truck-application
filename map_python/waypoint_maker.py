import carla
import csv
import os
import math

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.load_world("Town06")
    carla_map = world.get_map()

    # 시작점 지정
    truck_spawn = carla.Location(x=33.215,  y=-16.058,z=  0.0)
    start_wp = carla_map.get_waypoint(
        truck_spawn,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    start_tf  = start_wp.transform
    start_loc = start_tf.location
    start_yaw = start_tf.rotation.yaw

    # 파라미터
    step        = 0.5    # m
    tol_dist    = 0.05   # 5cm 이내
    tol_yaw     = 1.0    # 1도 이내
    min_pts     = 20     # 이만큼 포인트 수집 후에만 루프 체크

    waypoints = []
    wp = start_wp

    while True:
        loc  = wp.transform.location
        yaw  = wp.transform.rotation.yaw
        waypoints.append([loc.x, loc.y, loc.z])

        if len(waypoints) > min_pts:
            d = loc.distance(start_loc)
            # yaw 차이를 -180~+180 으로 환산
            dy = abs(((yaw - start_yaw + 180) % 360) - 180)
            if d < tol_dist and dy < tol_yaw:
                print(f"[INFO] 완전 루프 감지: 거리={d:.3f}m, yaw 차이={dy:.2f}° → 종료")
                # 첫 위치·높이를 그대로 다시 찍어 경로를 닫아줌
                waypoints.append([start_loc.x, start_loc.y, start_loc.z])
                break

        next_wps = wp.next(step)
        if not next_wps:
            print("[WARNING] 더 이상 다음 웨이포인트가 없습니다.")
            break
        wp = next_wps[0]

    # CSV 저장
    save_dir = os.path.expanduser("~/ros2_ws/src/town06_data")
    os.makedirs(save_dir, exist_ok=True)
    csv_path = os.path.join(save_dir, "0.csv")
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x","y","z"])
        for p in waypoints:
            writer.writerow(p)
    print(f"[SUCCESS] 총 {len(waypoints)}개 웨이포인트 → {csv_path}")

if __name__ == '__main__':
    main()

