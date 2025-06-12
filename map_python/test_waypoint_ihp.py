import carla
import csv
import os
import math

def main():
    # 1) CARLA 서버 연결 & 맵 로드
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.load_world("ihp_sce")   # 맵 이름을 ihp_sce로
    carla_map = world.get_map()

    # 2) 시작 웨이포인트: 토폴로지의 첫 번째 세그먼트 시작점
    topology = carla_map.get_topology()
    if not topology:
        raise RuntimeError("맵 토폴로지(도로)가 없습니다!")

    first_wp, _ = topology[0]
    start_wp     = first_wp
    start_tf     = start_wp.transform
    start_loc    = start_tf.location
    start_yaw    = start_tf.rotation.yaw

    # 3) 파라미터 설정
    step        = 0.5    # 웨이포인트 간격 (m)
    tol_dist    = 0.05   # 시작점 복귀 허용 오차 거리 (m)
    tol_yaw     = 1.0    # 시작점 복귀 허용 오차 회전 (deg)
    min_pts     = 20     # 최소 수집 포인트 수

    # 4) 순회 시작
    waypoints = []
    wp = start_wp

    while True:
        loc = wp.transform.location
        yaw = wp.transform.rotation.yaw
        waypoints.append([loc.x, loc.y, loc.z])

        # 일정량 수집 후, 시작점 근처로 돌아오면 종료
        if len(waypoints) > min_pts:
            d  = loc.distance(start_loc)
            dy = abs(((yaw - start_yaw + 180) % 360) - 180)
            if d < tol_dist and dy < tol_yaw:
                print(f"[INFO] 루프 종료 감지: 거리={d:.3f}m, yaw 차이={dy:.2f}°")
                # 닫힌 경로를 위해 시작점 다시 추가
                waypoints.append([start_loc.x, start_loc.y, start_loc.z])
                break

        # 다음 웨이포인트로 이동
        next_wps = wp.next(step)
        if not next_wps:
            print("[WARNING] 더 이상 다음 웨이포인트가 없습니다.")
            break
        wp = next_wps[0]

    # 5) CSV로 저장 (~/123.csv)
    home_dir = os.path.expanduser("~")
    csv_path = os.path.join(home_dir, "123.csv")
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "z"])
        writer.writerows(waypoints)

    print(f"[SUCCESS] 총 {len(waypoints)}개 포인트를 '{csv_path}'에 저장했습니다.")

if __name__ == "__main__":
    main()

