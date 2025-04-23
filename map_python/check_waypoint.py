import carla
import math
import csv

def get_distance(loc1, loc2):
    dx = loc1.x - loc2.x
    dy = loc1.y - loc2.y
    dz = loc1.z - loc2.z
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def main():
    # CARLA 연결
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    # 차량 찾기 (맨 앞 차량 사용)
    vehicles = world.get_actors().filter('vehicle.*')
    if not vehicles:
        print("차량이 없습니다.")
        return
    vehicle = vehicles[0]
    vehicle_loc = vehicle.get_location()
    print(f"[차량 위치] x={vehicle_loc.x:.2f}, y={vehicle_loc.y:.2f}, z={vehicle_loc.z:.2f}")

    # 웨이포인트 로딩
    csv_path = '/home/avees/ros2_ws/src/truck_controller/data/0.csv'
    waypoints = []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # 헤더 스킵
        for row in reader:
            x, y, z = float(row[0]), float(row[1]), float(row[2])
            waypoints.append(carla.Location(x, y, z))

    if len(waypoints) <= 18250:
        print(f"웨이포인트가 {len(waypoints)}개밖에 없습니다.")
        return

    target_wp = waypoints[18250]
    print(f"[18250번 웨이포인트 위치] x={target_wp.x:.2f}, y={target_wp.y:.2f}, z={target_wp.z:.2f}")

    # 거리 계산
    dist = get_distance(vehicle_loc, target_wp)
    print(f"차량과 18250번 웨이포인트 사이 거리: {dist:.2f} m")

if __name__ == '__main__':
    main()

