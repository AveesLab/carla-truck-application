import carla

def move_camera_to_waypoint(csv_path, index=14782):
    # CARLA 클라이언트 연결
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    # CSV 파일에서 waypoint 로딩
    import csv
    waypoints = []
    with open(csv_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # 헤더 스킵
        for row in reader:
            x = float(row[0])
            y = float(row[1])
            z = float(row[2])
            waypoints.append(carla.Transform(carla.Location(x, y, z)))

    # 카메라 이동
    if 0 <= index < len(waypoints):
        target_tf = waypoints[index]
        spectator = world.get_spectator()
        camera_tf = carla.Transform(
            carla.Location(
                target_tf.location.x,
                target_tf.location.y,
                target_tf.location.z + 10
            ),
            carla.Rotation(pitch=-45, yaw=target_tf.rotation.yaw)
        )
        spectator.set_transform(camera_tf)
        print(f"[INFO] Camera moved to waypoint {index}")
    else:
        print(f"[WARN] Index {index} is out of range (max: {len(waypoints) - 1})")

# 사용 예시
if __name__ == '__main__':
    move_camera_to_waypoint('/home/avees/ros2_ws/src/truck_controller/data/0.csv', index=18280)

