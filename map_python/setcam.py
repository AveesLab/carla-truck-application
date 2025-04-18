import carla
import math

# 스폰 위치 수동 지정 (예: Town04_Opt의 트럭 0 위치)
truck_spawn_locations = {
    "Town04_Opt": {
        0: {  # truck_num 0
            "x": -5.0,
            "y": 130.0,
            "z": 0.3,
            "pitch": 0.0,
            "yaw": 0.0,
            "roll": 0.0
        }
    }
}

def get_transform_from_params(x, y, z, pitch, yaw, roll):
    location = carla.Location(x=x, y=y, z=z)
    rotation = carla.Rotation(
        pitch=pitch,
        yaw=yaw,
        roll=roll
    )
    return carla.Transform(location, rotation)

# === CARLA 클라이언트 연결 ===
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
spectator = world.get_spectator()

# === 차량 확인 ===
vehicles = world.get_actors().filter('vehicle.*')

if vehicles:
    vehicle = vehicles[0]
    transform = vehicle.get_transform()
    spectator.set_transform(carla.Transform(
        transform.location + carla.Location(z=3),
        transform.rotation
    ))
    print("✅ 차량 위치로 카메라 이동 완료")
else:
    map_name = world.get_map().name.split('/')[-1]  # 예: Town04_Opt
    truck_num = 0

    if map_name in truck_spawn_locations and truck_num in truck_spawn_locations[map_name]:
        pos = truck_spawn_locations[map_name][truck_num]
        transform = get_transform_from_params(
            pos["x"], pos["y"], pos["z"] + 3.0,  # 위쪽에서 보기 위해 z += 3.0
            pos["pitch"], pos["yaw"], pos["roll"]
        )
        spectator.set_transform(transform)
        print(f"📍 차량 없음 → {map_name} 맵의 트럭 {truck_num} 위치로 이동")
    else:
        print("❌ 트럭 스폰 위치 정보를 찾을 수 없습니다.")

