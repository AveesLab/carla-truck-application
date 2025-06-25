import carla, random, traceback

# 1) CARLA 서버 연결
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# 2) Waypoint 얻기 — 이 부분이 빠져 있으면 wp가 정의되지 않습니다.
loc = carla.Location(x=18980.0, y=19509.0, z=100.0)  # 충분히 높은 Z
wp  = world.get_map().get_waypoint(
    loc,
    project_to_road=True,
    lane_type=carla.LaneType.Driving
)

if wp is None:
    raise RuntimeError("❌ Waypoint가 없습니다. 도로가 로드되지 않았거나 좌표가 잘못되었습니다.")

# 3) Transform 추출
transform = wp.transform

# 4) Blueprint 선택 및 'hero' 역할 지정
bp = random.choice(world.get_blueprint_library().filter('vehicle.*'))
bp.set_attribute('role_name', 'hero')

print(f"Attempting spawn at {transform.location}, yaw={transform.rotation.yaw}")

# 5) spawn_actor 로 예외 확인하면서 스폰 시도
try:
    vehicle = world.spawn_actor(bp, transform)
    print("✅ Spawn succeeded:", vehicle.id)
except Exception:
    print("❌ Spawn failed with exception:")
    traceback.print_exc()

