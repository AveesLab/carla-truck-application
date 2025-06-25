import carla, random, time

# 1) 서버 연결
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# 2) XODR 파일 읽어서 맵 전체 생성
with open('/home/avees/carla/Unreal/CarlaUE4/Content/map_package/Maps/mid_large_cycle/OpenDrive/mid_large_cycle.xodr') as f:
    opendrive = f.read()
params = carla.OpendriveGenerationParameters(
    vertex_distance       = 2.0,
    max_road_length       = 200.0,
    smooth_junctions      = True,
    enable_mesh_visibility= True
)
world = client.generate_opendrive_world(opendrive, params)
print("🌐 Entire OpenDRIVE world generated")

# 3) (선택) 스트리밍 거리 늘리기
settings = world.get_settings()
settings.tile_stream_distance  = 30000.0
settings.actor_active_distance = 30000.0
world.apply_settings(settings)

# 4) Waypoint로 위치, 높이, 방향 얻기
loc = carla.Location(x=19000.0, y=19000.0, z=100.0)
wp  = world.get_map().get_waypoint(loc, project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
if wp is None:
    raise RuntimeError("해당 위치에 driving 차선이 없습니다.")

# 5) 트럭 Blueprint 뽑아서 hero 역할 지정
truck_bps = world.get_blueprint_library().filter('vehicle.*truck*')
if not truck_bps:
    raise RuntimeError("Truck 모델이 없습니다.")
truck_bp = random.choice(truck_bps)
truck_bp.set_attribute('role_name', 'hero')

# 6) 스폰 시도
truck = world.try_spawn_actor(truck_bp, wp.transform)
print("🚚 Spawn", "성공!" if truck else "실패…")

