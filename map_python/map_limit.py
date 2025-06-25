import carla

# 1) CARLA 서버에 연결
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# 2) 맵 topology(도로망) 가져오기
topology = world.get_map().get_topology()

# 3) 모든 segment 끝점에서 X, Y 추출
xs = []
ys = []
for seg in topology:
    wp_start, wp_end = seg
    xs.append(wp_start.transform.location.x)
    xs.append(wp_end.transform.location.x)
    ys.append(wp_start.transform.location.y)
    ys.append(wp_end.transform.location.y)

# 4) 경계 계산
min_x, max_x = min(xs), max(xs)
min_y, max_y = min(ys), max(ys)

print(f"Map X range: {min_x:.3f}  to  {max_x:.3f}")
print(f"Map Y range: {min_y:.3f}  to  {max_y:.3f}")

