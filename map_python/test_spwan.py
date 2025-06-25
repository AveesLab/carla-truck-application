import carla

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

world = client.load_world("mid_large_cycle")
carla_map = world.get_map()

waypoints = carla_map.generate_waypoints(distance=2.0)

with open("0.csv", "w") as f:
    f.write("x,y,z,yaw,road_id,lane_id\n")
    for wp in waypoints:
        loc = wp.transform.location
        yaw = wp.transform.rotation.yaw
        road_id = wp.road_id
        lane_id = wp.lane_id

        # 바깥쪽 순환 도로 필터: lane_id < 0 (왼쪽 차선), road_type == Driving
        if wp.lane_id < 0 and wp.lane_type == carla.LaneType.Driving:
            f.write(f"{loc.x:.3f},{loc.y:.3f},{loc.z:.3f},{yaw:.2f},{road_id},{lane_id}\n")
