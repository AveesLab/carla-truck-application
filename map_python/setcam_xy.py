import carla

def move_camera_to(x, y, z, yaw_deg=0.0, pitch_deg=-45.0):
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    spectator = world.get_spectator()

    camera_transform = carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(pitch=pitch_deg, yaw=yaw_deg)
    )

    spectator.set_transform(camera_transform)
    print(f"[INFO] Moved camera to x={x}, y={y}, z={z}, yaw={yaw_deg}, pitch={pitch_deg}")

if __name__ == '__main__':
    # 예: (100, 50, 30) 좌표로 이동
    move_camera_to(x=-302.5, y=27.12, z=0.73, yaw_deg=90.0)

