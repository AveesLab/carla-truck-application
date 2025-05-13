import carla
import math

def get_camera_look_at_ground():
    client = carla.Client("localhost", 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    spectator = world.get_spectator()
    transform = spectator.get_transform()

    location = transform.location
    rotation = transform.rotation

    # pitch, yaw → 방향 벡터 계산
    pitch = math.radians(rotation.pitch)
    yaw = math.radians(rotation.yaw)

    direction = carla.Vector3D(
        math.cos(pitch) * math.cos(yaw),
        math.cos(pitch) * math.sin(yaw),
        math.sin(pitch)
    )

    # 광선이 z=0 지면과 만나는 위치 t 계산
    if direction.z == 0:
        print("Camera is parallel to the ground, can't compute intersection.")
        return

    t = -location.z / direction.z  # z=0일 때

    # 시선이 닿는 좌표
    hit_x = location.x + direction.x * t
    hit_y = location.y + direction.y * t
    hit_z = 0.0

    print(f"[LOOK AT] x={hit_x:.2f}, y={hit_y:.2f}, z={hit_z:.2f}")

if __name__ == "__main__":
    get_camera_look_at_ground()

