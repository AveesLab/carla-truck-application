# print_camera_pose.py
import carla
import time

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world     = client.get_world()
    spectator = world.get_spectator()

    print(">>> 현재 스펙테이터 카메라 위치·회전 출력 (Ctrl+C 로 종료) <<<")
    try:
        while True:
            tf  = spectator.get_transform()
            loc = tf.location
            rot = tf.rotation
            print(f"x={loc.x:8.3f}  y={loc.y:8.3f}  z={loc.z:8.3f}  |  "
                  f"pitch={rot.pitch:6.2f}  yaw={rot.yaw:6.2f}  roll={rot.roll:6.2f}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n종료합니다.")

if __name__ == "__main__":
    main()

