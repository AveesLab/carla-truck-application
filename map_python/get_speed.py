import carla
import time
import math

def get_speed(vehicle):
    vel = vehicle.get_velocity()
    return math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)  # [m/s]

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    # 트럭 필터 (예: DAF XF)
    trucks = world.get_actors().filter('vehicle.daf.dafxf*')

    if not trucks:
        print("[ERROR] No truck found in the world.")
        return

    truck = trucks[0]
    print(f"[INFO] Tracking truck ID: {truck.id}, Type: {truck.type_id}")

    while True:
        speed_mps = get_speed(truck)
        speed_kph = speed_mps * 3.6
        print(f"[SPEED] {speed_mps:.2f} m/s | {speed_kph:.2f} km/h")
        time.sleep(0.2)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")

