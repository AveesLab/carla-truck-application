#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
watch_mass.py
  - CARLA에서 특정 role_name( truck{i}, trailer{i} ) 차량들의 '현재 질량(kg)'을 실시간으로 출력
  - 물리 파라미터 변경은 절대 하지 않음 (READ-ONLY)
사용법:
  python3 watch_mass.py <num_pairs> [period_sec]
예:
  python3 watch_mass.py 3            # truck0..2, trailer0..2를 0.5초마다 출력
  python3 watch_mass.py 3 0.2        # 0.2초마다 출력
종료: Ctrl+C
"""

import sys, time
import carla
from collections import defaultdict

DEFAULT_PERIOD = 0.5  # 초

def v_mps(actor: carla.Actor) -> float:
    v = actor.get_velocity()
    return (v.x*v.x + v.y*v.y + v.z*v.z) ** 0.5

def find_by_role(actors, role):
    return next((a for a in actors if a.attributes.get("role_name") == role), None)

def describe(actor):
    pc = actor.get_physics_control()
    return {
        "id": actor.id,
        "mass": pc.mass,
    }

def main():
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <num_pairs> [period_sec]")
        sys.exit(1)

    num_pairs = int(sys.argv[1])
    period = float(sys.argv[2]) if len(sys.argv) >= 3 else DEFAULT_PERIOD

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    last = defaultdict(lambda: None)  # role_name -> last_mass
    print(f"[INFO] watching mass every {period:.2f}s for truck0..{num_pairs-1}, trailer0..{num_pairs-1}")

    try:
        while True:
            actors = world.get_actors().filter("vehicle.*")
            tstamp = time.strftime("%H:%M:%S")

            lines = []
            for i in range(num_pairs):
                for role in (f"truck{i}", f"trailer{i}"):
                    a = find_by_role(actors, role)
                    if not a or not a.is_alive:
                        lines.append(f"{tstamp}  {role:<9}  (missing)")
                        continue

                    pc = a.get_physics_control()
                    mass = pc.mass
                    spd = v_mps(a)

                    # 변경 감지(이전 mass와 다르면 * 표시)
                    mark = ""
                    prev = last[role]
                    if prev is None or abs(prev - mass) > 1e-6:
                        mark = " *"  # changed
                        last[role] = mass

                    lines.append(f"{tstamp}  {role:<9} id={a.id:<4} mass={mass:8.1f} kg  v={spd:6.3f} m/s{mark}")

            # 깔끔하게 한 번에 출력
            print("\n".join(lines))

            # 틱/슬립
            try:
                world.wait_for_tick()
            except Exception:
                pass
            time.sleep(max(0.0, period))
    except KeyboardInterrupt:
        print("\n[INFO] stopped.")

if __name__ == "__main__":
    main()

