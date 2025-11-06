#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, time
import carla

HOST = 'localhost'
PORT = 2000
WAIT_SEC = 60.0     # 차량 대기 최대시간
POLL = 0.5          # 폴링 간격
HZ = 10.0           # 출력 주기(Hz)
ROLES = ['truck0', 'truck1', 'truck2']  # 필요시 수정

def find_by_role(world, roles):
    actors = world.get_actors().filter('vehicle.*')
    by_role = {a.attributes.get('role_name',''): a for a in actors}
    return {r: by_role.get(r) for r in roles}

def wait_vehicles(world, timeout=WAIT_SEC, poll=POLL):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if len(world.get_actors().filter('vehicle.*')) > 0:
            return True
        time.sleep(poll)
    return False

def get_drag(vehicle):
    pc = vehicle.get_physics_control()
    # 일부 블루프린트에 drag_coefficient가 없을 수 있어 getattr 사용
    return float(getattr(pc, 'drag_coefficient', float('nan')))

def main():
    port = PORT
    if len(sys.argv) >= 2 and sys.argv[1].isdigit():
        port = int(sys.argv[1])

    client = carla.Client(HOST, port)
    client.set_timeout(10.0)
    world = client.get_world()

    if not wait_vehicles(world):
        print(f"[ERROR] no vehicles after {WAIT_SEC:.0f}s wait"); return

    targets = {k:v for k,v in find_by_role(world, ROLES).items() if v is not None}
    if not targets:
        print("[WARN] target roles not found:", ROLES)

    period = 1.0 / HZ
    print(f"[INFO] watching drag at {HZ:.1f} Hz for roles: {list(targets.keys())}")
    try:
        while True:
            # 재스폰 대비 재바인딩
            for rn in list(ROLES):
                veh = targets.get(rn)
                if veh is None or not veh.is_alive:
                    reb = find_by_role(world, [rn]).get(rn)
                    if reb and reb.is_alive:
                        targets[rn] = reb
                        veh = reb
                    else:
                        # 아직 없으면 스킵
                        continue
                try:
                    d = get_drag(veh)
                    print(f"{rn:8s} drag={d:.6f}")
                except RuntimeError:
                    print(f"{rn:8s} drag=NA (actor not ready)")
            time.sleep(period)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()