#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import carla

# 기본 질량 설정값
TRUCK_MASS = 1000.0     # kg
TRAILER_MASS = 9000.0  # kg

def set_actor_mass(actor, new_mass):
    """CARLA vehicle actor 질량 변경"""
    control = actor.get_physics_control()
    before = control.mass
    control.mass = float(new_mass)
    actor.apply_physics_control(control)
    after = actor.get_physics_control().mass
    return before, after

def main():
    if len(sys.argv) != 2:
        print(f"Usage: python3 {sys.argv[0]} <number_of_trucks>")
        print(f"Example: python3 {sys.argv[0]} 3")
        return

    try:
        num_trucks = int(sys.argv[1])
    except ValueError:
        print("Error: number_of_trucks must be integer")
        return

    # CARLA 연결
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # 액터 안정화 대기
    time.sleep(2.0)

    vehicles = world.get_actors().filter("vehicle.*")
    print(f"Found {len(vehicles)} vehicles in world")

    for i in range(num_trucks):
        truck_role = f"truck{i}"
        trailer_role = f"trailer{i}"

        for role_name, mass in [(truck_role, TRUCK_MASS), (trailer_role, TRAILER_MASS)]:
            actor = next((v for v in vehicles if v.attributes.get("role_name") == role_name), None)
            if actor is None:
                print(f"[WARN] No actor with role_name '{role_name}' found")
                continue
            if not actor.is_alive:
                print(f"[WARN] Actor {actor.id} (role={role_name}) not alive")
                continue

            try:
                before, after = set_actor_mass(actor, mass)
                print(f"[OK] {role_name} (id={actor.id}): {before:.1f} kg -> {after:.1f} kg")
            except Exception as e:
                print(f"[ERROR] Failed to set mass for {role_name} (id={actor.id}): {e}")

    print("Mass setting complete.")

if __name__ == "__main__":
    main()
