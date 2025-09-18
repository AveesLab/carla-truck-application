#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, time, carla
from typing import Optional

# 기본 인자 (원하면 커맨드라인에서 바꾸세요)
DEFAULT_TRUCK_MASS    = 7000.0   # kg (원래 9000에서 점진적 감소)
DEFAULT_TRAILER_MASS  = 3000.0   # kg
DEFAULT_TORQUE_SCALE  = 1.2      # 1.0 이면 토크 변경 없음 (1.4에서 1.2로 보수적 변경)

# 정지 판정
DEFAULT_SPEED_GATE = 0.05        # m/s (아주 작은 미동 허용)
DEFAULT_WAIT_SEC   = 3.0         # 최대 대기

def speed_mps(actor: carla.Actor) -> float:
    v = actor.get_velocity()
    return (v.x*v.x + v.y*v.y + v.z*v.z) ** 0.5

def almost_equal(a: float, b: float, eps: float = 1e-6) -> bool:
    return abs(a - b) <= eps

def debug_vehicle_physics(actor: carla.Actor) -> None:
    """차량의 물리 상태를 자세히 출력하는 디버깅 함수"""
    pc = actor.get_physics_control()
    ctrl = actor.get_control()
    print(f"=== Vehicle {actor.id} Physics Debug ===")
    print(f"   Mass: {pc.mass:.1f} kg")
    print(f"   Clutch: {pc.clutch_strength:.1f}")
    print(f"   Gear: {ctrl.gear} (manual: {ctrl.manual_gear_shift})")
    print(f"   Autobox: {pc.use_gear_autobox}")
    if pc.torque_curve:
        torque_sum = sum(p.y for p in pc.torque_curve)
        print(f"   Torque curve points: {len(pc.torque_curve)}, sum: {torque_sum:.1f}")
    print(f"   Speed: {speed_mps(actor):.2f} m/s")

def restore_safe_settings(actor: carla.Actor) -> bool:
    """안전한 기본 설정으로 복구하는 함수"""
    pc = actor.get_physics_control()
    print(f"[RESTORE] actor {actor.id}: restoring safe settings")
    
    # 안전한 클러치 값
    pc.clutch_strength = 80.0
    
    # 자동 기어박스 활성화
    pc.use_gear_autobox = True
    
    # 기어를 자동으로 설정
    ctrl = actor.get_control()
    ctrl.gear = 1
    ctrl.manual_gear_shift = False
    
    actor.apply_physics_control(pc)
    actor.apply_control(ctrl)
    
    print(f"   Applied: clutch=80.0, autobox=True, gear=auto")
    return True

def apply_without_physics_control(actor: carla.Actor, 
                                 new_mass: Optional[float],
                                 torque_scale: Optional[float]) -> bool:
    """apply_physics_control을 완전히 피하는 대안 방법"""
    world = actor.get_world()
    
    print(f"[ALTERNATIVE] actor {actor.id}: using control-based approach")
    
    # 기어박스 설정만 변경 (가장 안전)
    ctrl = actor.get_control()
    ctrl.manual_gear_shift = False  # 자동 기어박스
    ctrl.gear = 1
    actor.apply_control(ctrl)
    
    # 메시지 출력
    if new_mass:
        print(f"   Note: mass change to {new_mass} skipped (using control-based approach)")
    if torque_scale:
        print(f"   Note: torque scaling x{torque_scale} skipped (using control-based approach)")
    
    print(f"   Applied: auto gearbox enabled, manual control disabled")
    return True

def scale_torque_y_only(pc: carla.VehiclePhysicsControl, scale: float) -> bool:
    """
    토크커브의 y값만 배로 키움. x(RPM)과 커브 모양은 그대로 유지.
    이미 동일 스케일로 적용돼 있다면 다시 적용하지 않음(중복 스케일 방지).
    """
    if not pc.torque_curve or scale == 1.0:
        return False

    # 중복 적용 방지용: 총합 대비 비율로 대략 감지
    sum_before = sum(p.y for p in pc.torque_curve)
    new_curve = [carla.Vector2D(p.x, p.y * scale) for p in pc.torque_curve]
    sum_after  = sum(p.y for p in new_curve)

    # 스케일 효과가 거의 없으면 스킵
    if almost_equal(sum_before * scale, sum_after, eps=1e-3):
        return False

    pc.torque_curve = new_curve
    return True

def maybe_apply_safe(actor: carla.Actor,
                     new_mass: Optional[float],
                     torque_scale: Optional[float],
                     speed_gate=DEFAULT_SPEED_GATE,
                     wait_timeout=DEFAULT_WAIT_SEC) -> bool:
    """안전한 방식으로 물리 속성 적용 (carla 0.9.15.2 apply_physics_control 문제 회피)"""
    world = actor.get_world()

    # 1) 정지 대기
    t0 = time.time()
    while speed_mps(actor) > speed_gate and (time.time() - t0) < wait_timeout:
        try: world.wait_for_tick()
        except Exception: time.sleep(0.02)

    if speed_mps(actor) > speed_gate:
        print(f"[SKIP] actor {actor.id}: still moving -> no change")
        return False

    pc = actor.get_physics_control()
    changed = False

    # 2) 질량 (원하면만)
    if new_mass is not None:
        target = float(new_mass)
        if not almost_equal(pc.mass, target):
            print(f"   mass: {pc.mass:.1f} -> {target:.1f} kg")
            pc.mass = target
            changed = True

    # 3) 토크 y스케일 (원하면만)
    if torque_scale is not None and torque_scale != 1.0:
        if scale_torque_y_only(pc, float(torque_scale)):
            pts_after = [(int(round(p.x)), int(round(p.y))) for p in pc.torque_curve]
            print(f"   torque pts scaled x{torque_scale} -> {pts_after[:5]}{' …' if len(pts_after)>5 else ''}")
            changed = True
        else:
            print("   torque curve unchanged (maybe already scaled or empty)")

    # 4) ★ 클러치 강도 조정 (너무 높으면 차량이 멈출 수 있음)
    if pc.clutch_strength > 100.0:  # 너무 높으면 오히려 문제
        print(f"   clutch_strength: {pc.clutch_strength:.1f} -> 80.0 (기존 값이 너무 높음)")
        pc.clutch_strength = 80.0
        changed = True
    elif pc.clutch_strength < 50.0:  # 너무 낮으면 미끄러짐
        print(f"   clutch_strength: {pc.clutch_strength:.1f} -> 80.0")
        pc.clutch_strength = 80.0
        changed = True

    # 5) 실제 적용 - carla 0.9.15.2 안전 모드
    if not changed:
        print(f"[SKIP] actor {actor.id}: no effective change")
        return False

    # apply_physics_control 대신 단계적 적용
    try:
        # 먼저 차량 완전 정지 확인
        ctrl = actor.get_control()
        ctrl.throttle = 0.0
        ctrl.brake = 1.0
        actor.apply_control(ctrl)
        
        # 짧은 대기 후 물리 속성 적용
        time.sleep(0.1)
        world.tick()  # 한 틱 대기
        
        actor.apply_physics_control(pc)
        
        # 다시 한 틱 대기
        time.sleep(0.1)
        world.tick()
        
        # 브레이크 해제
        ctrl.brake = 0.0
        actor.apply_control(ctrl)
        
        print(f"[APPLY] actor {actor.id}: mass/torque updated safely (+ clutch adjustment)")
        return True
        
    except Exception as e:
        print(f"[ERROR] apply_physics_control failed: {e}")
        return False
    


def main():
    # 사용법: python3 set_truck_att.py <num_trucks> [torque_scale] [truck_mass] [trailer_mass] [옵션들]
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <num_trucks> [torque_scale] [truck_mass] [trailer_mass] [옵션들]")
        print(f"Example: python3 {sys.argv[0]} 3 1.2 7000 3000")
        print(f"         python3 {sys.argv[0]} 3 --debug           # 디버깅 모드")
        print(f"         python3 {sys.argv[0]} 3 --restore         # 안전 설정으로 복구")
        print(f"         python3 {sys.argv[0]} 3 --safe-mode       # carla 0.9.15.2 안전 모드")
        print(f"         python3 {sys.argv[0]} 3 --no-physics      # apply_physics_control 사용 안함")
        return
    
    # 옵션 확인
    debug_mode = "--debug" in sys.argv
    restore_mode = "--restore" in sys.argv
    safe_mode = "--safe-mode" in sys.argv
    no_physics = "--no-physics" in sys.argv

    num = int(sys.argv[1])
    torque_scale = float(sys.argv[2]) if len(sys.argv) >= 3 else DEFAULT_TORQUE_SCALE
    truck_mass   = float(sys.argv[3]) if len(sys.argv) >= 4 else DEFAULT_TRUCK_MASS
    trailer_mass = float(sys.argv[4]) if len(sys.argv) >= 5 else DEFAULT_TRAILER_MASS

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    time.sleep(0.5)

    vehicles = world.get_actors().filter("vehicle.*")
    print(f"Found {len(vehicles)} vehicles (torque x{torque_scale}, truck_mass={truck_mass}, trailer_mass={trailer_mass})")
    
    # 모드 표시
    if debug_mode:
        print("=== DEBUG MODE: 차량 상태만 확인 ===")
    elif restore_mode:
        print("=== RESTORE MODE: 안전 설정으로 복구 ===")
    elif no_physics:
        print("=== NO-PHYSICS MODE: apply_physics_control 사용 안함 ===")
    else:
        print("=== SAFE MODE: carla 0.9.15.2 호환 안전 적용 ===")

    def find_by_role(role):
        return next((v for v in vehicles if v.attributes.get("role_name") == role), None)

    for i in range(num):
        truck_role, trailer_role = f"truck{i}", f"trailer{i}"

        truck = find_by_role(truck_role)
        if truck and truck.is_alive:
            print(f"[TRUCK] {truck_role} id={truck.id}")
            try:
                if debug_mode:
                    debug_vehicle_physics(truck)
                elif restore_mode:
                    restore_safe_settings(truck)
                elif no_physics:
                    apply_without_physics_control(truck, truck_mass, torque_scale)
                else:
                    # 기본적으로 안전 모드 사용 (carla 0.9.15.2 고려)
                    maybe_apply_safe(truck, truck_mass, torque_scale)
                
                # 상태 출력(읽기만)
                ctrl = truck.get_control()
                pc   = truck.get_physics_control()
                print("   current state:",
                      {"gear": ctrl.gear, "manual": ctrl.manual_gear_shift,
                       "autobox": pc.use_gear_autobox, "clutch": round(pc.clutch_strength,1),
                       "mass": round(pc.mass,1)})
            except Exception as e:
                print(f"[ERROR] truck {truck_role}: {e}")
        else:
            print(f"[WARN] missing/dead: {truck_role}")

        trailer = find_by_role(trailer_role)
        if trailer and trailer.is_alive:
            print(f"[TRAILER] {trailer_role} id={trailer.id}")
            try:
                if debug_mode:
                    debug_vehicle_physics(trailer)
                elif restore_mode:
                    restore_safe_settings(trailer)
                elif no_physics:
                    apply_without_physics_control(trailer, trailer_mass, None)
                else:
                    # 기본적으로 안전 모드 사용 (carla 0.9.15.2 고려)
                    maybe_apply_safe(trailer, trailer_mass, None)
            except Exception as e:
                print(f"[ERROR] trailer {trailer_role}: {e}")
        else:
            print(f"[WARN] missing/dead: {trailer_role}")

    print("Done.")

if __name__ == "__main__":
    main()
