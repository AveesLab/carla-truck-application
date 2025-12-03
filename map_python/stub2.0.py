#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt32

# ====== CONFIG: 여기만 고치면 됨 ======
TRUCK_DRAG = {
    'truck0': 0.500,   # 기준값
    'truck1': 0.500,   # 기준값
    'truck2': 0.500,   # 기준값
}
JITTER = 0.01          # 프레임 바뀔 때마다 ±JITTER 범위 랜덤
HZ = 100.0             # 0.01s
LOG_EVERY_N = 1        # N틱마다 로그
DRAG_MIN, DRAG_MAX = 0.0, 5.0
# random.seed(42)      # 재현성 필요시 사용
# ======================================

class DragSpamHardcoded(Node):
    def __init__(self):
        super().__init__('drag_spammer_hardcoded')

        # 검증
        for ns, v in TRUCK_DRAG.items():
            if not isinstance(v, (int, float)):
                raise ValueError(f'{ns} drag가 숫자가 아님: {v}')
            if not (DRAG_MIN <= v <= DRAG_MAX):
                raise ValueError(f'{ns} drag 범위 초과: {v} not in [{DRAG_MIN},{DRAG_MAX}]')

        # 퍼블리셔
        self.pubs = {ns: self.create_publisher(Float32, f'/{ns}/drag', 10)
                     for ns in TRUCK_DRAG.keys()}

        # 현재 퍼블리시 값(프레임 변경 시에만 갱신)
        self.curr = {ns: float(v) for ns, v in TRUCK_DRAG.items()}

        # 프레임 ID 구독
        self.frame_id = None
        self._last_frame_id = None
        self.create_subscription(UInt32, '/sim/frame_id', self._on_frame_id, 10)

        # 타이머(유지)
        self.tick_count = 0
        self.timer = self.create_timer(1.0 / HZ, self._tick)

        kv = ' '.join([f'{ns}:{v:.3f}' for ns, v in TRUCK_DRAG.items()])
        self.get_logger().info(f'namespaces={list(TRUCK_DRAG.keys())} hz={HZ:.1f} log_every_n={LOG_EVERY_N}')
        self.get_logger().info(f'base_drag={{ {kv} }}  jitter=±{JITTER:.3f}')
        self.get_logger().info('Subscribing /sim/frame_id (UInt32)')

    def _on_frame_id(self, msg: UInt32):
        self.frame_id = int(msg.data)
        # 프레임이 바뀌었을 때만 랜덤 업데이트
        if self.frame_id != self._last_frame_id:
            self._last_frame_id = self.frame_id
            for ns, base in TRUCK_DRAG.items():
                val = base + random.uniform(-JITTER, JITTER)
                # 클램프
                if val < DRAG_MIN: val = DRAG_MIN
                if val > DRAG_MAX: val = DRAG_MAX
                self.curr[ns] = val

    def _tick(self):
        self.tick_count += 1
        # 현재 값 퍼블리시
        for ns, pub in self.pubs.items():
            pub.publish(Float32(data=self.curr[ns]))

        if self.tick_count % max(1, LOG_EVERY_N) == 0:
            now = self.get_clock().now().to_msg()
            t = now.sec + now.nanosec * 1e-9
            fid = self.frame_id if self.frame_id is not None else -1
            kv = ' '.join([f'{ns}:{self.curr[ns]:.3f}' for ns in self.pubs.keys()])
            self.get_logger().info(f'[tick={self.tick_count}] t={t:.6f}s frame_id={fid} {kv}')


def main():
    rclpy.init()
    node = DragSpamHardcoded()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
