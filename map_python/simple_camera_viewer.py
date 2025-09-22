#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class SimpleCameraViewer(Node):
    def __init__(self):
        super().__init__('simple_camera_viewer')
        
        # 카메라 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            'camera0',  # 카메라 토픽명
            self.image_callback,
            10
        )
        
        self.get_logger().info('간단한 Camera Viewer 시작됨. ESC 키로 종료')

    def image_callback(self, msg):
        try:
            # ROS Image를 직접 numpy array로 변환
            # bgra8 형식이므로 4채널
            height = msg.height
            width = msg.width
            
            # 이미지 데이터를 numpy array로 변환
            img_array = np.frombuffer(msg.data, dtype=np.uint8)
            img_array = img_array.reshape((height, width, 4))  # BGRA
            
            # BGR로 변환 (알파 채널 제거)
            img_bgr = img_array[:, :, :3]
            
            # 이미지 표시
            cv2.imshow('CARLA Camera', img_bgr)
            
            # ESC 키로 종료
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC key
                self.get_logger().info('ESC 키 눌림. 종료 중...')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'이미지 처리 오류: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        viewer = SimpleCameraViewer()
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"오류: {e}")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
