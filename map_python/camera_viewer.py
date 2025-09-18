#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 카메라 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            'camera0',  # 카메라 토픽명
            self.image_callback,
            10
        )
        
        self.get_logger().info('Camera Viewer 시작됨. ESC 키로 종료')

    def image_callback(self, msg):
        try:
            # ROS Image를 OpenCV 이미지로 변환
            # bgra8 -> bgr8 (알파 채널 제거)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 이미지 표시
            cv2.imshow('CARLA Camera', cv_image)
            
            # ESC 키로 종료
            if cv2.waitKey(1) & 0xFF == 27:  # ESC key
                self.get_logger().info('ESC 키 눌림. 종료 중...')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'이미지 처리 오류: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        viewer = CameraViewer()
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
