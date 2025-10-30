#!/usr/bin/env python3
"""
카메라 기반 간단한 비전 제어 (YOLO 없이 OpenCV만 사용)
- 특정 색상 추적
- 간단한 차선 감지
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraFollower(Node):
    def __init__(self):
        super().__init__('camera_follower')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscriber & Publisher
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 파라미터
        self.linear_speed = 0.5
        self.angular_speed = 0.5
        
        self.get_logger().info('카메라 추적 시작!')
        
    def image_callback(self, msg):
        try:
            # ROS Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 이미지 처리
            height, width = cv_image.shape[:2]
            
            # 하단 영역만 확인 (차선 영역)
            roi = cv_image[int(height*0.6):height, :]
            
            # 그레이스케일 변환
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            
            # 엣지 검출
            edges = cv2.Canny(gray, 50, 150)
            
            # 좌우 영역 나누기
            mid = width // 2
            left_edges = edges[:, :mid]
            right_edges = edges[:, mid:]
            
            # 좌우 엣지 개수 세기
            left_count = np.count_nonzero(left_edges)
            right_count = np.count_nonzero(right_edges)
            
            # 제어 명령 생성
            cmd = Twist()
            
            if left_count > 1000 or right_count > 1000:
                # 차선이 감지됨
                cmd.linear.x = self.linear_speed
                
                # 좌우 균형 조정
                if left_count > right_count * 1.5:
                    # 왼쪽에 더 많은 차선 -> 우회전
                    cmd.angular.z = -self.angular_speed * 0.3
                    self.get_logger().info('우회전')
                elif right_count > left_count * 1.5:
                    # 오른쪽에 더 많은 차선 -> 좌회전
                    cmd.angular.z = self.angular_speed * 0.3
                    self.get_logger().info('좌회전')
                else:
                    # 균형 -> 직진
                    cmd.angular.z = 0.0
                    self.get_logger().info('직진')
            else:
                # 차선이 없음 -> 정지
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info('차선 없음 - 정지')
            
            self.cmd_vel_pub.publish(cmd)
            
            # 디버깅용 이미지 표시
            cv2.imshow('Camera View', cv_image)
            cv2.imshow('ROI', roi)
            cv2.imshow('Edges', edges)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'이미지 처리 오류: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자가 중지함')
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

