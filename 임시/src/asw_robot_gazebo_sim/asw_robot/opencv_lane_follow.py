#!/usr/bin/env python3
"""
OpenCV 기반 차선 추종 (YOLO 불필요)
- Hough Line Transform
- HSV 색공간 필터링
- PID 제어
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class OpenCVLaneFollower(Node):
    def __init__(self):
        super().__init__('opencv_lane_follower')
        
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
        
        # 이미지 크기
        self.img_width = 640
        self.img_height = 480
        
        # ROI 설정
        self.roi_top_offset = 200  # ROI 상단 y 좌표
        self.roi_height = 200      # ROI 높이
        
        # HSV 노란색 차선 범위
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])
        
        # 차선 중심값
        self.left_line_x = 0
        self.right_line_x = 640
        self.lane_center = 320
        
        # PID 제어 파라미터
        self.kp = 0.008
        self.kd = 0.003
        self.error_old = 0.0
        
        # 속도 설정
        self.normal_speed = 0.6
        self.slow_speed = 0.3
        
        # 차선 검출 플래그
        self.left_detected = False
        self.right_detected = False
        
        self.get_logger().info('OpenCV 차선 추종 시작! (YOLO 없음)')
        
    def detect_line(self, roi, side='left'):
        """ROI에서 차선 검출"""
        # HSV 변환
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 노란색 필터링
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # Canny edge detection
        blur = cv2.GaussianBlur(mask, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        # Hough Line Transform
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=30,
            minLineLength=30,
            maxLineGap=50
        )
        
        if lines is None:
            return None, mask
        
        # 유효한 라인 필터링 (각도 제한)
        valid_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            if x2 - x1 == 0:
                continue
                
            angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
            
            # 각도 필터링 (거의 수직선 제외)
            if abs(angle) < 70:  # 70도 이하만 허용
                valid_lines.append(line[0])
        
        if len(valid_lines) == 0:
            return None, mask
        
        # 라인들의 중심 x 좌표 평균
        x_coords = []
        for x1, y1, x2, y2 in valid_lines:
            # ROI 중앙 높이에서의 x 좌표 계산
            mid_y = self.roi_height // 2
            if y2 - y1 != 0:
                slope = (x2 - x1) / (y2 - y1)
                x_at_mid = x1 + slope * (mid_y - y1)
                x_coords.append(x_at_mid)
        
        if len(x_coords) > 0:
            avg_x = int(np.mean(x_coords))
            return avg_x, mask
        
        return None, mask
    
    def image_callback(self, msg):
        try:
            # ROS Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            height, width = cv_image.shape[:2]
            self.img_width = width
            self.img_height = height
            
            # ROI 영역 설정
            roi_top = self.roi_top_offset
            roi_bottom = roi_top + self.roi_height
            
            # 왼쪽 ROI
            left_roi = cv_image[roi_top:roi_bottom, 0:width//2]
            # 오른쪽 ROI
            right_roi = cv_image[roi_top:roi_bottom, width//2:width]
            
            # 차선 검출
            left_x, left_mask = self.detect_line(left_roi, 'left')
            right_x, right_mask = self.detect_line(right_roi, 'right')
            
            # 전역 좌표로 변환
            if left_x is not None:
                self.left_line_x = left_x
                self.left_detected = True
            else:
                self.left_detected = False
            
            if right_x is not None:
                self.right_line_x = right_x + width // 2  # 오른쪽 ROI offset 더하기
                self.right_detected = True
            else:
                self.right_detected = False
            
            # 차선 중심 계산
            if self.left_detected and self.right_detected:
                # 양쪽 차선 모두 검출
                self.lane_center = (self.left_line_x + self.right_line_x) // 2
            elif self.left_detected:
                # 왼쪽만 검출 (오른쪽은 추정)
                self.lane_center = self.left_line_x + 160  # 차선 폭 가정
            elif self.right_detected:
                # 오른쪽만 검출 (왼쪽은 추정)
                self.lane_center = self.right_line_x - 160
            
            # 제어 계산
            self.control_robot()
            
            # 디버그 이미지 표시
            display = cv_image.copy()
            
            # ROI 박스 그리기
            cv2.rectangle(display, (0, roi_top), (width//2, roi_bottom), (0, 255, 0), 2)
            cv2.rectangle(display, (width//2, roi_top), (width, roi_bottom), (0, 255, 0), 2)
            
            # 차선 중심선 그리기
            if self.left_detected:
                cv2.line(display, (self.left_line_x, roi_top), 
                        (self.left_line_x, roi_bottom), (0, 255, 255), 3)
            if self.right_detected:
                cv2.line(display, (self.right_line_x, roi_top), 
                        (self.right_line_x, roi_bottom), (0, 255, 255), 3)
            
            # 목표 중심선
            cv2.line(display, (self.lane_center, roi_top), 
                    (self.lane_center, roi_bottom), (255, 255, 0), 3)
            
            # 이미지 중앙선
            cv2.line(display, (width//2, 0), (width//2, height), (0, 0, 255), 2)
            
            # 상태 텍스트
            status = f"L: {self.left_detected} R: {self.right_detected} Center: {self.lane_center}"
            cv2.putText(display, status, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow('OpenCV Lane Detection', display)
            cv2.imshow('Left Mask', left_mask)
            cv2.imshow('Right Mask', right_mask)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'이미지 처리 오류: {e}')
    
    def control_robot(self):
        """로봇 제어"""
        cmd = Twist()
        
        # 오차 계산 (이미지 중심 - 차선 중심)
        error = self.img_width // 2 - self.lane_center
        
        # PID 제어
        derivative = error - self.error_old
        angular_z = self.kp * error + self.kd * derivative
        
        # 조향 제한
        angular_z = np.clip(angular_z, -1.0, 1.0)
        
        # 속도 조절
        if abs(error) > 100:
            cmd.linear.x = self.slow_speed
        else:
            cmd.linear.x = self.normal_speed
        
        cmd.angular.z = angular_z
        
        # 차선 검출 실패 시
        if not self.left_detected and not self.right_detected:
            cmd.linear.x = 0.2
            cmd.angular.z = self.error_old * self.kp * 0.5  # 이전 방향 유지
            self.get_logger().warn('차선 미검출!')
        
        self.cmd_vel_pub.publish(cmd)
        self.error_old = error
        
        # 로그
        if abs(error) > 10:
            self.get_logger().info(
                f'오차: {error:4.0f}px | 조향: {angular_z:+.3f} | 속도: {cmd.linear.x:.2f}m/s'
            )

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVLaneFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자가 중지함')
    finally:
        # 정지
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

