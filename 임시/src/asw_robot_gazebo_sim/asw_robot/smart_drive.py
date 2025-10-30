#!/usr/bin/env python3
"""
스마트 주행 예제 - 레이저 센서를 활용한 장애물 회피
전방에 장애물이 있으면 회피합니다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class SmartDrive(Node):
    def __init__(self):
        super().__init__('smart_drive')
        
        # Publisher와 Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.laser_callback, 
            10
        )
        
        # 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 상태 변수
        self.obstacle_detected = False
        self.min_distance = float('inf')
        self.obstacle_angle = 0.0
        
        # 속도 파라미터
        self.normal_speed = 1.5
        self.safe_distance = 1.5  # 안전 거리 (미터)
        
        self.get_logger().info('스마트 주행 시작! (장애물 회피 기능)')
        
    def laser_callback(self, msg):
        """레이저 스캔 데이터 처리"""
        ranges = np.array(msg.ranges)
        
        # 무한대 값 제거
        ranges = np.where(np.isinf(ranges), 100.0, ranges)
        
        # 전방 90도 범위 확인 (정면 ±45도)
        front_ranges = np.concatenate([
            ranges[-45:],  # 우측 45도
            ranges[:45]    # 좌측 45도
        ])
        
        self.min_distance = np.min(front_ranges)
        
        if self.min_distance < self.safe_distance:
            self.obstacle_detected = True
            # 장애물이 있는 각도 찾기
            min_idx = np.argmin(front_ranges)
            if min_idx >= 45:
                self.obstacle_angle = -(min_idx - 45)  # 우측
            else:
                self.obstacle_angle = 45 - min_idx  # 좌측
        else:
            self.obstacle_detected = False
            
    def control_loop(self):
        """제어 루프"""
        msg = Twist()
        
        if self.obstacle_detected:
            # 장애물 발견! 회피 기동
            self.get_logger().info(
                f'장애물 감지! 거리: {self.min_distance:.2f}m - 회피중...'
            )
            
            # 속도 줄이고 회전
            msg.linear.x = 0.3
            
            # 장애물이 오른쪽에 있으면 왼쪽으로, 왼쪽에 있으면 오른쪽으로
            if self.obstacle_angle > 0:
                msg.angular.z = 0.8  # 좌회전
            else:
                msg.angular.z = -0.8  # 우회전
        else:
            # 전방 안전 - 직진
            msg.linear.x = self.normal_speed
            msg.angular.z = 0.0
            
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SmartDrive()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자가 중지함')
    finally:
        # 정지 명령
        msg = Twist()
        node.cmd_vel_pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

