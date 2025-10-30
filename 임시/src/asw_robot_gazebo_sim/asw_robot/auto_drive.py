#!/usr/bin/env python3
"""
로봇 자동 주행 예제
- 직진 2초
- 좌회전 2초
- 직진 2초
- 우회전 2초
반복
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class AutoDrive(Node):
    def __init__(self):
        super().__init__('auto_drive')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('자동 주행 시작!')
        
        # 속도 설정 (안전한 속도로 조정)
        self.linear_speed = 0.8  # 선속도 (m/s) - 천천히!
        self.angular_speed = 0.3  # 각속도 (rad/s)
        
    def publish_velocity(self, linear, angular):
        """속도 명령 발행"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        
    def drive_pattern(self):
        """주행 패턴 실행"""
        while rclpy.ok():
            # 직진
            self.get_logger().info('직진!')
            self.publish_velocity(self.linear_speed, 0.0)
            time.sleep(2.0)
            
            # 좌회전하며 전진
            self.get_logger().info('좌회전!')
            self.publish_velocity(self.linear_speed, self.angular_speed)
            time.sleep(2.0)
            
            # 직진
            self.get_logger().info('직진!')
            self.publish_velocity(self.linear_speed, 0.0)
            time.sleep(2.0)
            
            # 우회전하며 전진
            self.get_logger().info('우회전!')
            self.publish_velocity(self.linear_speed, -self.angular_speed)
            time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    node = AutoDrive()
    
    try:
        node.drive_pattern()
    except KeyboardInterrupt:
        node.get_logger().info('사용자가 중지함')
    finally:
        # 정지 명령
        node.publish_velocity(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

