#!/usr/bin/env python3
"""
로봇 키보드 제어 노드
w/s: 전진/후진
a/d: 좌회전/우회전  
x: 정지
q: 종료
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class RobotTeleop(Node):
    def __init__(self):
        super().__init__('robot_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('로봇 키보드 제어 시작!')
        self.get_logger().info('w/s: 전진/후진, a/d: 좌회전/우회전, x: 정지, q: 종료')
        
        # 속도 설정
        self.linear_speed = 1.0  # 선속도 (m/s)
        self.angular_speed = 0.5  # 각속도 (rad/s)
        
    def get_key(self):
        """키보드 입력 받기"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_velocity(self, linear, angular):
        """속도 명령 발행"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        self.get_logger().info(f'속도 명령: linear={linear:.2f}, angular={angular:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotTeleop()
    
    # 터미널 설정 저장
    node.settings = termios.tcgetattr(sys.stdin)
    
    try:
        while True:
            key = node.get_key()
            
            if key == 'w':
                # 전진
                node.publish_velocity(node.linear_speed, 0.0)
            elif key == 's':
                # 후진
                node.publish_velocity(-node.linear_speed, 0.0)
            elif key == 'a':
                # 좌회전
                node.publish_velocity(node.linear_speed, node.angular_speed)
            elif key == 'd':
                # 우회전
                node.publish_velocity(node.linear_speed, -node.angular_speed)
            elif key == 'x':
                # 정지
                node.publish_velocity(0.0, 0.0)
            elif key == 'q':
                # 종료
                node.get_logger().info('프로그램 종료')
                break
            else:
                node.get_logger().info(f'알 수 없는 키: {key}')
                
    except Exception as e:
        node.get_logger().error(f'오류 발생: {e}')
    finally:
        # 정지 명령
        node.publish_velocity(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

