#!/usr/bin/env python3
"""
SLLIDAR 각도 필터링 노드
원하는 각도 범위의 스캔 데이터만 필터링해서 발행합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class AngleFilterNode(Node):
    def __init__(self):
        super().__init__('angle_filter_node')
        
        # 파라미터 선언 (각도는 도(degree) 단위, 거리는 미터 단위)
        self.declare_parameter('angle_min_deg', 30.0)
        self.declare_parameter('angle_max_deg', 60.0)
        self.declare_parameter('range_min', 0.0)  # 최소 거리 (미터)
        self.declare_parameter('range_max', 2.0)  # 최대 거리 (미터)
        
        # 파라미터 가져오기
        self.target_angle_min_deg = self.get_parameter('angle_min_deg').value
        self.target_angle_max_deg = self.get_parameter('angle_max_deg').value
        self.target_range_min = self.get_parameter('range_min').value
        self.target_range_max = self.get_parameter('range_max').value
        
        # 라디안으로 변환
        self.target_angle_min_rad = math.radians(self.target_angle_min_deg)
        self.target_angle_max_rad = math.radians(self.target_angle_max_deg)
        
        self.get_logger().info(f'각도 필터링 범위: {self.target_angle_min_deg}° ~ {self.target_angle_max_deg}°')
        self.get_logger().info(f'거리 필터링 범위: {self.target_range_min}m ~ {self.target_range_max}m')
        
        # 구독 및 발행
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.filter_callback,
            10)
        
        self.publisher = self.create_publisher(
            LaserScan,
            'scan_filtered',
            10)
    
    def filter_callback(self, msg):
        # 각 포인트의 실제 각도를 계산하여 필터링
        filtered_ranges = []
        filtered_intensities = []
        filtered_angles = []
        
        for i in range(len(msg.ranges)):
            # 현재 포인트의 각도 계산 (라디안)
            current_angle_rad = msg.angle_min + i * msg.angle_increment
            # 0~2π 범위로 정규화
            current_angle_rad = (current_angle_rad + 2 * math.pi) % (2 * math.pi)
            # 도 단위로 변환
            current_angle_deg = math.degrees(current_angle_rad)
            
            # 목표 각도 범위 정규화 (0~360)
            target_min = self.target_angle_min_deg % 360
            target_max = self.target_angle_max_deg % 360
            
            # 각도 범위 체크 (순환 고려)
            in_angle_range = False
            if target_min <= target_max:
                # 일반 범위 (예: 30~150도)
                in_angle_range = target_min <= current_angle_deg <= target_max
            else:
                # 순환 범위 (예: 330~30도)
                in_angle_range = current_angle_deg >= target_min or current_angle_deg <= target_max
            
            if in_angle_range:
                filtered_angles.append(current_angle_rad)
                filtered_ranges.append(msg.ranges[i])
                filtered_intensities.append(msg.intensities[i])
        
        # 필터링된 데이터가 없으면 종료
        if len(filtered_ranges) == 0:
            self.get_logger().warn('필터링 결과 데이터가 없습니다. 각도 범위를 확인하세요.')
            return
        
        # 새로운 LaserScan 메시지 생성
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = min(filtered_angles)
        filtered_msg.angle_max = max(filtered_angles)
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = max(msg.range_min, self.target_range_min)
        filtered_msg.range_max = min(msg.range_max, self.target_range_max)
        
        # 거리 필터링 적용
        final_ranges = []
        final_intensities = []
        for r, i in zip(filtered_ranges, filtered_intensities):
            # 거리가 설정 범위 내에 있는지 확인
            if self.target_range_min <= r <= self.target_range_max:
                final_ranges.append(r)
                final_intensities.append(i)
            else:
                # 범위 밖의 데이터는 무한대로 설정 (유효하지 않음)
                final_ranges.append(float('inf'))
                final_intensities.append(0.0)
        
        filtered_msg.ranges = final_ranges
        filtered_msg.intensities = final_intensities
        
        # 발행
        self.publisher.publish(filtered_msg)
        
        # 첫 번째 메시지에만 로그 출력
        if not hasattr(self, '_first_message_logged'):
            self._first_message_logged = True
            valid_points = sum(1 for r in final_ranges if r != float('inf'))
            self.get_logger().info(
                f'필터링된 포인트 수: {valid_points}/{len(filtered_msg.ranges)} '
                f'(각도: {math.degrees(filtered_msg.angle_min):.1f}° ~ '
                f'{math.degrees(filtered_msg.angle_max):.1f}°, '
                f'거리: {self.target_range_min}m ~ {self.target_range_max}m)'
            )


def main(args=None):
    rclpy.init(args=args)
    
    angle_filter_node = AngleFilterNode()
    
    try:
        rclpy.spin(angle_filter_node)
    except KeyboardInterrupt:
        pass
    finally:
        angle_filter_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

