#!/usr/bin/env python3
"""
각도 필터링이 적용된 SLLIDAR A1 런치 파일 (RViz 포함)
원하는 각도 범위만 시각화할 수 있습니다.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # SLLIDAR 기본 파라미터
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    
    # 각도 필터링 파라미터 (도 단위)
    angle_min_deg = LaunchConfiguration('angle_min_deg', default='90.0')
    angle_max_deg = LaunchConfiguration('angle_max_deg', default='270.0')
    
    # 거리 필터링 파라미터 (미터 단위)
    range_min = LaunchConfiguration('range_min', default='0.0')
    range_max = LaunchConfiguration('range_max', default='1.0')
    
    # RViz 설정 (필터링된 데이터용)
    rviz_config_dir = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'rviz',
        'sllidar_ros2_filtered.rviz')
    
    return LaunchDescription([
        # 기본 SLLIDAR 파라미터
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='라이다 채널 타입'),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='USB 시리얼 포트'),
        
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='시리얼 baudrate'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='프레임 ID'),
        
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='스캔 데이터 반전 여부'),
        
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='각도 보정 활성화 여부'),
        
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='스캔 모드'),
        
        # 각도 필터링 파라미터
        DeclareLaunchArgument(
            'angle_min_deg',
            default_value=angle_min_deg,
            description='최소 각도 (도 단위, 예: 30)'),
        
        DeclareLaunchArgument(
            'angle_max_deg',
            default_value=angle_max_deg,
            description='최대 각도 (도 단위, 예: 150)'),
        
        # 거리 필터링 파라미터
        DeclareLaunchArgument(
            'range_min',
            default_value=range_min,
            description='최소 거리 (미터 단위, 예: 0.5)'),
        
        DeclareLaunchArgument(
            'range_max',
            default_value=range_max,
            description='최대 거리 (미터 단위, 예: 5.0)'),
        
        # SLLIDAR 노드
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode
            }],
            output='screen'),
        
        # 각도 및 거리 필터링 노드
        Node(
            package='sllidar_ros2',
            executable='angle_filter_node.py',
            name='angle_filter_node',
            parameters=[{
                'angle_min_deg': angle_min_deg,
                'angle_max_deg': angle_max_deg,
                'range_min': range_min,
                'range_max': range_max
            }],
            output='screen'),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
    ])

