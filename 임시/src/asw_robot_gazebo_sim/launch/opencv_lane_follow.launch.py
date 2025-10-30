from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    OpenCV 기반 차선 추종 (YOLO 불필요!)
    - 빠르고 가벼움
    - Hough Line Transform
    - PID 제어
    """
    
    # 패키지 경로
    asw_robot_pkg = get_package_share_directory('asw_robot')
    
    return LaunchDescription([
        # 1. Gazebo + RViz2 시뮬레이터
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(asw_robot_pkg, 'launch', 'gazebo_rviz_display.launch.py')
            ])
        ),
        
        # 2. OpenCV 차선 추종 (YOLO 없음)
        Node(
            package='asw_robot',
            executable='opencv_lane_follow',
            name='opencv_lane_follower',
            output='screen'
        )
    ])

