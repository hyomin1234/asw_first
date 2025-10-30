from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    OpenCV + Cubic Spline 경로 추정 차선 추종
    - OpenCV로 차선 검출
    - Cubic Spline으로 부드러운 경로 생성
    - path_planner_node 방식 적용
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
        
        # 2. OpenCV + Spline 차선 추종
        Node(
            package='asw_robot',
            executable='opencv_spline_lane_follow',
            name='opencv_spline_lane_follower',
            output='screen'
        )
    ])

