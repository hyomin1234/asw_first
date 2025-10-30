import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock if true"
    )
    
    package_name = 'asw_robot'
    pkg_share = get_package_share_directory(package_name)
    
    urdf_path = os.path.join(pkg_share, "urdf", "asw_robot_sim.xacro")
    gazebo_world_path = os.path.join(pkg_share, "worlds", "asw_robot_stadium_3x.world")
    rviz_config_path = os.path.join(pkg_share, "rviz", "asw_robot_gazebo.rviz")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": xacro.process_file(urdf_path).toxml(),
            }
        ],
    )

    # Model path for aa10_ground texture
    models_path = os.path.join(pkg_share, "models")
    
    # Set Gazebo model path
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path
    )
    
    # Gazebo launch with model path
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
        launch_arguments={
            "world": gazebo_world_path,
            "extra_gazebo_args": f"-s libgazebo_ros_init.so -s libgazebo_ros_factory.so"
        }.items(),
    )

    gazebo_spawn_entity_node = Node(
        name="urdf_spawner",
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "asw_robot", 
                  "-x", "-10.7", "-y", "-12.5", "-z", "0.1", "-Y", "1.5708"],
        output="screen",
    )

    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            set_gazebo_model_path,
            gazebo_node,
            gazebo_spawn_entity_node,
            robot_state_publisher_node,
            rviz2_node,
        ]
    )