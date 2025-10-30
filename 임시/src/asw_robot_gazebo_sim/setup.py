from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'asw_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'models/aa10_ground'), glob('models/aa10_ground/model.*')),
        (os.path.join('share', package_name, 'models/aa10_ground/materials/scripts'), glob('models/aa10_ground/materials/scripts/*')),
        (os.path.join('share', package_name, 'models/aa10_ground/materials/textures'), glob('models/aa10_ground/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='goblue',
    maintainer_email='goblue@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rotate_wheels=asw_robot.wheel_control:main",
            "robot_teleop=asw_robot.robot_teleop:main",
            "auto_drive=asw_robot.auto_drive:main",
            "smart_drive=asw_robot.smart_drive:main",
            "camera_follow=asw_robot.camera_follow:main",
            "opencv_lane_follow=asw_robot.opencv_lane_follow:main",
            "opencv_spline_lane_follow=asw_robot.opencv_spline_lane_follow:main",
        ],
    },
)
