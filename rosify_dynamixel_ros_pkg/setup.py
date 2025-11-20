from setuptools import setup
import os

package_name = 'rosify_dynamixel_ros_pkg'

# Les inn requirements.txt hvis den finnes
requirements = []
if os.path.exists('requirements.txt'):
    with open('requirements.txt') as f:
        requirements = f.read().splitlines()

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=requirements + ['setuptools'],
    zip_safe=True,
    maintainer='rocotics',
    maintainer_email='rocotics@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # format:  <kommando-navn> = <pakke>.<filnavn>:main
            'dxl_profile_setter = rosify_dynamixel_ros_pkg.dxl_profile_setter:main',
            'send_single_joint_cmd = rosify_dynamixel_ros_pkg.send_single_joint_cmd:main',
            'visualizer_publisher = rosify_dynamixel_ros_pkg.visualizer_publisher:main',
            'visualizer_to_robot = rosify_dynamixel_ros_pkg.visualizer_to_robot:main',
            'testing_av_torque = rosify_dynamixel_ros_pkg.testing_av_torque:main',
            'Punkt_test = rosify_dynamixel_ros_pkg.Punkt_test:main',
            'visualizer_visualizer = rosify_dynamixel_ros_pkg.visualizer_visualizer:main',
            'ball_tracker = rosify_dynamixel_ros_pkg.ball_tracker:main',
            'koordinat_transformer = rosify_dynamixel_ros_pkg.koordinat_transformer:main',
            'visualize_goal_points = rosify_dynamixel_ros_pkg.visualize_goal_points:main',
            'manual_goal_points = rosify_dynamixel_ros_pkg.manual_goal_points:main',
            'manual_goal_pointsV2 = rosify_dynamixel_ros_pkg.manual_goal_pointsV2:main',
        ],

    },
)