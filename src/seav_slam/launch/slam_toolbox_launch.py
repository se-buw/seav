from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                '/home/pi/ros_we/ros2_ws_fazil/ros2_ws/src/seav_slam/mapper_params_online_async.yaml'
            ],
            output='screen',
        )
    ])
