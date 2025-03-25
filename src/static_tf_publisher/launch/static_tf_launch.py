from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform for IMU
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_publisher',
            arguments=['0.202', '0.035', '0.060', '0', '0', '0', '1', 'base_link', 'imu_link'],
            output='screen'
        ),
        # Static transform for Lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_publisher',
            arguments=['-0.03602', '0', '0.077736', '0', '0', '0', '1', 'base_link', 'laser'],
            output='screen'
        ),
    ])
