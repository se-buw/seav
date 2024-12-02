from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration  # Import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare a launch argument for the robot description
        DeclareLaunchArgument(
            "robot_description",
            default_value="/home/user/workspace_world/ws_1/src/ros2_control_demos/example_11/description/urdf/carlikebot.urdf.xacro",
            description="Robot description file"
        ),
        # Define the ROS 2 control node
        Node(
            package="controller_manager",  # 'ros2_control_node' belongs to 'controller_manager'
            executable="ros2_control_node",
            output="both",
            parameters=[
                {"robot_description": LaunchConfiguration("robot_description")}
            ],
        ),
    ])
