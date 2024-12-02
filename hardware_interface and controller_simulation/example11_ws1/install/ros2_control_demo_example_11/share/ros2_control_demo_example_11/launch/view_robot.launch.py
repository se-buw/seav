# Import necessary modules for launch description and ROS2 nodes
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    # These are inputs to the launch file that allow customization when the launch file is executed
    declared_arguments = []

    # Argument: description_package
    # Specifies the package containing the URDF/XACRO description files
    # Default value points to `ros2_control_demo_description`, but can be overridden for custom setups
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ros2_control_demo_description",
            description="Description package with robot URDF/Xacro files. Usually the argument "
                        "is not set, it enables use of a custom description.",
        )
    )

    # Argument: description_file
    # Specifies the main URDF/XACRO file for the robot
    # Default value is set to `carlikebot.urdf.xacro`, which can be replaced with your robot's file
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="carlikebot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    # Argument: gui
    # Controls whether RViz2 and Joint State Publisher GUI should be launched
    # Default is `true`, but can be set to `false` for headless operation
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 and Joint State Publisher gui automatically "
                        "with this launch file.",
        )
    )

    # Argument: prefix
    # Allows setting a prefix for joint names, useful for multi-robot setups
    # Default is an empty string. If changed, ensure the controller configuration matches the new prefix
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. "
                        "If changed, then also joint names in the controllers' configuration "
                        "have to be updated.",
        )
    )

    # Initialize Arguments
    # These variables retrieve the values of the launch arguments provided by the user
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    # This command processes the specified XACRO file into a URDF string at runtime
    # Update `ros2_control_demo_example_11` and the XACRO file path as needed for your custom robot
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # Locate the `xacro` executable
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_control_demo_example_11"), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # RViz configuration file
    # Specify the path to the RViz configuration file
    # Update the path if your RViz configuration is located elsewhere
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "carlikebot/rviz", "carlikebot_view.rviz"]
    )

    # Nodes to be launched
    # Joint State Publisher GUI node
    # Visualizes joint states; requires GUI support
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),  # Only launched if GUI is enabled
    )

    # Robot State Publisher node
    # Publishes the robot's URDF to the `/robot_description` topic
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",  # Logs to console and files
        parameters=[robot_description],  # Provide the robot description
    )

    # RViz2 visualization node
    # Launches RViz2 with the specified configuration file
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],  # Load RViz configuration
        condition=IfCondition(gui),  # Only launched if GUI is enabled
    )

    # Combine all nodes to be launched
    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    # Return the launch description
    return LaunchDescription(declared_arguments + nodes)



'''
Given your project and setup, here are some nodes you should consider adding to the launch file to support your autonomous vehicle project effectively:

---

### **1. Controller Manager**
To manage the ROS2 controllers for motor control and steering:
```python
controller_manager_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    output="both",
    parameters=[
        robot_description,
        {"use_sim_time": False},  # Set to True if running in simulation
    ],
)
```

---

### **2. Joint State Broadcaster**
To broadcast joint states for visualization and simulation:
```python
joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"],
    output="log",
)
```

---

### **3. Ackermann Steering Controller**
If you’re using `ackermann_steering_controller` for your vehicle:
```python
ackermann_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["ackermann_steering_controller"],
    output="log",
)
```

---

### **4. Hardware Interface Node**
Since you plan to integrate with real hardware:
- Include the hardware interface for your motors and steering system:
```python
hardware_interface_node = Node(
    package="your_hardware_package",
    executable="hardware_interface",
    name="hardware_interface_node",
    output="both",
    parameters=[robot_description],
)
```
Replace `"your_hardware_package"` and `"hardware_interface"` with your custom package and executable for hardware communication.

---

### **5. LIDAR Node**
For RPLIDAR A1 integration, include the relevant driver node:
```python
lidar_node = Node(
    package="rplidar_ros",
    executable="rplidar_node",
    name="rplidar_node",
    output="both",
    parameters=[{"serial_port": "/dev/ttyUSB0", "frame_id": "laser_frame"}],
)
```

---

### **6. IMU Node**
To integrate the MPU6050 IMU sensor:
```python
imu_node = Node(
    package="mpu6050_driver",
    executable="mpu6050_node",
    name="imu_node",
    output="both",
    parameters=[{"i2c_address": 0x68, "frame_id": "imu_frame"}],
)
```

---

### **7. Nav2 Nodes**
If you plan to implement navigation in future iterations:
```python
nav2_bringup_node = Node(
    package="nav2_bringup",
    executable="bringup_launch",
    output="both",
    parameters=[{"use_sim_time": False}],
)
```

---

### **8. Custom Nodes for Additional Sensors**
For sensors like a color sensor, ultrasonic sensors, or encoders:
- Add respective nodes or custom implementations. Example for a color sensor:
```python
color_sensor_node = Node(
    package="color_sensor_driver",
    executable="color_sensor_node",
    name="color_sensor",
    output="log",
)
```

---

### **9. Diagnostic Node**
To monitor system health and diagnose potential issues:
```python
diagnostic_node = Node(
    package="diagnostic_aggregator",
    executable="aggregator_node",
    name="diagnostics",
    output="both",
    parameters=[{"analyzers": "/path/to/diagnostic_config.yaml"}],
)
```

---

### **Integration into Your Launch File**
Add these nodes to the `nodes` list in your launch file:
```python
nodes += [
    controller_manager_node,
    joint_state_broadcaster_spawner,
    ackermann_controller_spawner,
    lidar_node,
    imu_node,
    diagnostic_node,
]
```

---

### **Prioritize Based on Milestones**
1. **Basic Motion:**
   - Controller Manager
   - Joint State Broadcaster
   - Hardware Interface Node

2. **Sensor Feedback:**
   - LIDAR Node
   - IMU Node
   - Color Sensor (if implemented)

3. **Autonomous Navigation:**
   - Nav2 Nodes
   - SLAM Nodes (e.g., Cartographer or SLAM Toolbox)

Add nodes incrementally to avoid overwhelming the system and to ensure proper debugging at each step.

Summary of Changes
Node	Effect on System
Controller Manager	Manages controllers, enabling motor and steering commands.
Joint State Broadcaster	Publishes joint states for visualization and integration.
Ackermann Controller	Converts velocity commands into steering and motor movements.
Hardware Interface	Interfaces ROS2 commands with physical hardware.
LIDAR Node	Adds LIDAR data for mapping, SLAM, and obstacle avoidance.
IMU Node	Adds motion and orientation data for odometry and stabilization.
Nav2 Nodes	Enables autonomous navigation (path planning and obstacle avoidance).
Custom Sensor Nodes	Adds capabilities from extra sensors for improved sensing and feedback.
Diagnostic Node	Provides real-time monitoring of the robot's health and performance.
'''
