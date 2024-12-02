##### Simple personal understanding
A **package** in ROS2 is like a folder that holds all the tools and files needed to make one part of a robot work. Imagine it as a "robot app" that does a specific job.

For example:
- One package might be for making the robot’s wheels move.
- Another package might be for helping the robot "see" with a sensor like a camera.

Each package has:
- **Instructions** (code) for what it does.
- **Information** on how to talk to other robot parts.
- **Setup files** to make it easy to turn on and use.

So, a package is like a mini-toolbox that helps the robot do one thing well, and you can mix different packages together to make the robot do more things!

------------------------

In ROS2, a *package* is a structured, self-contained collection of resources<mark style="background: #CACFD9A6;"> that provides specific functionalities for robotic applications</mark>. A ROS2 package is the basic unit for organizing, sharing, and reusing code. Each package <mark style="background: #FFF3A3A6;">can contain nodes (programs), configuration files, launch files, custom messages, and other resources related to a particular functionality or task</mark>. Packages are designed to be modular, meaning they can be individually developed, tested, and reused across different projects.

### Key Components of a ROS2 Package
Here are the main components typically found in a ROS2 package:

1. **Nodes**: Executable programs (scripts in Python or compiled binaries in C++) that perform tasks, such as reading sensor data, controlling motors, or running algorithms.
   
2. **Message and Service Definitions**: Custom message types (for exchanging structured data between nodes) and services (for request-response interactions) can be defined within a package. These files are placed in the `msg` and `srv` folders, respectively.

3. **Launch Files**: Files that specify which nodes to start, along with their parameters and configurations. In ROS2, launch files are often written in Python (`.launch.py`), making it easy to start multiple nodes with specific settings at once.

4. **Configuration Files**: YAML or other configuration files used to set parameters for nodes, such as sensor calibration values, motor speeds, or mapping settings.

5. **URDF/Xacro Files**: Description files for robot models, typically in URDF (Unified Robot Description Format) or Xacro format, define the physical structure of a robot. These files are useful for visualization and simulation in tools like RViz and Gazebo.

6. **Package Metadata Files**:
   - **`package.xml`**: Contains metadata like the package name, version, description, author, dependencies, and license. It’s crucial for package dependency management.
   - **`setup.py`** (for Python packages): Contains setup information for Python nodes, including entry points for executable scripts.
   - **`CMakeLists.txt`** (for C++ packages): Specifies build instructions for C++ nodes, including dependencies and compilation settings.

### Structure of a ROS2 Package
When you create a ROS2 package, it follows a standard structure. For example, a package called `my_robot` might look like this:

```
my_robot/
├── package.xml           # Package metadata (name, version, dependencies)
├── CMakeLists.txt        # Build instructions (for C++ packages)
├── setup.py              # Python setup file (for Python packages)
├── launch/               # Launch files to start nodes
│   └── my_robot_launch.py
├── config/               # Configuration files
│   └── my_robot_config.yaml
├── urdf/                 # Robot model description files
│   └── my_robot.urdf.xacro
├── msg/                  # Custom message definitions
│   └── CustomMessage.msg
├── srv/                  # Custom service definitions
│   └── CustomService.srv
└── my_robot/             # Package source code
    └── my_robot_node.py  # Example Python node
```

### Benefits of Using Packages in ROS2
- **Modularity**: Packages allow you to organize different parts of a robot’s functionality into independent, reusable modules.
- **Reusability**: Since each package is self-contained, it can be reused across multiple projects without modification.
- **Dependency Management**: Packages list their dependencies in `package.xml`, so ROS2 can install and manage required libraries and other packages.
- **Collaboration**: Packages provide a structured way for teams to work on different parts of a project, allowing members to work independently and integrate code easily.
- **Scalability**: As projects grow, you can add or remove packages without affecting other parts of the system.

### Types of ROS2 Packages
- **Application Packages**: Contain the main code to perform specific tasks, such as sensor data processing, motor control, or path planning.
- **Library Packages**: Provide shared libraries or tools for other packages (e.g., `rclcpp`, `rclpy`).
- **Message Packages**: Define custom messages and services for specific data structures or interactions in a system.
- **Launch and Configuration Packages**: Contain launch files and configurations for starting multiple packages with specific settings, often used in robot bring-up.

### Example: Creating a Simple ROS2 Package
To create a package named `my_robot_package` with dependencies on `rclpy` and `std_msgs`, you would use:

```bash
cd ~/ros2_ws/src
ros2 pkg create my_robot_package --build-type ament_python --dependencies rclpy std_msgs
```

This command creates a package folder with basic files (`package.xml`, `setup.py`, and a folder structure) to get you started.

In summary, ROS2 packages help organize and modularize robotic software, making it easier to manage, develop, and share complex robotic systems.