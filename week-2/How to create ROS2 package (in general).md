Creating a ROS2 package involves a few steps. I’ll walk you through it with detailed commands and code for a Python-based ROS2 package. In this example, we’ll create a package named `my_robot`.

### Step 1: **Set Up Your ROS2 Workspace (If Not Done Already)**
   - Create the workspace and `src` directory:
     ```bash
     mkdir -p ~/ros2_ws/src
     cd ~/ros2_ws
     ```
   - Build the workspace:
     ```bash
     colcon build
     ```
   - Source the workspace:
     ```bash
     source install/setup.bash
     ```

### Step 2: **Create the ROS2 Package**
   - Go to the `src` directory in your workspace:
     ```bash
     cd ~/ros2_ws/src
     ```
   - Use the `ros2 pkg create` command to create a new package. For a Python package, use the `--build-type ament_python` flag.
     ```bash
     ros2 pkg create my_robot --build-type ament_python --dependencies rclpy std_msgs
     ```
   - This command creates a directory `my_robot` with the following basic structure:
     ```
     my_robot/
     ├── package.xml
     ├── setup.py
     ├── resource/
     │   └── my_robot
     └── my_robot/
         └── __init__.py
     ```

### Step 3: **Understand the Package Structure**
   - `package.xml`: Contains metadata for your package, including dependencies and version.
   - `setup.py`: Configures the package as a Python module.
   - `resource/`: Contains a file named after the package (required by ROS2 for Python packages).
   - `my_robot/`: This directory is where you’ll put your Python nodes. It contains an `__init__.py` file, marking it as a Python module.

### Step 4: **Write a ROS2 Node in Python**
   - In the `my_robot` directory (inside the `my_robot` package), create a Python script for your node. Here’s an example node that publishes a message to a topic:
     ```bash
     cd ~/ros2_ws/src/my_robot/my_robot
     touch my_node.py
     ```
   - Open `my_node.py` and add the following code:

     ```python
     # my_node.py
     import rclpy
     from rclpy.node import Node
     from std_msgs.msg import String

     class MyRobotNode(Node):
         def __init__(self):
             super().__init__('my_robot_node')
             self.publisher_ = self.create_publisher(String, 'greetings', 10)
             timer_period = 1.0  # seconds
             self.timer = self.create_timer(timer_period, self.timer_callback)
             self.get_logger().info("MyRobotNode has been started.")

         def timer_callback(self):
             msg = String()
             msg.data = 'Hello from MyRobotNode!'
             self.publisher_.publish(msg)
             self.get_logger().info(f'Publishing: "{msg.data}"')

     def main(args=None):
         rclpy.init(args=args)
         node = MyRobotNode()
         try:
             rclpy.spin(node)
         except KeyboardInterrupt:
             pass
         finally:
             node.destroy_node()
             rclpy.shutdown()

     if __name__ == '__main__':
         main()
     ```

### Step 5: **Modify `setup.py` to Make Your Node Executable**
   - Open the `setup.py` file and modify the `entry_points` section to add your node:
     ```python
     from setuptools import setup

     package_name = 'my_robot'

     setup(
         name=package_name,
         version='0.0.0',
         packages=[package_name],
         data_files=[
             ('share/ament_index/resource_index/packages',
              ['resource/' + package_name]),
             ('share/' + package_name, ['package.xml']),
         ],
         install_requires=['setuptools'],
         zip_safe=True,
         maintainer='your_name',
         maintainer_email='your_email@example.com',
         description='Example ROS2 package',
         license='License',
         tests_require=['pytest'],
         entry_points={
             'console_scripts': [
                 'my_node = my_robot.my_node:main',
             ],
         },
     )
     ```

### Step 6: **Build the Package**
   - Return to the root of your workspace and build your package with `colcon`:
     ```bash
     cd ~/ros2_ws
     colcon build
     ```

### Step 7: **Source the Workspace**
   - After building, source your workspace to recognize the new package:
     ```bash
     source install/setup.bash
     ```

### Step 8: **Run Your Node**
   - You can now run your node using `ros2 run`:
     ```bash
     ros2 run my_robot my_node
     ```

   - This should start the node, and you’ll see messages being published to the `greetings` topic.

### Full Directory Structure Recap
After completing all the steps, your directory structure should look like this:

```
ros2_ws/
├── src/
│   └── my_robot/
│       ├── package.xml
│       ├── setup.py
│       ├── resource/
│       │   └── my_robot
│       └── my_robot/
│           ├── __init__.py
│           └── my_node.py
└── install/ (created after building)
```

### Example Workflow Recap for ROS2 Package Creation

1. Create the package:
   ```bash
   ros2 pkg create my_robot --build-type ament_python --dependencies rclpy std_msgs
   ```
2. Write your node script in `my_node.py`, make it executable, and update `setup.py` with the entry point.
3. Build the workspace:
   ```bash
   colcon build
   ```
4. Source the workspace:
   ```bash
   source install/setup.bash
   ```
5. Run the node:
   ```bash
   ros2 run my_robot my_node
   ```

This completes your ROS2 package creation with a basic publisher node! You can expand this structure by adding more nodes, services, or custom messages as needed.