## Useful ROS 2 Command Line Interface (CLI) Commands

The ROS 2 Command Line Interface (CLI) provides a powerful set of tools for managing and interacting with ROS 2 applications. Below is a list of some of the most useful commands along with brief descriptions.

### General Command Structure
All ROS 2 CLI commands start with `ros2`, followed by a command and optional arguments. The general syntax is:

```sh
ros2 <command> [<sub-command>] [<options>]
```

### Common Commands

- **Run a Node**: Starts a specific executable from a package.
  ```sh
  ros2 run <package_name> <executable_name>
  ```

- **List Nodes**: Displays all active nodes in the ROS graph.
  ```sh
  ros2 node list
  ```

- **Node Information**: Provides details about a specific node.
  ```sh
  ros2 node info <node_name>
  ```

- **List Topics**: Shows all available topics.
  ```sh
  ros2 topic list
  ```

- **Echo Topic Messages**: Subscribes to a topic and prints the messages being published.
  ```sh
  ros2 topic echo <topic_name>
  ```

- **Publish to a Topic**: Publishes a message to a specified topic.
  ```sh
  ros2 topic pub <topic_name> <msg_type> "<msg_data>"
  ```

- **Get Topic Information**: Displays information about a specific topic.
  ```sh
  ros2 topic info <topic_name>
  ```

- **Service List**: Lists all available services.
  ```sh
  ros2 service list
  ```

- **Service Information**: Provides details about a specific service.
  ```sh
  ros2 service info <service_name>
  ```

### Additional Commands

- **Launch Files**: Runs a launch file to start multiple nodes and configurations.
  ```sh
  ros2 launch <package_name> <launch_file>
  ```

- **Parameter Management**: Lists all parameters for nodes or sets parameters for specific nodes.
  
   - List parameters:
   ```sh
   ros2 param list
   ```
  
   - Set a parameter:
   ```sh
   ros2 param set <node_name> <parameter_name> <value>
   ```

### Debugging Commands

- **Check ROS Setup**: Checks the setup for any potential issues.
  ```sh
  ros2 doctor
  ```

- **Show Interface Information**: Displays information about message types or services.
```sh
ros2 interface show <interface_type>
```

### Help and Documentation

For detailed usage and options for any command, append `--help` or `-h` to the command:

```sh
ros2 <command> --help
```

These commands form the backbone of working with ROS 2 through the command line, enabling developers to efficiently manage nodes, topics, and services while debugging their applications.

Citations:
[1] https://osrf.github.io/ros2multirobotbook/ros2_cli.html
[2] https://ubuntu.com/blog/ros-2-command-line-interface
[3] https://roboticsbackend.com/ros2-run-and-ros2-node-start-debug-ros2-nodes/
[4] https://roboticsbackend.com/ros2-topic-cmd-line-tool-debug-ros2-topics-from-the-terminal/
[5] https://iotdesignshop.com/ros2-research-homepage/ros-2-command-line-cheatsheet/
[6] https://docs.ros.org/en/foxy/Concepts/About-Command-Line-Tools.html