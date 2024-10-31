## ROS2 Overview and Nodes

1. **ROS2 Preinstalled Nodes**  
   - ROS2 comes with preinstalled nodes, and users can also create their own.
   - Example nodes include:
     - **talker** and **listener**.
   - To run these examples:  
     ```bash
     ros2 run demo_nodes_cpp talker
     ros2 run demo_nodes_cpp listener
     ```

2. **rqt_graph**  
   - Command: `rqt_graph`
   - Displays active nodes and their communication links, providing a visual of node interactions.

3. **Node Definition**  
   - A **node** is essentially a C++ or Python program.
   - Nodes are connected through ROS2 communication, enabling them to use ROS2 functionalities and communicate with other nodes.

---

## 2D Experimentation with ROS2

- **Package Name**: `turtlesim`
- **Node Name**: `turtlesim_node`

---

## Creating and Setting Up a ROS2 Workspace

1. **Purpose of ROS2 Workspace**  
   - When sharing ROS2 projects, share the entire **ROS2 workspace**, as it contains all relevant nodes and files.

2. **Building Tool**  
   - Use **colcon** as the build tool for creating a workspace:
     ```bash
     colcon build
     ```

3. **Creating a Workspace**  
   - Creating a workspace is as simple as creating a folder, which will overlay the ROS2 global installation.
   - After adding nodes/programs, run `colcon build` to compile the local ROS2 workspace.
   - The workspace can now be used for developing additional ROS2 applications.

---

## Creating a ROS2 Python Package

- **Goal**: Create a package that contains nodes to control the direction of the turtle in `turtlesim`.
- **Naming Convention**: `robotname_function`
- **Node Language Options**: Nodes can be created in C++ or Python.

## ROS2 Development Tools and Configuration

1. **colcon**  
   - A build tool for compiling and managing ROS2 workspaces.

2. **ament**  
   - The build system used by ROS2, working alongside `colcon`.

3. **rclpy**  
   - A Python library for ROS2, enabling Python-based node creation and ROS2 functionalities.

4. **Autocompletion in ROS2**  
   - ROS2 supports autocompletion, enhancing productivity by suggesting commands and code snippets.

5. **Configuring ROS2 and VSCode**  
   - Proper configuration is necessary for seamless development in VSCode, including enabling autocompletion and other ROS2-specific features.

---

## ROS2 Package Structure

1. **package.xml**  
   - The main configuration file in each ROS2 package.
   - Contains metadata such as maintainer details and package dependencies.

2. **setup.cfg**  
   - Specifies where to install nodes within the ROS2 environment.

3. **setup.py**  
   - The setup script used for building and installing Python packages in ROS2.
## Handling colcon Build Warnings

- **SetuptoolsDeprecationWarning**:  
  If `colcon build` shows a `setuptoolsDeprecationWarning`, downgrade `setuptools` to avoid compatibility issues.

---

## How `colcon build` Works

- **Function**: `colcon build` compiles and builds ROS2 packages within a workspace.
- **Process**: It reads the package configuration files (`package.xml`, `setup.py`, etc.), resolves dependencies, and compiles the code as specified in the build configuration.
- **Output Location**: Builds are typically stored in a directory within the ROS2 workspace, ready for running and testing.

---

## Creating a ROS2 Node in Python with OOP

1. **Hello World Node**  
   - To create a new Python node file:
     ```bash
     touch my_first_node.py
     chmod +x my_first_node.py
     ```
   - A "Hello World" node in ROS2 can be written using Python and the Object-Oriented Programming (OOP) paradigm.

2. **Running the Node**  
   - For now, you can run the node directly from the command line.
   - To enable `ros2 run` functionality, install the node as an executable:
     - Define three names in `setup.py`:
       - **Node Name**
       - **File Name**
       - **Executable File Name**

3. **Avoiding Rebuilds with Symlink Install**  
   - By using `symlink install`, you can avoid rebuilding the package every time a change is made to the Python file, allowing direct execution of the updated code.## What is ROS2?

- **ROS2** (Robot Operating System 2) is a framework designed to develop and manage robotic systems.
- It provides tools and libraries to enable nodes to communicate with each other in a networked, modular manner.
- Key features include support for real-time computing, multi-platform compatibility, and enhanced security.

---

## Nodes in ROS2

- **Nodes**: The fundamental building blocks of a ROS2 system, representing independent processes (often as C++ or Python programs) that can perform specific tasks.
- **Node Communication**: Nodes communicate with each other using topics, services, and actions.
- **Creating a Node**:
  - In Python, you can create a node file and make it executable:
    ```bash
    touch my_first_node.py
    chmod +x my_first_node.py
    ```
  - To allow the node to be run with `ros2 run`, set it up in `setup.py` and build it with `colcon build`.

---

## Chatter: Communication Between Nodes

- **Chatter**: A topic used as a communication channel between nodes.
  - **Talker Node**: Publishes messages on the `chatter` topic.
  - **Listener Node**: Subscribes to the `chatter` topic to receive messages from `talker`.
  - **Data Flow**: `talker` sends data through `chatter` to `listener`.

### Command Line Tools for Node Communication

1. **Running Nodes**  
   - Start the talker node:
     ```bash
     ros2 run demo_nodes_cpp talker
     ```
   - Start the listener node:
     ```bash
     ros2 run demo_nodes_cpp listener
     ```

2. **Introspecting Topics and Nodes**  
   - **List Active Nodes**:
     ```bash
     ros2 node list
     ```
   - **List Active Topics**:
     ```bash
     ros2 topic list
     ```
   - **Inspect Topic Information**:
     ```bash
     ros2 topic info /chatter
     ```
   - **Echo Topic Messages**:
     ```bash
     ros2 topic echo /chatter
     ```

3. **Graph Visualization**  
   - Use `rqt_graph` to view active nodes and their communication links visually:
     ```bash
     rqt_graph
     ```

By utilizing these tools and commands, you can inspect, debug, and understand the communication between nodes in ROS2.## ROS2 Key Concepts

- **Publisher**: A node component responsible for sending (or publishing) messages to a topic.
  - Example: A `talker` node acts as a publisher on a topic.

- **Talker**: A specific node that publishes data.
  - Often used to demonstrate basic message publishing on a topic (e.g., `chatter`).

- **Chatter**: A **topic** used as a communication channel between nodes.
  - `talker` publishes data to `chatter`, which can be read by other nodes that subscribe to it.

- **Receiver**: Any node or entity that receives data.
  - In the `talker` and `listener` example, the `listener` is a receiver node.

- **Subscriber**: A node component that listens to (or subscribes) a topic to receive messages.
  - Example: The `listener` node acts as a subscriber on the `chatter` topic.

---

### ROS2 Communication Example

1. **Publisher Node** (`talker`):
   - Publishes messages on the `chatter` topic.

2. **Subscriber Node** (`listener`):
   - Subscribes to the `chatter` topic to receive messages from `talker`.

3. **Communication Flow**:
   - `talker` (publisher) -> `chatter` (topic) -> `listener` (subscriber/receiver)

### Useful Commands

- **Run Publisher Node (talker)**:
  ```bash
  ros2 run demo_nodes_cpp talker


ros2 run demo_nodes_cpp listener

ros2 topic list

ros2 topic echo /chatter


## Running Turtlesim Node with Commands (3 Tabs Setup)

### Tab 1: Start Turtlesim Node

- **Command**: 
  ```bash
  ros2 run turtlesim turtlesim_node
---------------

%% %% %%


---

---

---
_________________________
