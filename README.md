

# SEAV ROS 2 Workspace

Software Engineering for Autonomous Vehicles.

---

## 🧭 Diagrams and Their Purpose

| Image | Description |
|-------|-------------|
| ![TF Tree](tf_tree.png) | **TF Tree** showing static transforms between `base_link`, `imu`, and `lidar`. |
| ![Nav2 Nodes & Topics](nav2_nodes_topics.png) | Overview of important nodes and topics in the Nav2 stack. |
| ![ROS 2 Communication Flow](ros2_communication_flow.png) | Shows how different ROS 2 nodes communicate through topics and services. |
| ![SLAM System Overview](slam.drawio.png) | Visualizes SLAM pipeline used in the simulation world. |
| ![High-Level Architecture](high%20level%20architecture.png) | Block-level overview of how different packages work together. |
| ![Circuit Diagram](Circuit%20Diagram.png) | Hardware setup for connecting the microcontroller and sensors. |
| ![IMU Before Calibration](imu_before_calib.png) | Raw IMU data before applying calibration filters. |
| ![AMCL Overview](amcl.drawio.png) | Adaptive Monte Carlo Localization node flow and topic structure. |
| ![Slam Without IMU](slam_before_imu.jpeg) | SLAM performance before IMU fusion was introduced. |
| ![Odom Publisher Diagram](odom_publisher.drawio.png) | Internal flow of the odometry calculation node. |
| ![Layers and Workspace](layers%20and%20workspace.drawio.png) | Stack diagram showing physical, simulation, and planning layers. |
| ![Untitled Diagram](Untitled%20Diagram(1).drawio) | Draw.io file for making the diagrams

## 🗺️ TF Tree

![TF Tree](docs/images/tf_tree.png)

---

## 📚 Table of Contents

- [TF Tree](#tf-tree)
- [Package Overview](#package-overview)
- [How to Build](#how-to-build)
- [How to Run Packages](#how-to-run-packages)
- [Folder Structure](#folder-structure)
- [Requirements](#requirements)

---

## 📦 Package Overview

### 1. **seav_test_sensors**
Collects and publishes data from IMU, distance sensor, and odometry.

- **Nodes:** `imu_serial_publisher`, `distance_publisher`, `odom_publisher`
- **Publishes:** `/imu`, `/distance`, `/odom`
- **Subscribes:** `/imu`, `/distance`
- **Dependencies:** `rclpy`, `robot_localization`, `tf2_ros`
- **Launch File:** `ekf.launch.py`
- **Status:** ✅ Built

---

### 2. **rplidar_ros**
Driver package for RPLIDAR A1.

- **Publishes:** `/parameter_events`, `/rosout`
- **Launch File:** `ekf.launch.py`
- **Status:** ✅ Built

---

### 3. **my_nav2_pkg**
Configurations and launch files for ROS 2 Nav2 stack.

- **Nodes:** `robot_state_publisher`, `map_server`, `amcl`, etc.
- **Publishes:** `/tf`, `/map`, `/amcl_pose`, `/cmd_vel`
- **Subscribes:** `/scan`, `/goal_pose`, `/map`
- **Launch Files:** `nav2.launch.py`, `bringup.launch.py`, etc.
- **Status:** ✅ Built

---

### 4. **my_robot_controller**
Forwards velocity commands from `/cmd_vel` to Arduino.

- **Publishes/Subscribes:** `/cmd_vel`, `/rosout`
- **Status:** ✅ Built

---

### 5. **static_tf_publisher**
Publishes static TFs between `base_link`, `imu`, and `lidar`.

- **Node:** `static_transform_publisher`
- **Launch File:** `static_tf_publisher.launch.py`
- **Status:** ✅ Built

---

### 6. **seav_description**
Contains robot URDF/Xacro, meshes, and worlds for Gazebo simulation.

- **URDF:** Modular robot description using `xacro` files
- **Mesh:** `lidar.dae`
- **Launch File:** `bot.launch.py`
- **Worlds:** `empty.world`
- **Status:** ✅ Built

---

### 7. **seav_simulation**
Provides simulation environments and maps for testing and SLAM.

- **Launch Files:**  
  - `autonomous_navigation.launch.py`  
  - `house_sim.launch.py`  
  - `house_slam.launch.py`  
  - `outdoor.launch.py`
- **Maps:** `house_map.pgm`, `house_map.yaml`
- **Worlds:** `house.world`, `bookstore.world`
- **Configs:** `ekf.yaml`, `nav2_params.yaml`
- **Status:** ✅ Built

---

## 🚀 How to Build

From the workspace root (`ros2_ws/`):

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## ▶️ How to Run Packages

### Run Sensor Stack (with EKF)
```bash
ros2 launch seav_test_sensors ekf.launch.py
```

### Run Lidar Node
```bash
ros2 launch rplidar_ros ekf.launch.py
```

### Launch Navigation Stack
```bash
ros2 launch my_nav2_pkg nav2.launch.py
```

### Run Robot Controller Node
```bash
ros2 run my_robot_controller my_robot_controller_node
```

### Publish Static Transforms
```bash
ros2 launch static_tf_publisher static_tf_publisher.launch.py
```

### Launch Simulation (House Map)
```bash
ros2 launch seav_simulation house_sim.launch.py
```

---

## 📁 Folder Structure

```
ros2_ws/
├── src/
│   ├── my_nav2_pkg/
│   ├── my_robot_controller/
│   ├── rplidar_ros/
│   ├── seav_description/
│   ├── seav_simulation/
│   ├── seav_slam/
│   ├── seav_test_sensors/
│   ├── static_tf_publisher/
│   └── scripts/
├── docs/
│   ├── images/  ← TF tree, architecture, circuit diagrams, etc.
│   ├── notebooks/
│   ├── ppt/
│   └── ros2_workspace_documentation.xlsx
├── README.md
├── build/
├── install/
└── log/
```

---

## 🛠 Requirements

- **ROS 2 Distro:** (Humble)
- **Tools:**  
  - `colcon`  
  - `xacro`  
  - `rviz2`  
  - `robot_localization`  
  - `nav2_bringup`

## References

