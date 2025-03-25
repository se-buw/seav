# seav
Software Engineering for Autonomous Vehicles .

## TF Tree

![TF Tree](docs/images/tf_tree.png)


1. **Each package's purpose**
2. **Nodes, topics, launch files**
3. **How to run them**

---

## ✅ Sample `README.md` Section (Based on Your Table)

```markdown
# SEAV ROS 2 Workspace

This repository contains a collection of ROS 2 packages developed for autonomous robot navigation, sensing, and control. Each package resides in the `src/` folder and is designed for a specific functionality.

---

## 📦 Package Overview

### 1. **seav_test_sensors**
Collects and publishes data from IMU, distance sensor, and odometry source.

- **Nodes:** `imu_serial_publisher`, `distance_publisher`, `odom_publisher`
- **Publishes:** `/imu`, `/distance`, `/odom`
- **Subscribes:** `/imu`, `/distance`
- **Dependencies:** `rclpy`, `robot_localization`, `tf2_ros`, `sensor_msgs`
- **Launch File:** `ekf.launch.py`
- **Status:** ✅ Built

> Includes an EKF node using `robot_localization` to improve state estimation.

---

### 2. **rplidar_ros**
Driver for RPLIDAR A1 sensor.

- **Publishes:** `/parameter_events`, `/rosout`
- **Launch File:** `ekf.launch.py`
- **Status:** ✅ Built

---

### 3. **my_nav2_pkg**
Configuration and launch support for Nav2 stack.

- **Nodes:** `robot_state_publisher`, `map_server`, `amcl`, etc.
- **Publishes:** `/tf`, `/map`, `/amcl_pose`, `/cmd_vel`, etc.
- **Subscribes:** `/scan`, `/goal_pose`, `/map`
- **Dependencies:** `nav2`, `launch`, `rclcpp`, etc.
- **Launch Files:** `nav2.launch.py`, `bringup.launch.py`, `ekf.launch.py`, etc.
- **Status:** ✅ Built

---

### 4. **my_robot_controller**
Forwards velocity commands to Arduino.

- **Publishes/Subscribes:** `/cmd_vel`, `/rosout`, etc.
- **Status:** ✅ Built

---

### 5. **static_tf_publisher**
Publishes static transforms between base_link, imu, and lidar.

- **Node:** `static_transform_publisher`
- **Launch File:** `static_tf_publisher.launch.py`
- **Status:** ✅ Built

---

## 🚀 How to Build

Make sure you're in the root of the workspace (e.g. `ros2_ws/`):

```bash
colcon build --symlink-install
```

Then source the environment:

```bash
source install/setup.bash
```

---

## ▶️ How to Run Packages

### Run Sensor Stack with EKF
```bash
ros2 launch seav_test_sensors ekf.launch.py
```

### Run Lidar Node
```bash
ros2 launch rplidar_ros ekf.launch.py
```

### Launch Navigation Stack (Nav2)
```bash
ros2 launch my_nav2_pkg nav2.launch.py
```

### Run Robot Controller
```bash
ros2 run my_robot_controller my_robot_controller_node
```

### Publish Static Transforms
```bash
ros2 launch static_tf_publisher static_tf_publisher.launch.py
```

---

## 📁 Folder Structure

```
ros2_ws/
├── src/
│   ├── seav_test_sensors/
│   ├── rplidar_ros/
│   ├── my_nav2_pkg/
│   ├── my_robot_controller/
│   └── static_tf_publisher/
├── build/
├── install/
└── README.md
```

---

## 🛠 Requirements

Ensure you have the following installed:

- ROS 2 Humble/Foxy/Galactic (whichever version you're using)
- Colcon
- Python3 dependencies: `rclpy`, `sensor_msgs`, `robot_localization`, etc.

---

