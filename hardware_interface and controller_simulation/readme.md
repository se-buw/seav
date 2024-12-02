# Aim : make hardware interface for driving the steering vehicle, front :servo, back differential


## Protocol
- ros2_control overview - personal
- Writing a URDF - done
- Writing a hardware interface - done
- Writing a controller – Ackermann 



## **Car-like Robot Simulation in Gazebo**

### **Overview**
This repository contains the hardware interface and controller setup for simulating a car-like robot in Gazebo. It uses ROS 2 Humble and the `ackermann_steering_controller` for precise control of car-like steering dynamics. The robot's configuration is defined through a URDF/Xacro file, and the system is fully integrated with Gazebo for testing.

---

### **Features**
- Hardware interface for real-time joint state updates and command handling.
- ROS 2 controllers for Ackermann steering and bicycle steering.
- Gazebo simulation with accurate car-like dynamics.
- Modular design for easy extension and integration.

---

### **Repository Structure**

workspace is example11_ws1

```plaintext
D:.
├───.vscode
├───bringup
│   ├───config
│   └───launch
├───description
│   ├───launch
│   ├───ros2_control
│   └───urdf
├───doc
├───hardware
│   └───include
│       └───ros2_control_demo_example_11
└───test
```

---

### **Requirements**
- **ROS 2 Humble**
- **Gazebo** (Recommended version: Fortress or Garden)
- **ros2_control** and **ros2_controllers**
  ```bash
  sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
  ```

---

### **Setup Instructions**

1. **Clone the Repository**:
   ```bash
   git clone <repository_url>
   cd <repository_folder>
   ```

2. **Build the Workspace**:
   ```bash
   cd ~/example11_ws1
   colcon build
   source install/setup.bash
   ```

3. **Launch Gazebo Simulation**:
   ```bash
   ros2 launch ros2_control_demo_example_11 view_robot.launch.py # for viewing the file in rviz
   ros2 launch ros2_control_demo_example_11 carlikebot.launch.py remap_odometry_tf:=true # for running in Gazebo
   
   ```
   
   

4. **Send Velocity Commands**:
   Use the `/cmd_vel` topic to control the robot:
   ```bash
   ros2 topic pub --rate 30 /bicycle_steering_controller/reference geometry_msgs/msg/TwistStamped "
  twist:
    linear:
      x: 1.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.1"
   ```
```bash (log of hardware interface)

```
---

### **Key Components**

#### **1. Hardware Interface**
- Implements the `hardware_interface::SystemInterface`.
- Handles joint state updates and command interfaces for:
  - Steering joints (`virtual_front_wheel_joint`).
  - Drive joints (`virtual_rear_wheel_joint`).

#### **2. Controllers**
- **Ackermann Steering Controller**:
  - Precise modeling of Ackermann steering geometry.
  - Configured in `ackermann_steering_controller.yaml`.

- **Bicycle Steering Controller**:
  - Simplified car-like dynamics approximation.
  - Configured in `bicycle_steering_controller.yaml`.

#### **3. URDF/Xacro**
- Defines the robot's geometry, joints, and dynamics.
- File: `urdf/carlikebot.urdf.xacro`.

---

### **Customization**

1. **Modify the URDF/Xacro**:
   Adjust dimensions, joint configurations, or materials in `carlikebot.urdf.xacro`.

2. **Controller Parameters**:
   - Fine-tune the `ackermann_steering_controller.yaml` for your robot's specific dynamics.
   - Example:
     ```yaml
     wheelbase: 0.558
     rear_wheels_names: ['rear_left_wheel_joint', 'rear_right_wheel_joint']
     ```

3. **Hardware Interface**:
   Extend `carlikebot_system.hpp` and `carlikebot_system.cpp` for additional sensors or actuators.

---

### **Simulation in Gazebo**
Launch the simulation using the provided launch file. The robot is configured with plugins to simulate joint state publishers, velocity controllers, and steering dynamics.

---

### **Future Work**

- Extend to real hardware using ROS 2 `hardware_interface`.

https://github.com/se-buw/seav/blob/adi/task/hardware_interface%20and%20controller_simulation/Screenshot%202024-12-02%20213403.jpg

