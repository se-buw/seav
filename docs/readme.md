

### Reflections from Member 1: Lessons in ROS 2 Development and Hardware Integration

As we progress through our robotics project, the past few weeks have offered valuable insights—especially for someone balancing hardware constraints, limited time, and the ever-growing complexity of the ROS 2 ecosystem. Below is a summary of key themes and learning experiences from my side.

#### 1. Common Compile-Time Errors
Much of the initial struggle stemmed from standard compile-time errors. These included incorrect dependencies in `CMakeLists.txt` and `package.xml`, undefined symbols, and occasional version mismatches. Resolving them has underscored the importance of understanding the build system deeply, especially when mixing C++ and Python packages.

#### 2. Configuration Files
We have started to rely heavily on configuration files—YAML for parameters, URDF/Xacro for robot description, and sensor-specific settings. Maintaining clean, well-organized config files has become essential for repeatability and collaboration.

#### 3. Launch Files
Learning to structure launch files effectively has been another cornerstone. Grouping nodes logically, applying namespace remappings, and using launch arguments for modularity are techniques that have helped simplify testing and integration phases.

#### 4. Learning Package Development – Catkin and Python
My understanding of ROS 2 has deepened significantly by exploring both `catkin`-based and Python-based package development. Although our project mainly uses Python nodes, being able to navigate CMake and understand the catkin workspace structure has been very useful.

#### 5. Debug Arguments and Logging
One of the more practical skills learned was how to leverage ROS 2's logging and debugging options. Passing `--ros-args` with appropriate log levels and reviewing the log directory (`~/.ros/log`) has often been more revealing than console output alone.

#### 6. Learning from TurtleBot3 Structure
In terms of software architecture, we borrowed significantly from the TurtleBot3 stack. Replicating its structure provided a solid starting point and helped us learn how a modular, well-documented robot setup functions in the real world.

#### 7. Hardware-Centric Project with Limited Resources
Unlike simulation-heavy setups, our work is grounded in configuring real hardware with practical limitations. Time constraints and limited resources have shaped many of our decisions. The focus has remained on achieving functionality using available components rather than ideal setups.

#### 8. ROS Mobile Application Integration
We also explored mobile integration through the ROS Mobile App. This was primarily used for teleoperation and monitoring in scenarios where physical access to the robot was limited. Though not central to the project, it added flexibility during testing.

#### 9. Modular Planning and Basic Task Management
As development progressed, it became evident that a structured task management approach was necessary. We began experimenting with lightweight SCRUM principles—breaking work into modules, assigning tasks, and conducting short sync-ups. Even a basic Kanban board made a difference in clarity and tracking progress.

#### 10. Incremental Testing and Integration
A key takeaway from this phase has been the effectiveness of testing individual components in isolation before integration. Attempting to debug an entire system at once proved inefficient. Testing in smaller units helped isolate errors early and allowed for more stable integration.

---

### Concluding Remarks
This phase of the project has been more about **engineering with what is available** than chasing ideal solutions. It required shifting focus from purely theoretical planning to practical decision-making—something that has enriched my understanding of robotics beyond just software. While there's still a long way to go, this process has laid a strong foundation for more robust development going forward.

---
