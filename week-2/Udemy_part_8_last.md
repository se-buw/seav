
```Shell
sudo apt install ros-humble-urdf-tutorial

ros2 launch urdf_tutorial display.launch.py model:=/home/user/my_robot.urdf

gedit my_robot.urdf : to  see whats inside the file urdf
```

### Refer to URDF documentation

- we can even visualise the different turtlebot3s setup URDF file using 
`cd /opt/ros/humble/share/turtlebot3_description/` , due to some problem i cant see in mine.
- we can define camera link and whatever else we want.

`ros2 launch urdf_tutorial display.launch.py model:=/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_waffle.urdf.
For viewing the URDF file of turtlebot3 , use above command.

#### [[What is package here]]
##### 1. [[How to create ROS2 package (in general)]]
##### 2. [[How to create ROS2 workspace]]
##### 3. [[How to decide how many packages to construct in general]]

##### 4. Could you provide me with the Architecture of the system based on the given information 

> [!given information]
> a small car for our project "software for autonomus vehicles" we are going to use Car: 2 x 25-watt motor (total 50 watts), quiet tyres with a soft rubber ring. 2.4 GHz remote control, seat belt, horn. 12 Volt 4,5 Ah battery, up to 6 km/h speed Sensors: Raspberry Pi 4 - https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/ RPLIDAR A1 - https://www.slamtec.com/en/Lidar/A1/ and ros2 humble as the middleware

##### 5. [[How to create package for odometry]]

##### 6. [[How to troubleshoot odometry pkg]]
##### 7. [[Which questions to answer for writing a UDF file for our car configuration]]


> [!NOTE] Car description in brief
> for my given task descripion could you give me URDF file , if more information is required also let me know. Description of car, a small car for our project : 2 x 25-watt motor (total 50 watts), quiet tyres with a soft rubber ring. 2.4 GHz remote control, seat belt, horn. 12 Volt 4,5 Ah battery, up to 6 km/h speed Sensors: Raspberry Pi 4 - https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/ RPLIDAR A1 - https://www.slamtec.com/en/Lidar/A1/ , , There are total four wheels two front , which is controlled by a single servo motor and in the back two wheels both have its dc motor. 

##### 8. [[Examples of URDF file]]
##### 9. [[List of packages required for the task defined]]


