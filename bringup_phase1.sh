#!/bin/bash

SESSION=ros_phase1
WORKSPACE="/home/pi/demo_testing_30_march/seav"
# Running in Simulation (Gazebo) with ROS2 Control (Recommended):
# tmux new-session -d -s $SESSION -n rsp "source $WORKSPACE/install/setup.bash && ros2 launch seav_description rsp.launch.py use_sim_time:=true use_ros2_control:=true sim_mode:=true"
# . Running on Real Hardware with ROS2 Control
# tmux new-session -d -s $SESSION -n rsp "source $WORKSPACE/install/setup.bash && ros2 launch seav_description rsp.launch.py use_sim_time:=false use_ros2_control:=true sim_mode:=true"
#  3. Running in Simulation using Gazebo plugins (No ROS2 Control):
# tmux new-session -d -s $SESSION -n rsp "source $WORKSPACE/install/setup.bash && ros2 launch seav_description rsp.launch.py use_sim_time:=true use_ros2_control:=false sim_mode:=true"
tmux new-session -d -s $SESSION -n tf_pub "source $WORKSPACE/install/setup.bash && ros2 launch static_tf_publisher static_tf_launch.py"
tmux new-window -t $SESSION -n rplidar "source $WORKSPACE/install/setup.bash && ros2 launch rplidar_ros rplidar_a1_launch.py"
tmux new-window -t $SESSION -n distance_pub "source $WORKSPACE/install/setup.bash && ros2 run seav_test_sensors distance_publisher"
tmux new-window -t $SESSION -n imu_pub "source $WORKSPACE/install/setup.bash && ros2 run seav_test_sensors imu_serial_publisher"
tmux new-window -t $SESSION -n odom_pub "source $WORKSPACE/install/setup.bash && ros2 run seav_test_sensors odom_publisher"

tmux attach -t $SESSION
