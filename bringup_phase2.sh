#!/bin/bash

SESSION=ros_phase2
WORKSPACE="/home/pi/demo_testing_30_march/seav"
PARAM_FILE="$WORKSPACE/src/seav_slam/mapper_params_online_async.yaml"

tmux new-session -d -s $SESSION -n motor_ctrl "source $WORKSPACE/install/setup.bash && ros2 run my_robot_controller serial_motor_controller"
tmux new-window -t $SESSION -n slam_toolbox "source $WORKSPACE/install/setup.bash && ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file $PARAM_FILE"

tmux attach -t $SESSION
