#!/bin/bash

SESSION=ros_phase3
WORKSPACE="/home/pi/demo_testing_30_march/seav"
MAP_FILE="$WORKSPACE/src/my_nav2_pkg/config/maze_map_save.yaml"

tmux new-session -d -s $SESSION -n ekf "source $WORKSPACE/install/setup.bash && ros2 launch my_robot_description ekf_launch.py"
tmux new-window -t $SESSION -n nav2_local "source $WORKSPACE/install/setup.bash && ros2 launch my_nav2_pkg localization_launch.py map:=$MAP_FILE"
tmux new-window -t $SESSION -n nav2_nav "source $WORKSPACE/install/setup.bash && ros2 launch my_nav2_pkg/navigation_launch.py"
tmux new-window -t $SESSION -n yolov8 "source $WORKSPACE/install/setup.bash && ros2 run yolov8_ros yolov8_node -- device 0"

tmux attach -t $SESSION
