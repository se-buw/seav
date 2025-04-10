WORKSPACE="/home/pi/demo_testing_30_march/seav"
PARAM_FILE="$WORKSPACE/src/seav_slam/mapper_params_online_async.yaml"
MAP_BASENAME="$WORKSPACE/src/my_nav2_pkg/map_from_slam/auto_slam_map"
MAP_FILE="$MAP_BASENAME.yaml"
SESSION="ros_phase3"

# 1. Start static TF publisher first – this is critical for SLAM
tmux new-session -d -s $SESSION -n tf_pub "source $WORKSPACE/install/setup.bash && ros2 launch static_tf_publisher static_tf_launch.py"
sleep 3

# 2. Launch RPLIDAR – SLAM toolbox depends on laser scans
tmux new-window -t $SESSION -n rplidar "source $WORKSPACE/install/setup.bash && ros2 launch rplidar_ros rplidar_a1_launch.py"
sleep 3

# 3. Launch IMU and Distance Publishers – used by odometry node
tmux new-window -t $SESSION -n imu_pub "source $WORKSPACE/install/setup.bash && ros2 run seav_test_sensors imu_serial_publisher"
sleep 3
tmux new-window -t $SESSION -n distance_pub "source $WORKSPACE/install/setup.bash && ros2 run seav_test_sensors distance_publisher"
sleep 3

# 4. Launch Odometry publisher – critical for SLAM and Nav2
tmux new-window -t $SESSION -n odom_pub "source $WORKSPACE/install/setup.bash && ros2 run seav_test_sensors odom_publisher"
sleep 3

# 5. Launch Motor Controller (if required for motion)
tmux new-window -t $SESSION -n motor_ctrl "source $WORKSPACE/install/setup.bash && ros2 run my_robot_controller serial_motor_controller"
sleep 3

# 6. Launch SLAM Toolbox – requires laser and TF to be working
tmux new-window -t $SESSION -n slam_toolbox "sleep 3 && source $WORKSPACE/install/setup.bash && ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file $PARAM_FILE"
sleep 60

# 7. Automatically save the map after mapping is done
tmux new-window -t $SESSION -n map_saver "source $WORKSPACE/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f $MAP_BASENAME"
sleep 3

# 8. Launch Nav2 Localization (AMCL) with the saved map
tmux new-window -t $SESSION -n nav2_local "sleep 3 && source $WORKSPACE/install/setup.bash && ros2 launch my_nav2_pkg localization_launch.py map:=$MAP_FILE"
sleep 3

# 9. Launch Nav2 Navigation (planner, controller, etc.)
tmux new-window -t $SESSION -n nav2_nav "source $WORKSPACE/install/setup.bash && ros2 launch my_nav2_pkg/navigation_launch.py"
sleep 3

# 10. Launch YOLOv8 ROS node (for object detection)
tmux new-window -t $SESSION -n yolov8 "source $WORKSPACE/install/setup.bash && ros2 run yolov8_ros yolov8_node --device 0"
