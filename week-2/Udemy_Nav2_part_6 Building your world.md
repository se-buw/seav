
> [!NOTE]
> gazebo << Edit << Building Editor 

Now creating my first ros2 workspace, where based on the floor plan , a world is created in gazebo, which will serve as a simulation for turtlebot. 


For setting up the workspace :
```Shell

git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

then build it using colcon

add into the bash file, so it launches every time when it starts

```

#### Launching
```bash
two file are required 
1. my_world.world
2. turtlebot3_my_world_launch.py - contains the initial points, physics  server - its a launch file
```

```Shell
ros2 launch turtlebot3_gazebo turtlebot3_my_world.launch.py

ros2 launch turtlebot3_navigation2 naivigation2.launch.py use_sim_time:=True maps:maps/custom_map.yaml





```


### Q. Why this grey patch created at first place?
![[WhatsApp Image 2024-11-04 at 16.05.51_6eb49eb0.jpg]]

### Protocol of creating your world to simulating bot autonomously
1. create a world in gazebo from maze
- .world extension
- provide scale to  image
2. Adapt turtlebot3 for this world
- make the start at one of entrances
3. Generate map using SLAM + navigation
- use gimp to clear the noise
<mark style="background: #ADCCFFA6;">skipping the video from $38$ to $40$. </mark>

