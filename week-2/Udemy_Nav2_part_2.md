## There are two things we need to short out for setup of Nav2
1. DDS change to cyclonedds.
2. We need to update a configuration file where turtlebot3 is installed, we may need to do it again, as its a temporary solution until new patch comes up. 
```shell
user@DESKTOP-1NU5M8F:/$ cd /opt/ros/humble/

user@DESKTOP-1NU5M8F:/$ cd /opt/ros/humble/share$ ls

to see all the packages installed in humble by you

cd turtlebot3_navigation2/

then ls

cd param/
ls
edit waffle.yaml

then we should edit "waffle.yaml" file and change the argument of 

robot_model_type: "nav2_amcl::DifferentialMotionModel"

use: 
sudo nano waffle.yaml , for editing in administrator 


ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/my_map.yaml
```

>  ok, this took some time to resolve, "the problem is map was not getting loaded in Rviz".
>  what i did?
>  downloaded map from other git repo, use that as path file. 

In the video "Make Robot navigate using the generated Map"
+ we loaded gazebo , rviz with pre-created map based on Lidar manual scanning.
+ and then in Rviz, we assigned the direction of bot, based on local mapping is created as a layer.
- Global planner
- Controller - local planner
- we follow right hand coordinate system here, X forward, 90 deg  left Y,  and thumb towards Z.
- To run the system again, provide 2d point estimate and then assign __Nav2 Goal__
- Now this is weird , the system is rebooted for other purpose, after that when the nav2 tried on the file __my_turtlebot3_ map.yaml__ its not working, before it was working. 
- So, now i tried with my generated map __my_map.yaml__ and its starts to work. 


### Multi points goals , which bot takes, two modes
1. "Start Waypoint Following"
2. "Start Nav Through Poses" - bot goes without stoping - but not stable feature , bot some times stuck

#### Dynamic Obstacle Avoidance
We are going to place, obstacles  which is not present in scene initially and trying to see its effect. 



## Task for home universe
- Turning in the corner points is difficult to do bot, the present is not perfect for this tasks.
- To tackle, manually we should assign in "Multi mode" number of navigation points around the sharp corner
- Maybe in latter part we learn about some params tweaking.

