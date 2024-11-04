## understanding the nav2 stack

- global and local planner
![[Pasted image 20241104020145.png]]

- In the image above , it can be seen that pink line represent global path (arrow lines) and purple/blue (very small line) show local path.

---

# Global Planner

The **Global Planner** is responsible for generating a feasible path by considering the entire environment. It calculates a route based on a **cost map** and provides a high-level path suggestion for navigation. This path is later refined by the local planner, which adjusts for real-time obstacles and conditions.

### Key Concepts and Components

#### 1. Cost Map
A **cost map** is a representation of the environment where each pixel has an assigned cost value. The cost values influence the path planning by indicating areas to avoid or prefer:
   - **Pixel Cost**: Each pixel represents a specific cost value.
   - **Obstacle Costs**: Obstacles in the environment have the highest cost to discourage the planner from passing through them.
   - **Color Representation**: 
     - **White pixels**: Represent areas with the lowest cost, typically free space.
     - **Shades of Gray**: Intermediate areas with varying levels of accessibility.
     - **Black pixels**: Represent areas with obstacles or high costs that the robot should avoid.

#### 2. Global Path vs. Local Path
   - **Global Path**: Once the global path is generated, it’s sent to the local path planner.
   - **Update Rate**: The update rate of the local path is higher than the global path. 
     - The global path acts as a high-level guide with a lower update rate (e.g., 1 Hz), similar to a GPS.
     - The local path planner adjusts at a faster rate (e.g., 20 Hz), responding quickly to immediate obstacles, much like a driver’s real-time control of a vehicle.

---

# Navigation Parameters for Tuning the Cost Map

### Global Planning Parameters

To tune the global planning process, various parameters can be adjusted to better navigate the environment. This can be done using **rqt**, a ROS tool for dynamic parameter tuning.

```shell
# Start rqt for dynamic parameter configuration
rqt
# Navigate to:
# Plugins >> Configuration >> Dynamic Reconfigure >> Global Cost Map
```

#### Important Parameters in the Global Cost Map

1. **Publish Frequency**: Determines how frequently the cost map updates (default 1 Hz). A higher frequency ensures more frequent updates but can increase computational load.
2. **Inflation Radius**: This is the radius around each obstacle within which additional cost is applied, creating a safety buffer around obstacles.
3. **Robot Radius**: Defines the robot’s physical size, used to keep the path clear of tight spaces.

> Example: By modifying the **inflation radius**, you control how close the robot can approach obstacles.
   - **Lower Inflation Radius**: Easier path computation but a higher risk of collision with obstacles.
   - **Higher Inflation Radius**: Safer path with a larger buffer around obstacles, though potentially more computationally expensive.

---

### Local Planning Parameters

Local planning parameters are similar to those in the global cost map but optimized for real-time adjustments. Some commonly used parameters are:
1. **Local Publish Frequency**: Typically higher than the global publish frequency to allow for real-time responsiveness.
2. **Inflation Radius**: Helps in maintaining a safe buffer in dynamic environments where obstacles may move or change.

---

# Controller Server Parameters

The **Controller Server** is responsible for sending velocity commands to the robot based on the planned paths. Important parameters include:

1. **Acceleration**: Specifies the rate at which the robot can increase or decrease its speed.
2. **Controller Frequency**: Controls the frequency (in Hz) at which velocity commands are sent to the robot.

---

## Additional Resources

For further understanding and practical implementation, 
- **ROS Documentation on Navigation Stack**: [ROS Navigation Stack](http://wiki.ros.org/navigation)
- **Dynamic Reconfigure in ROS**: [ROS Dynamic Reconfigure Tutorial](http://wiki.ros.org/dynamic_reconfigure/Tutorials)
- **Cost Map 2D API**: [Costmap_2D Documentation](http://wiki.ros.org/costmap_2d)

![[Pasted image 20241104023653.png]] Image showing different params and then sub-params in it. Refer the architecture of Nav2. 

![[nav2_architecture.png]]