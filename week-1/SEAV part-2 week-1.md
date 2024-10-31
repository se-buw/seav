## Running Turtlesim Node with Commands (3 Tabs Setup)

### Tab 1: Start Turtlesim Node

- **Command**: 
  ```bash
  ros2 run turtlesim turtlesim_node

Purpose: Launches the turtlesim_node, which opens a GUI with a turtle that can be controlled programmatically.



---

Tab 2: Control the Turtle

Command:

ros2 run turtlesim turtle_teleop_key

Purpose: Opens a keyboard control interface to move the turtle manually. Use arrow keys to control the turtle's movement in the GUI.




---

Tab 3: Inspect Topics and Message Interfaces

1. List Available Topics

Command:

ros2 topic list

Purpose: Lists all active topics, allowing you to see available communication channels, such as /turtle1/cmd_vel.



2. Get Topic Info for /turtle1/cmd_vel

Command:

ros2 topic info /turtle1/cmd_vel

Purpose: Shows details about the /turtle1/cmd_vel topic, including publisher and subscriber nodes. This topic is used to control the turtle's velocity.



3. Show Message Structure for geometry_msgs/msg/Twist

Command:

ros2 interface show geometry_msgs/msg/Twist

Purpose: Displays the structure of the Twist message, which is used to send velocity commands (linear and angular) to the turtle.





---

Summary of Commands by Tab

Tab 1

ros2 run turtlesim turtlesim_node

Launches the main Turtlesim GUI node with a turtle.



Tab 2

ros2 run turtlesim turtle_teleop_key

Allows you to control the turtle with keyboard commands.



Tab 3

ros2 topic list

Lists active topics.


ros2 topic info /turtle1/cmd_vel

Shows detailed information about the /turtle1/cmd_vel topic.


ros2 interface show geometry_msgs/msg/Twist

Shows the structure of the Twist message used for controlling turtle velocity.



By following these commands in separate terminal tabs, you can manage, control, and inspect the Turtlesim node and its communication.

