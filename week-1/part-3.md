## Important Elements for Publishing a Message in ROS2

When publishing a message in ROS2, certain elements are critical for successful communication:

1. **Message Type**  
   - Specifies the format and structure of the message being sent.
   - Example: `geometry_msgs/msg/Twist` is the type for velocity messages.

2. **Topic Name**  
   - Defines the communication channel over which the message is sent.
   - Both publisher and subscriber must use the same topic name to exchange information.

   > **Note**: If the message type or topic name is not correctly defined, communication between nodes will not work.

---

### Commands for Verifying Message Publishing

After starting the node, use the following commands to verify that publishing is working correctly:

1. **List Active Topics**
   - **Command**:
     ```bash
     ros2 topic list
     ```
   - **Purpose**: Displays all active topics, allowing you to identify channels for communication.

2. **Get Topic Information**
   - **Command**:
     ```bash
     ros2 topic info <topic_name>
     ```
   - **Example**:
     ```bash
     ros2 topic info /turtle1/cmd_vel
     ```
   - **Purpose**: Provides details about the specified topic, including the message type and any active publishers or subscribers.

3. **View Messages from Publisher**
   - **Command**:
     ```bash
     ros2 topic echo /turtle1/cmd_vel
     ```
   - **Purpose**: Displays messages published to `/turtle1/cmd_vel`, allowing you to see the data being transmitted.

Using these commands, you can confirm that a message is being published on the correct topic with the proper message type.

# Write a ROS Subscriber with Python

In this tutorial, we will write a Python script to create a ROS2 subscriber that listens to a topic and receives data. This subscriber will retrieve the current coordinates of a turtle in the `turtlesim` simulation.

---

## Steps to Create a Subscriber

### 1. Understand the Data Type and Topic

- **Data Type**: Before creating the subscriber, it’s essential to know the type of data that will be received. In this case, we want to obtain the turtle’s coordinates, which are published as `turtlesim/msg/Pose` messages.
  
- **Topic to Subscribe**:  
  - **Topic Name**: `/turtle1/pose`
  - **Purpose**: This topic provides the turtle’s current position (x, y) and orientation in the `turtlesim` simulation.

### 2. Writing the Subscriber Code

Below is the Python code for a ROS2 subscriber that subscribes to the `/turtle1/pose` topic to receive the turtle’s coordinates:

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtlePoseSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_pose_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def pose_callback(self, msg):
        # Print the received coordinates and orientation
        self.get_logger().info(
            f"Received Turtle Pose - X: {msg.x}, Y: {msg.y}, Theta: {msg.theta}"
        )

def main(args=None):
    rclpy.init(args=args)
    turtle_pose_subscriber = TurtlePoseSubscriber()
    rclpy.spin(turtle_pose_subscriber)
    turtle_pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


---

Explanation of the Code

Subscriber Node (TurtlePoseSubscriber):

Initializes a ROS2 node named turtle_pose_subscriber.

Creates a subscription to the /turtle1/pose topic, which uses the Pose message type.

Defines the callback function pose_callback to process and print the turtle’s coordinates whenever a new message is received.


Callback Function (pose_callback):

Receives Pose messages and logs the turtle's current x, y, and theta values (coordinates and orientation).



3. Run the Subscriber

To test the subscriber, ensure that the turtlesim node is running. Then, execute this script to see real-time updates of the turtle’s coordinates.


---

Commands for Testing

1. Start the turtlesim Node:

ros2 run turtlesim turtlesim_node


2. Run the Subscriber:

python3 turtle_pose_subscriber.py



The subscriber should now receive and display the turtle's current position as published on the /turtle1/pose topic.
