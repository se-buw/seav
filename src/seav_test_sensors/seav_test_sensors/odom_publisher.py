import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # ✅ Use Reliable QoS for TF & Odometry
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)

        # ✅ **Force TF2 to Store Only 0.1s of Data**
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=0.1))
        self.tf_broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ✅ **Force a Rolling Cache (Flush Old Data)**
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_distance = 0.0
        self.last_time = self.get_clock().now()

        # Subscriptions
        self.create_subscription(Float32, '/distance', self.distance_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Angular velocity
        self.angular_velocity_z = 0.0

        # Publish odometry at a fixed rate (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_odometry)

    def distance_callback(self, msg):
        current_distance = msg.data
        if current_distance >= self.last_distance:
            delta_distance = current_distance - self.last_distance
            self.last_distance = current_distance

            # Update x, y based on theta
            self.x += delta_distance * math.cos(self.theta)
            self.y += delta_distance * math.sin(self.theta)

    def imu_callback(self, msg):
        self.angular_velocity_z = msg.angular_velocity.z

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # Convert to seconds
        self.last_time = current_time

        # Update theta
        self.theta += self.angular_velocity_z * dt

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation
        quat = self.euler_to_quaternion(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation = quat

        # Velocity
        odom_msg.twist.twist.linear.x = self.last_distance / dt if dt > 0 else 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity_z

        self.odom_pub.publish(odom_msg)

        # ✅ **Fix: Broadcast TF with Correct Timestamp & Enforce Rolling Cache**
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()  # ✅ Fresh timestamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat

        self.tf_broadcaster.sendTransform(t)  # ✅ TF now has fresh data with limited history

        # ✅ **Force the TF Cache to Only Store Recent Data**
        self.flush_old_tf_data()

    def flush_old_tf_data(self):
        """ Forcibly remove any old TF data. """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()

    try:
        rclpy.spin(odom_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        odom_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
