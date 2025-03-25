import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import time
import message_filters

class SlamPreCheck(Node):
    def __init__(self):
        super().__init__('slam_precheck')

        self.timestamps = {
            "scan": None,
            "imu": None,
            "odom": None,
            "tf": None
        }

        # ✅ Subscribing to necessary topics
        self.scan_sub = message_filters.Subscriber(self, LaserScan, "/scan")
        self.imu_sub = message_filters.Subscriber(self, Imu, "/imu")
        self.odom_sub = message_filters.Subscriber(self, Odometry, "/odom")
        self.tf_sub = message_filters.Subscriber(self, TFMessage, "/tf")

        # ✅ Synchronizer: Allow messages **without timestamps**
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.scan_sub, self.imu_sub, self.odom_sub, self.tf_sub],
            queue_size=10,
            slop=0.05,  # Allow max 50ms delay between messages
            allow_headerless=True  # ✅ Auto-assign ROS time to headerless messages
        )
        self.ts.registerCallback(self.sync_callback)

    def get_timestamp(self, msg):
        """ Extract timestamp from message or assign ROS time if missing. """
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            return time.time()  # ✅ Assign system time for messages without timestamps

    def sync_callback(self, scan, imu, odom, tf):
        """ Processes synchronized sensor messages """
        self.timestamps["scan"] = self.get_timestamp(scan)
        self.timestamps["imu"] = self.get_timestamp(imu)
        self.timestamps["odom"] = self.get_timestamp(odom)
        self.timestamps["tf"] = self.get_timestamp(tf.transforms[0]) if tf.transforms else time.time()

        self.check_sensors()

    def check_sensors(self):
        """ Verifies timestamp synchronization across all SLAM-related topics """
        timestamps = list(filter(None, self.timestamps.values()))
        if len(timestamps) < 4:
            return  # Ignore if any topic has not published yet

        max_time = max(timestamps)
        min_time = min(timestamps)
        max_diff = max_time - min_time

        print("\n🚀 SLAM Pre-Check Results 🚀")
        print(f"🔍 Max Time Difference: {max_diff:.6f} sec")
        if max_diff > 0.05:
            print("❌ WARNING: Time sync issue detected!")
        else:
            print("✅ Time synchronization is within limits.")

        for key, ts in self.timestamps.items():
            print(f"✅ {key}: {ts}")

        print("----------------------------------------")

def main(args=None):
    rclpy.init(args=args)
    slam_precheck = SlamPreCheck()

    try:
        rclpy.spin(slam_precheck)
    except KeyboardInterrupt:
        pass
    finally:
        slam_precheck.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
