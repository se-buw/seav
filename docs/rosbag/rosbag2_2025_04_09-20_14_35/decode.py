from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.serde import deserialize_cdr
import pandas as pd

# Path to your bag directory
bag_path = Path('/home/prims/Downloads/rosbag2_2025_04_09-20_14_35')

# Output CSV path
csv_output = 'odom_data.csv'

# Use AnyReader WITHOUT calling .open()
with AnyReader([bag_path]) as reader:
    # Filter connections for the /odom topic
    connections = [c for c in reader.connections if c.topic == '/odom']

    rows = []

    for conn, timestamp, rawdata in reader.messages(connections):
        msg = deserialize_cdr(rawdata, conn.msgtype)

        rows.append({
            'timestamp': timestamp,
            'pos_x': msg.pose.pose.position.x,
            'pos_y': msg.pose.pose.position.y,
            'pos_z': msg.pose.pose.position.z,
            'orient_x': msg.pose.pose.orientation.x,
            'orient_y': msg.pose.pose.orientation.y,
            'orient_z': msg.pose.pose.orientation.z,
            'orient_w': msg.pose.pose.orientation.w,
            'lin_vel_x': msg.twist.twist.linear.x,
            'lin_vel_y': msg.twist.twist.linear.y,
            'lin_vel_z': msg.twist.twist.linear.z,
            'ang_vel_x': msg.twist.twist.angular.x,
            'ang_vel_y': msg.twist.twist.angular.y,
            'ang_vel_z': msg.twist.twist.angular.z,
        })

    df = pd.DataFrame(rows)
    df.to_csv(csv_output, index=False)
    print(f"✅ /odom data saved to: {csv_output}")
