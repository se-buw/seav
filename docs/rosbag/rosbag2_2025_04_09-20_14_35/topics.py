from pathlib import Path
from rosbags.highlevel import AnyReader

# Set path to the folder containing metadata.yaml and the .db3 file
bag_path = Path('/home/prims/Downloads/rosbag2_2025_04_09-20_14_35')

# ✅ DO NOT call reader.open() manually inside a `with` block
with AnyReader([bag_path]) as reader:
    print("\n📌 Topics in this ROS 2 bag:")
    for conn in reader.connections:
        print(f"  {conn.topic} : {conn.msgtype}")
