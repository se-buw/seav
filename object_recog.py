import cv2
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from ultralytics import YOLO

# Estimated focal length 
FOCAL_LENGTH = 923  # Adjusted based on calibration
REAL_OBJECT_HEIGHT = 0.35  

def estimate_distance(pixel_height):
    if pixel_height <= 0:
        return None
    return (FOCAL_LENGTH * REAL_OBJECT_HEIGHT) / pixel_height

class YOLOv8DistancePublisher(Node):
    def __init__(self, device_index):
        super().__init__('yolov8_distance_publisher')
        self.distance_publisher = self.create_publisher(Float32, 'object_distance', 10)
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()  # Converts OpenCV images to ROS Image messages

        # Load YOLOv8 model
        self.model = YOLO("yolov8n.pt")

        # Start webcam 
        self.cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video device {device_index}")
            return

        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Could not read frame")
            return

        # Perform object detection
        results = self.model(frame)

        for result in results:
            for box in result.boxes:
                conf = box.conf[0].item()
                if conf > 0.5:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    pixel_height = y2 - y1  
                    
                    # Estimate distance
                    distance = estimate_distance(pixel_height)
                    
                    if distance:
                        msg = Float32()
                        msg.data = distance
                        self.distance_publisher.publish(msg)
                        self.get_logger().info(f'Published Distance: {distance:.2f}m')

                    # Draw bounding box
                    cls = int(box.cls[0].item())
                    label = f"{self.model.names[cls]}: {conf * 100:.2f}%, {distance:.2f}m"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Convert frame to ROS Image message and publish it
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_publisher.publish(image_msg)

        cv2.imshow("Object Recognition with Distance Estimation", frame)

        if cv2.waitKey(1) & 0xFF == ord('x'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main():
    rclpy.init()
    
    parser = argparse.ArgumentParser(description="YOLOv8 with Distance Estimation in ROS 2")
    parser.add_argument("--device", type=int, default=0, help="Video capture device index")
    args = parser.parse_args()

    node = YOLOv8DistancePublisher(args.device)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
