import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import json

class DistancePublisher(Node):
    def __init__(self):
        super().__init__('distance_publisher')

        self.publisher_ = self.create_publisher(Float32, '/distance', 10)

        # Initialize serial communication
        try:
            self.serial_port = serial.Serial('/dev/arduino', 2000000, timeout=1)
            self.get_logger().info("Serial port initialized successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to initialize serial port: {e}")
            self.serial_port = None

        # Store last known distance
        self.last_distance = 0.0

        # Publish at a fixed rate (10 Hz)
        self.timer = self.create_timer(0.1, self.read_distance)

    def read_distance(self):
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().error("Serial port not available.")
            return

        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line:
                data = json.loads(line)
                distance = data.get("distance")

                if distance is not None:
                    self.last_distance = distance  # Update last known distance

            # Always publish the last known distance
            msg = Float32()
            msg.data = self.last_distance
            self.publisher_.publish(msg)

            self.get_logger().info(f"Published distance: {self.last_distance}")

        except json.JSONDecodeError:
            self.get_logger().debug(f"Ignored invalid JSON data: {line}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    distance_publisher = DistancePublisher()

    try:
        rclpy.spin(distance_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        distance_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
