import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class SerialMotorController(Node):
    def __init__(self):
        super().__init__('serial_motor_controller')
        # Subscribe to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        # Initialize serial communication with the Arduino
        self.serial_port = serial.Serial('/dev/arduino', 2000000, timeout=1)
        self.default_speed = 100  # Default speed (on a scale of 0-255)
        self.current_speed = self.default_speed
        self.send_speed(self.current_speed)  # Set the default speed on the Arduino

        # Only prompt for reset once during initialization
        self.prompt_for_reset()

    def prompt_for_reset(self):
        """Prompt user once at startup for resetting the distance."""
        user_input = input("Type 'RESET' to reset distance or press Enter to continue: ").strip()
        if user_input.upper() == "RESET":
            self.serial_port.write(b"RESET\n")
            self.get_logger().info("Sent RESET command to Arduino")
        else:
            self.get_logger().info("Continuing without reset")

    def send_speed(self, speed):
        """Send the speed to the Arduino."""
        self.serial_port.write(f"SPEED {speed}\n".encode())
        self.get_logger().info(f"Speed set to: {speed}")

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # If a movement command is detected, use the current or default speed
        if linear_x > 0:
            self.serial_port.write(f"SPEED {self.current_speed}\n".encode())
            self.serial_port.write(b"FORWARD\n")
            self.get_logger().info("Command sent: FORWARD")
        elif linear_x < 0:
            self.serial_port.write(f"SPEED {self.current_speed}\n".encode())
            self.serial_port.write(b"BACKWARD\n")
            self.get_logger().info("Command sent: BACKWARD")
        elif angular_z > 0:
            self.serial_port.write(b"LEFT\n")
            self.get_logger().info("Command sent: LEFT")
        elif angular_z < 0:
            self.serial_port.write(b"RIGHT\n")
            self.get_logger().info("Command sent: RIGHT")
        else:
            # Stop if no movement command is present
            self.serial_port.write(b"STOP\n")
            self.get_logger().info("Command sent: STOP")

    def destroy_node(self):
        # Close the serial port before shutting down
        if self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motor_controller = SerialMotorController()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
