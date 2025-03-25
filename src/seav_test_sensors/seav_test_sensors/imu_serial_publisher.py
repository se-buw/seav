import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2 as smbus
import math
import time
import traceback

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  
        self.bus = smbus.SMBus(1)
        self.mpu_addr = 0x68  # Default I2C address for MPU6050

        # Wake up the MPU6050
        self.bus.write_byte_data(self.mpu_addr, 0x6B, 0)

    def read_word(self, reg):
        try:
            high = self.bus.read_byte_data(self.mpu_addr, reg)
            low = self.bus.read_byte_data(self.mpu_addr, reg + 1)
            value = (high << 8) + low
            if value >= 0x8000:  # Handle negative values
                value = -((65535 - value) + 1)
            return value
        except IOError as e:
            self.get_logger().warn(f"I2C Read Error: {e}")
            return 0  # Return a default value in case of error

    def read_scaled(self, reg, scale):
        return self.read_word(reg) / scale

    def reset_i2c_bus(self):
        self.bus.close()  # Close the bus
        time.sleep(0.5)   # Add a small delay before reopening
        self.bus = smbus.SMBus(1)  # Reopen the I2C bus

    def publish_imu_data(self):
        imu_msg = Imu()

        # Read accelerometer data (scaled to m/s²)
        accel_x = self.read_scaled(0x3B, 16384.0) * 9.81
        accel_y = self.read_scaled(0x3D, 16384.0) * 9.81
        accel_z = self.read_scaled(0x3F, 16384.0) * 9.81

        # Read gyroscope data (scaled to rad/s)
        gyro_x = self.read_scaled(0x43, 131.0) * math.pi / 180
        gyro_y = self.read_scaled(0x45, 131.0) * math.pi / 180
        gyro_z = self.read_scaled(0x47, 131.0) * math.pi / 180

        # Populate IMU message
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Orientation (set to zero if not available)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        imu_msg.orientation_covariance[0] = -1  # Indicates no orientation data

        # Angular velocity
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                               0.0, 0.01, 0.0,
                                               0.0, 0.0, 0.01]

        # Linear acceleration
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        imu_msg.linear_acceleration_covariance = [0.1, 0.0, 0.0,
                                                  0.0, 0.1, 0.0,
                                                  0.0, 0.0, 0.1]

        # Publish the IMU message
        self.publisher_.publish(imu_msg)
        self.get_logger().info("Published IMU data")

    def handle_crash(self, e):
        """Handles and logs the exception with traceback"""
        self.get_logger().error(f"Error occurred: {e}")
        self.get_logger().error("Traceback:")
        tb_str = ''.join(traceback.format_exception(etype=type(e), value=e, tb=e.__traceback__))
        self.get_logger().error(tb_str)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()

    # Spin the node to keep publishing
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        imu_publisher.handle_crash(e)  # Log the crash and traceback
        imu_publisher.reset_i2c_bus()  # Reset I2C bus if an error occurs
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
