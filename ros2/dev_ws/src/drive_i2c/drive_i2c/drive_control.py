import rclpy
from rclpy.node import Node
import busio
from board import *

from robomower_interfaces.msg import I2CDrive # pwr_left, pwr_right

#
# Converts power commands from the Drive controller to I2C wire instructions for the Ardiuno
#

# For RPI 3
i2c = busio.I2C(SCL, SDA)
TARGET_ADDR = 0x9
INBOUND_TOPIC = 'i2c_drive_cmd'

def set_bit(value, bit):
    return value | (1<<bit)

class DriveSubscriber(Node):
    def __init__(self):
        super().__init__('i2c_drive_subscriber')
        self.subscription = self.create_subscription(
            I2CDrive,
            INBOUND_TOPIC,
            self.listener_callback,
            10)
        self.get_logger().info("Ready")

    def listener_callback(self, msg):
        self.get_logger().info('I heard: {} {}'.format(msg.pwr_left, msg.pwr_right))
        direction = 0

        if msg.pwr_left < 0:
            direction = set_bit(direction, 0)

        if msg.pwr_right < 0:
            direction = set_bit(direction, 1)

        # Obtain the lock
        try:
            while not i2c.try_lock():
                pass

            # Write the data
            self.get_logger().info("Writing: {} {} {}".format(hex(direction), hex(abs(msg.pwr_left)), hex(abs(msg.pwr_right))))
            i2c.writeto(TARGET_ADDR, bytes([direction, abs(msg.pwr_left), abs(msg.pwr_right)]), stop=False)
        except Exception as e:
            self.get_logger().error("{}".format(e))
        finally:
            # Release the lock
            i2c.unlock()



def main(args=None):
    rclpy.init(args=args)
    drive_subscriber = DriveSubscriber()
    rclpy.spin(drive_subscriber)
    drive_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
