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


class DriveSubscriber(Node):
    def __init__(self):
        super().__init__('drive_subscriber')
        self.subscription = self.create_subscription(
            I2CDrive,
            INBOUND_TOPIC,
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: {} {}'.format(msg.pwr_left, msg.pwr_right))
        direction = 0
        self.update_bit(direction, 0, 1 if msg.pwr_left < 0 else 0)
        self.update_bit(direction, 1, 1 if msg.pwr_right < 0 else 0)
        while not i2c.try_lock():
            i2c.writeto(TARGET_ADDR, bytes([direction, abs(msg.pwr_left), abs(msg.pwr_right)]), stop=False)

    def update_bit(self, num, bit, i):
        mask = ~(1 << i)
        return (num & mask) | (bit << i)


def main(args=None):
    rclpy.init(args=args)
    drive_subscriber = DriveSubscriber()
    rclpy.spin(drive_subscriber)
    drive_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
