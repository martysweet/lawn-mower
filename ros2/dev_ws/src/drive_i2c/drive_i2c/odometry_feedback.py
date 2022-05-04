import rclpy
from rclpy.node import Node
import busio

from robomower.msg import I2COdometry


i2c = busio.I2C(3, 2)
TARGET_ADDR = 0x9
OUTBOUND_TOPIC = 'i2c_odometry'

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(I2COdometry, OUTBOUND_TOPIC, 20)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Read from I2C
        result = bytearray(2)
        while not i2c.try_lock():
            i2c.readfrom_into(TARGET_ADDR, result)

        msg = I2COdometry()
        msg.cnt_left = int.from_bytes([result[0]], "big")       # TODO Check Endian
        msg.cnt_right = int.from_bytes([result[1]], "big")
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing message')


def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
