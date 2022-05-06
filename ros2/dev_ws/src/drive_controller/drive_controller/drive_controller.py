# Inspiration & Resources
# https://github.com/ros-controls/ros2_controllers/blob/master/diff_drive_controller/src/diff_drive_controller.cpp
#
# Inputs:
# - Topic: i2c_odometry
# - drive_i2c publishes a count periodically, reset to 0 on each read
# - Message contains cnt_left, cnt_left, noting how many wheel magnets have passed the Hall Effect sensor
# - Each count represents 51.015mm (0.051015 meters) travelled by the wheel (measured center-center in Fusion 360)
# - 12 counts is a full revolution
#
# - Topic: cmd_vel
# - Receives Twist messages from navigation modules or teleop_twist_keyboard (for development) https://github.com/ros-teleop/teleop_twist_keyboard
# - Control in the x-y plane by manipulating the x component of linear speed
# - Control in the z component of angular speed
#
#
# Outputs:
# - Topic: i2c_drive_cmd
# - Outputs motor power commands, which the drive_i2c module then translates into I2C for the Ardiuno controller
#
# - Topic: pose
# - Outputs the currently known location of the robot based on odometry. TODO: In the future, we will pass this through an IMU service.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robomower_interfaces.msg import I2CDrive # pwr_left, pwr_right
from robomower_interfaces.msg import I2COdometry

WHEEL_RADIUS = 0.1 # 100mm
WHEEL_SEPARATION = 0.17 # 170mm
ODOMETRY_TRAVEL = 0.051015
TOPIC_I2C_DRIVE = 'i2c_drive_cmd'
TOPIC_I2C_ODOMETRY = "i2c_odometry"

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_subscriber')
        self.twist_subscription = self.create_subscription(
            Twist,
            "cmd_vel",
            self.update_drive,
            10)

        self.odometry_subscription = self.create_subscription(
            I2COdometry,
            TOPIC_I2C_ODOMETRY,
            self.update_odometry,
            10)

        self.i2c_cmd = self.create_publisher(I2CDrive, TOPIC_I2C_DRIVE, 20)

        self.right_travel = 0

        self.get_logger().info("Ready")

    def update_drive(self, msg):
        linear_cmd = msg.linear.x
        angular_cmd = msg.angular.z
        self.get_logger().info('New Target - Linear: {} Angular: {}'.format(linear_cmd, angular_cmd))

        velocity_left = (linear_cmd - angular_cmd * WHEEL_SEPARATION / 2.0) #/ WHEEL_RADIUS
        velocity_right = (linear_cmd + angular_cmd * WHEEL_SEPARATION / 2.0) #/ WHEEL_RADIUS

        self.get_logger().info('{} {}'.format(velocity_left, velocity_right))

        # Publish Drive speed (between 150-255 motor power)
        # Max speed 0.3m/s (map it to 0.35
        msg = I2CDrive()
        msg.pwr_left = int(self.speed_to_pwr(velocity_left))
        msg.pwr_right = int(self.speed_to_pwr(velocity_right))
        self.i2c_cmd.publish(msg)

        self.get_logger().info('{} {}'.format(msg.pwr_left, msg.pwr_right))


    def update_odometry(self, msg):
        self.get_logger().info('{}'.format(msg))

        self.right_travel += msg.cnt_right * ODOMETRY_TRAVEL # TODO * current wheel direction

        # Add right_speed for this second

        self.get_logger().info('{}'.format(self.right_travel))

        # TODO: Publish the pose, then visualise in Rviz


    def speed_to_pwr(self, speed):
        if speed > 0:
            return translate(speed, 0, 0.35, 150, 255)  # TODO: Ideally we want this to push max speed when possible

        if speed < 0:
            return translate(speed, 0, -0.35, -150, -255)

        return 0

def main(args=None):
    rclpy.init(args=args)
    drive_subscriber = DriveController()
    rclpy.spin(drive_subscriber)
    drive_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

