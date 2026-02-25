"""AVROS Webots vehicle driver — cmd_vel to Car API.

Follows the webots_ros2_driver plugin pattern (same as webots_ros2_tesla).
Subscribes to /cmd_vel (Twist) and drives the Webots Car via
setCruisingSpeed() and setSteeringAngle().

Ackermann inverse kinematics matches actuator_node.py:162-176.
"""

import rclpy
from geometry_msgs.msg import Twist
from math import atan2

WHEELBASE = 1.23
MAX_STEERING_RAD = 0.489


class AvrosVehicleDriver:

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        rclpy.init(args=None)
        self.__node = rclpy.create_node('avros_vehicle_driver')
        self.__node.create_subscription(
            Twist, 'cmd_vel', self.__cmd_vel_callback, 1
        )

        self.__speed = 0.0
        self.__steering = 0.0

    def __cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        self.__speed = v

        # Ackermann inverse: same math as actuator_node.py
        if abs(v) > 0.01:
            steering_rad = atan2(omega * WHEELBASE, abs(v))
        elif abs(omega) > 0.01:
            # Turning in place: max steering in direction of omega
            steering_rad = MAX_STEERING_RAD if omega > 0 else -MAX_STEERING_RAD
        else:
            steering_rad = 0.0

        # Clip to mechanical limits
        steering_rad = max(-MAX_STEERING_RAD,
                           min(MAX_STEERING_RAD, steering_rad))

        self.__steering = steering_rad

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # setCruisingSpeed expects km/h, cmd_vel is m/s
        self.__robot.setCruisingSpeed(self.__speed * 3.6)
        self.__robot.setSteeringAngle(self.__steering)
