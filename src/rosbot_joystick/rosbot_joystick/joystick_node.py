#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Roboworks ROSbot Joystick Teleoperation
#
# This node subscribes to sensor_msgs/Joy and publishes geometry_msgs/Twist
# messages to control a differential-drive robot.
#
# Author: dotX Automation s.r.l
# License: Apache-2.0
# -----------------------------------------------------------------------------

from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


# -----------------------------------------------------------------------------
# ROS 2 Joystick Teleop Node
# -----------------------------------------------------------------------------
class RosbotJoystick(Node):

    def __init__(self) -> None:
        super().__init__("rosbot_joystick")

        # Declare parameters
        self.declare_parameter("max_vel_linear", 0.5)    # m/s
        self.declare_parameter("max_vel_angular", 1.0)   # rad/s
        self.declare_parameter("axis_linear", 1)         # left stick vertical
        self.declare_parameter("axis_angular", 0)        # left stick horizontal
        self.declare_parameter("deadzone_linear", 0.1)   # [-1.0, 1.0]
        self.declare_parameter("deadzone_angular", 0.1)  # [-1.0, 1.0]

        # Read parameters
        self.max_vel_linear = float(self.get_parameter("max_vel_linear").value)
        self.max_vel_angular = float(self.get_parameter("max_vel_angular").value)
        self.axis_linear = int(self.get_parameter("axis_linear").value)
        self.axis_angular = int(self.get_parameter("axis_angular").value)
        self.deadzone_linear = float(self.get_parameter("deadzone_linear").value)
        self.deadzone_angular = float(self.get_parameter("deadzone_angular").value)

        # Subscriber for Joy messages
        self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    # -------------------------------------------------------------------------
    def _safe_axis(self, axes, index: int) -> float:
        """Safely read an axis and warn once if index is out of range."""
        if index < 0 or index >= len(axes):
            self.get_logger().warn_once(
                f"Axis index {index} is out of range for Joy axes (size {len(axes)})."
            )
            return 0.0
        return float(axes[index])

    # -------------------------------------------------------------------------
    def joy_callback(self, msg: Joy) -> None:
        """Joystick callback: convert Joy to Twist."""
        axes = msg.axes

        # Read raw axes
        linear = self._safe_axis(axes, self.axis_linear)
        angular = self._safe_axis(axes, self.axis_angular)

        # Apply deadzones
        if abs(linear) < self.deadzone_linear:
            linear = 0.0
        if abs(angular) < self.deadzone_angular:
            angular = 0.0

        # Publish Twist message
        twist = Twist()
        twist.linear.x = self.max_vel_linear * linear
        twist.angular.z = self.max_vel_angular * angular
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RosbotJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
