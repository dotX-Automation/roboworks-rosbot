#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Roboworks ROSbot Keyboard Teleoperation
#
# This node reads keyboard keys from a TTY and publishes geometry_msgs/Twist
# messages to control a differential-drive robot.
#
# Author: dotX Automation s.r.l
# License: Apache-2.0
# -----------------------------------------------------------------------------

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# -------------------------------------------------------------------------
HELP_MSG = """
Roboworks ROSbot Keyboard Control
---------------------------------
Movement:
  q  w  e
  a  s  d
  z  x  c

Speed adjustment:
  o/p : increase/decrease linear speed
  k/l : increase/decrease angular speed

Other:
  SPACE or s : stop immediately
  CTRL+C     : quit
---------------------------------
"""

MOVE_BINDINGS = {
    "q": (1.0, 1.0),
    "w": (1.0, 0.0),
    "e": (1.0, -1.0),
    "a": (0.0, 1.0),
    "s": (0.0, 0.0),
    "d": (0.0, -1.0),
    "z": (-1.0, 1.0),
    "x": (-1.0, 0.0),
    "c": (-1.0, -1.0),
}

# -------------------------------------------------------------------------
def get_key(settings) -> str:
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# -----------------------------------------------------------------------------
# ROS 2 Keyboard Teleop Node
# -----------------------------------------------------------------------------
class RosbotKeyboard(Node):

    def __init__(self) -> None:
        super().__init__("rosbot_keyboard")

        # Declare parameters
        self.declare_parameter("max_vel_linear", 0.5)   # m/s
        self.declare_parameter("max_vel_angular", 1.0)  # rad/s
        self.declare_parameter("step_linear", 0.05)     # m/s
        self.declare_parameter("step_angular", 0.1)     # rad/s

        # Read parameters
        self.max_vel_linear = float(self.get_parameter("max_vel_linear").value)
        self.max_vel_angular = float(self.get_parameter("max_vel_angular").value)
        self.step_linear = float(self.get_parameter("step_linear").value)
        self.step_angular = float(self.get_parameter("step_angular").value)

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Current speed limits
        self.linear = 0.0
        self.angular = 0.0

        # Movement direction
        self.dir_linear = 0.0   # forward/backward (-1 to 1)
        self.dir_angular = 0.0  # rotation (-1 to 1)

        self.get_logger().info(HELP_MSG)
        self.print_speed_info()

    # -----------------------------------------------------
    def print_speed_info(self):
        self.get_logger().info(
            f"Linear limit: {self.linear:.2f} m/s, "
            f"Angular limit: {self.angular:.2f} rad/s"
        )

    # -----------------------------------------------------
    def apply_move_key(self, key):
        self.dir_linear, self.dir_angular = MOVE_BINDINGS[key]

    # -----------------------------------------------------
    def increase_linear(self):
        self.linear = min(self.max_vel_linear, self.linear + self.step_linear)
        self.print_speed_info()

    # -----------------------------------------------------
    def decrease_linear(self):
        self.linear = max(0.0,self.linear - self.step_linear)
        self.print_speed_info()

    # -----------------------------------------------------
    def increase_angular(self):
        self.angular = min(self.max_vel_angular,self.angular + self.step_angular)
        self.print_speed_info()

    # -----------------------------------------------------
    def decrease_angular(self):
        self.angular = max(0.0,self.angular - self.step_angular)
        self.print_speed_info()


# -----------------------------------------------------------------------------
def main(args=None):
    if not sys.stdin.isatty():
        print("ERROR: This node requires a TTY to read keyboard input.")
        return
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = RosbotKeyboard()
    try:
        while rclpy.ok():
            key = get_key(settings)

            # Movement keys
            if key in MOVE_BINDINGS:
                node.apply_move_key(key)
            elif key == "o":
                node.increase_linear()
            elif key == "p":
                node.decrease_linear()
            elif key == "k":
                node.increase_angular()
            elif key == "l":
                node.decrease_angular()
            elif key == "\x03":  # CTRL+C
                break
            else:
                node.dir_linear = 0.0
                node.dir_angular = 0.0

            # Publish Twist message
            twist = Twist()
            twist.linear.x = node.dir_linear * node.linear
            twist.angular.z = node.dir_angular * node.angular
            node.cmd_vel_pub.publish(twist)

    finally:
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
