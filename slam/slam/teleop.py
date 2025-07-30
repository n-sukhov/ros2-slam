#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(TwistStamped, '/diff_drive_base_controller/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0.5
        self.turn = 0.5
        self.get_logger().info("Control: WASD + QE, SPACE - stop")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'

                if key == 'w':
                    msg.twist.linear.x = self.speed
                elif key == 's':
                    msg.twist.linear.x = -self.speed
                elif key == 'a':
                    msg.twist.angular.z = self.turn
                elif key == 'd':
                    msg.twist.angular.z = -self.turn
                elif key == 'q':
                    msg.twist.linear.x = self.speed
                    msg.twist.angular.z = self.turn
                elif key == 'e':
                    msg.twist.linear.x = self.speed
                    msg.twist.angular.z = -self.turn
                elif key == ' ':
                    msg.twist.linear.x = 0.0
                    msg.twist.angular.z = 0.0
                elif key == '\x03':
                    break
                else:
                    continue

                self.publisher.publish(msg)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopKeyboard()
    teleop.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
