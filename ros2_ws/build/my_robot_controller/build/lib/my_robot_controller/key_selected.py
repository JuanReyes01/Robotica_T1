#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tty
import sys
import termios

class SelectedKey(Node):
    def __init__(self):
        orig_settings = termios.tcgetattr(sys.stdin)
        super().__init__("selected_key")
        self.get_logger().info("Key Selected Node Begin")
        self._timer = self.create_timer(0.5, self.getKey,orig_settings)

    def getKey(self,orig_settings):
        
        tty.setcbreak(sys.stdin)
        x = 0
        while x != chr(27): # ESC
            x=sys.stdin.read(1)[0]
            print("You pressed", x)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings) 


def main(args=None):

    rclpy.init(args=args)
    node = SelectedKey()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
