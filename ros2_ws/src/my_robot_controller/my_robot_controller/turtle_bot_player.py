#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json

class turtleBotPlayer(Node):

    def __init__(self):
        super().__init__("turtle_bot_player")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtlebot_cmdVel",  1)
        try:
            with open('./moves.json', 'r') as file:
                self.data = json.load(file)
                print("Data loaded succesfully")
        except FileNotFoundError:
            print("File 'moves.json' not found.")
            self.data = []
        except json.decoder.JSONDecodeError:
            print("Error decoding JSON data.")
            self.data = []
        self.i = 0
        self.j = len(self.data)
        self._timer = self.create_timer(0.1, self.play_move)
                
    def play_move(self):
        msg = Twist()
        if self.i<self.j:
            print("recreating move: "+ str(self.i))
            msg.angular.z = float(self.data[self.i]["angular"]["z"])
            msg.linear.x = float(self.data[self.i]["linear"]["x"])
            self.cmd_vel_pub_.publish(msg)
            self.i = self.i+1
        else:
            print("Recreation ended")

def main(args=None):
    rclpy.init(args=args)
    node = turtleBotPlayer()

    rclpy.spin(node)
    rclpy.shutdown()
    
   

if __name__ == '__main__':
    main()