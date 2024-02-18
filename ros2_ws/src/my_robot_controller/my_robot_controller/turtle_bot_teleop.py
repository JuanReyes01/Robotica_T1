#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from curtsies import Input
import queue
from geometry_msgs.msg import Twist
import threading


class turtleBotTeleop(Node):
    
    def __init__(self):
        """
            Atribute declaration
        """
        # set timeout counter to 1
        self.timeo = 1
        # set vel counter to 0.0
        self.vel =  0.0
        # gather linear and angular vel from user
        self.linear, self.angular = self.read_values()
        # key press queue (not useful now, maybe later)
        self.key_queue = queue.Queue()
        """
            Construction of the object
        """
        # initializing the Node
        super().__init__("Turtle_bot_teleop")
        # log into console that the node has been initialized
        self.get_logger().info("Key Selected Node Begin")
        print("""
        Use:
             w
            asd
        To control the robot
              """)
        # create the publisher
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtlebot_cmdVel",  1)
        # create a thread to manage key reading/ TODO: thread must be stoped when program exits
        self.read_keys_thread = threading.Thread(target=self.read_keys)
        #self.read_keys_thread.daemon(True)
        self.read_keys_thread.start()
        # start a async callout for getkey method 
        self._timer = self.create_timer(0.1, self.getKey)
            
        

    """
    Method that recieves the values for linear and angular velocity at the start of the program
    """
    def read_values(self):
        while True:
            v = input("Input linear velocity value: ")
            print("\n")
            a = input("Input angular velocity value: ")
            if v.isnumeric() and a.isnumeric():
                break
            else:
                print("\n\nERROR: please, input numeric values\n")
        return float(v), float(a)

    """
    Method that reads the pressed keys 
    """
    def read_keys(self):
        with Input(keynames='curses') as input_generator:
            for e in input_generator:
                self.key_queue.put(e)

    """
    Method that reacts to the read keys
    """
    def getKey(self):
        msg = Twist()
        try:
            e = self.key_queue.get_nowait()
            if e.__class__.__name__ == 'str':
                self.timeo = self.timeo+1
                print("key pressed: "+str(e))
                if e == 'w':
                    self.vel =  self.linear
                    msg.linear.x =  self.linear
                    self.cmd_vel_pub_.publish(msg)
                elif e == 'd':
                    msg.angular.z = -self.angular
                    msg.linear.x = self.vel
                    self.cmd_vel_pub_.publish(msg)
                elif e == 's':
                    self.vel = -self.linear
                    msg.linear.x = -self.linear
                    self.cmd_vel_pub_.publish(msg)
                elif e == 'a':
                    msg.linear.x = self.vel
                    msg.angular.z =  self.angular
                    self.cmd_vel_pub_.publish(msg)
            self.key_queue = queue.Queue()

        except queue.Empty:
           if self.timeo>0:
                self.vel = 0.0
                msg.linear.x =  0.0
                msg.angular.z =  0.0
                self.cmd_vel_pub_.publish(msg)
                self.timeo = 0
           else:
               self.timeo = self.timeo+1

def main(args=None):
    rclpy.init(args=args)
    node = turtleBotTeleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
