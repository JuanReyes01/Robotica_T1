import rclpy
from rclpy.node import Node
from curtsies import Input
from geometry_msgs.msg import Twist
import threading
import time

class turtleBotTeleop(Node):
    
    def __init__(self):
        """
        Attribute declaration
        """
        # Set timeout counter to 1
        self.timeout = 1
        # Set velocity counter to 0.0
        self.vel = 0.0
        # Gather linear and angular velocity from user
        self.linear, self.angular = self.read_values()
        # Store the last key pressed
        self.last_key_pressed = None
        # Initialize a timer
        self.last_key_time = time.time()

        """
        Construction of the object
        """
        # Initializing the Node
        super().__init__("Turtle_bot_teleop")
        # Log into console that the node has been initialized
        self.get_logger().info("Key Selected Node Begin")
        print("""
        Use:
             w
            asd
        To control the robot
              """)
        # Create the publisher
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtlebot_cmdVel",  1)
        # Create a thread to manage key reading
        self.read_keys_thread = threading.Thread(target=self.read_keys)
        self.read_keys_thread.start()
        # Continuously execute the action associated with the last key pressed
        self.execute_last_action()

    """
    Method that receives the values for linear and angular velocity at the start of the program
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
                if e is None:
                    self.last_key_pressed = None
                else:
                    print(e)
                    # Update the last key pressed
                    self.last_key_pressed = e
                    # Update the time when a key was last pressed
                    self.last_key_time = time.time()

    """
    Method that executes the action associated with the last key pressed
    """
    def execute_last_action(self):
        msg = Twist()
        while True:
            current_time = time.time()
            # Check if it has been more than the timeout value since the last key press
            if current_time - self.last_key_time > self.timeout:
                self.last_key_pressed = None  # Reset last_key_pressed to None
            if self.last_key_pressed:
                if self.last_key_pressed == 'w':
                    self.vel =  self.linear
                    msg.linear.x =  self.linear
                elif self.last_key_pressed == 'd':
                    msg.angular.z = -self.angular
                elif self.last_key_pressed == 's':
                    self.vel = -self.linear
                    msg.linear.x = -self.linear
                elif self.last_key_pressed == 'a':
                    msg.angular.z =  self.angular
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            self.cmd_vel_pub_.publish(msg)
            time.sleep(0.1)  # wait for 0.1 seconds

def main(args=None):
    rclpy.init(args=args)
    node = turtleBotTeleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
