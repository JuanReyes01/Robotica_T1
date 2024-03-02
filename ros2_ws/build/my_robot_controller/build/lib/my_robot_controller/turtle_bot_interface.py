#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt    
from matplotlib.animation import FuncAnimation
import threading 
import json
from tkinter import messagebox
import time

class turtleBotInterface(Node):
    

    def __init__(self):
        super().__init__("turtle_interface")
        ask = messagebox.askyesno(title="TurtleBotSim", message="Do you want to record")
        self.pos = {'x': [], 'y': []}
        self.turtle_position_ = self.create_subscription(Twist, "/turtlebot_position", self.position_callback, 1)
        if ask:
            open("./moves.json", "w")
            print("Recording in progress")
            self.turtle_moves_ = self.create_subscription(Twist, "/turtlebot_cmdVel",self.save_movement ,1) 
    
    def save_movement(self, msg: Twist):
        try:
            with open('./moves.json', 'r') as file:
                self.data = json.load(file)
        except FileNotFoundError:
            print("File 'moves.json' not found.")
            self.data = []
        except json.decoder.JSONDecodeError:
            print("Error decoding JSON data.")
            self.data = []

        # Append the new movement data
        self.data.append({
            "linear": {
                "x": msg.linear.x,
                "y": msg.linear.y,
                "z": msg.linear.z
            },
            "angular": {
                "x": msg.angular.x,
                "y": msg.angular.y,
                "z": msg.angular.z
            }
        })

        # Write the updated data back to the file
        with open('./moves.json', 'w') as file:
            json.dump(self.data, file)

        print("Movement data saved successfully" + str(time))


        """
        with open("./moves.json", "r") as file:
            self.read = file.read().replace('[','').replace(']','')
            if len(self.read)>0 and self.read[0]==',':
                self.read = self.read[1,len(self.read)]
            self.content = self.read+ ',' + str(mov)
        with open("./moves.json", "w") as file:
            file.write('['+self.content+']')
        """
    
    def plot_data(self):
        fig, ax = plt.subplots()
        ax.set_title("Taller 1")
        ax.grid()
        ln, = ax.plot([], [], 'ro')
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)

        def init():
            return ln,

        def update(frame):
            ln.set_data(self.pos['x'], self.pos['y'])
            return ln,

        ani = FuncAnimation(fig, update, frames=None, init_func=init, blit=True)
        plt.show()

    def position_callback(self, msg: Twist):
        if len(self.pos['x'])>0 and len(self.pos['y'])>0: 
            # x changes but y is still the same
            if self.pos['x'][len(self.pos['x'])-1] != round(msg._linear._x,2) and self.pos['y'][len(self.pos['y'])-1] == round(msg._linear._y,2):
                self.pos['x'] = self.pos['x'].__add__([round(msg._linear._x,2)])
                self.pos['y'] = self.pos['y'].__add__([self.pos['y'][len(self.pos['y'])-1]])
            # y changes but x is still the same
            if self.pos['y'][len(self.pos['y'])-1] != round(msg._linear._y,2) and self.pos['x'][len(self.pos['x'])-1] == round(msg._linear._x,2):
                self.pos['y'] = self.pos['y'].__add__([round(msg._linear._y,2)])
                self.pos['x'] = self.pos['x'].__add__([self.pos['x'][len(self.pos['x'])-1]])
            # x changes
            if self.pos['x'][len(self.pos['x'])-1] != round(msg._linear._x,2):
                self.pos['x'] = self.pos['x'].__add__([round(msg._linear._x,2)])
            # y changes
            if self.pos['y'][len(self.pos['y'])-1] != round(msg._linear._y,2):
                self.pos['y'] = self.pos['y'].__add__([round(msg._linear._y,2)])
        else:
            self.pos['x'] = [round(msg._linear._x,2)]
            self.pos['y'] = [round(msg._linear._y,2)]
            
        
        #print(self.pos)
    def on_shutdown(self):
        with open("./moves.txt", "w") as file:
            file.write(self.content)

def main(args=None):
    rclpy.init(args=args)
    node = turtleBotInterface()
    
    # Create a thread to run the plot
    plot_thread = threading.Thread(target=node.plot_data)
    #save_thread = threading.Thread(target=node.on_shutdown)
    #plot_thread.daemon(True)
    plot_thread.start()

    rclpy.spin(node)

    #node.on_shutdown(node.on_shutdown_callback)
    rclpy.shutdown()


if __name__ == '__main__':
    content = ""
    main()
