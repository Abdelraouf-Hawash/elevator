#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from simple_elevator.msg import State
from simple_elevator.msg import Geometry
from simple_elevator.msg import Scheduled
from simple_elevator.msg import Order
from simple_elevator.action import Go
import tkinter as tk
import threading
import sys


class GUI(Node):

    def __init__(self):
        
        # create new node
        super().__init__('GUI')

        # declare some parameters


        # declares that the node publishes messages and subscribe
        self.order_publisher = self.create_publisher(Order, 'order', 10)
        self.state_subscriber = self.create_subscription(State, 'state', self.state_callback, 10)
        self.scheduled_subscriber = self.create_subscription(Scheduled, 'scheduling', self.scheduling_callback, 10)

        ## GUI
        self.window = tk.Tk()
        self.window.title('simple elevator')
        self.window.configure(bg='gray65')
        self.window.resizable(width=False, height=False)
        # Buttons and labels
        self.state_label = tk.Label(self.window, text="_ , _", width=10, height = 5, fg="red", bg = "black", font="arial 10 bold")
        self.in_door_label = tk.Label(self.window, text="In door", fg="black", bg = "gray65", font="arial 10 bold")
        self.out_door_label = tk.Label(self.window, text="Out door", fg="black", bg = "gray65", font="arial 10 bold")
        # in door buttons
        self.in_30_button = tk.Button(self.window, text="3.0", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(3,0,True) )
        self.in_31_button = tk.Button(self.window, text="3.1", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(3,1,True) )
        self.in_32_button = tk.Button(self.window, text="3.2", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(3,2,True) )
        self.in_20_button = tk.Button(self.window, text="2.0", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(2,0,True) )
        self.in_21_button = tk.Button(self.window, text="2.1", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(2,1,True) )
        self.in_22_button = tk.Button(self.window, text="2.2", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(2,2,True) )
        self.in_10_button = tk.Button(self.window, text="1.0", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(1,0,True) )
        self.in_11_button = tk.Button(self.window, text="1.1", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(1,1,True) )
        self.in_12_button = tk.Button(self.window, text="1.2", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(1,2,True) )
        self.in_00_button = tk.Button(self.window, text="0.0", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(0,0,True) )
        self.in_01_button = tk.Button(self.window, text="0.1", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(0,1,True) )
        self.in_02_button = tk.Button(self.window, text="0.2", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(0,2,True) )
        # out buttons
        self.out_30_button = tk.Button(self.window, text="+", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(3,0,False) )
        self.out_31_button = tk.Button(self.window, text="+", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(3,1,False) )
        self.out_32_button = tk.Button(self.window, text="+", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(3,2,False) )
        self.out_20_button = tk.Button(self.window, text="+", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(2,0,False) )
        self.out_21_button = tk.Button(self.window, text="+", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(2,1,False) )
        self.out_22_button = tk.Button(self.window, text="+", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(2,2,False) )
        self.out_10_button = tk.Button(self.window, text="+", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(1,0,False) )
        self.out_11_button = tk.Button(self.window, text="+", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(1,1,False) )
        self.out_12_button = tk.Button(self.window, text="+", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(1,2,False) )
        self.out_00_button = tk.Button(self.window, text="+", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(0,0,False) )
        self.out_01_button = tk.Button(self.window, text="+", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(0,1,False) )
        self.out_02_button = tk.Button(self.window, text="+", width=5, fg="black", bg = "gray65", font="arial 10 bold", command = lambda: self.button_cb(0,2,False) )
        #Positions
        self.state_label.grid(row=1, column=3, padx= 10, pady=15)
        self.in_door_label.grid(row=1, column=1, padx= 10, pady=15)
        self.out_door_label.grid(row=1, column=5, padx= 10, pady=15)

        self.in_30_button.grid(row=3, column=0, padx= 10, pady=15)
        self.in_31_button.grid(row=3, column=1, padx= 10, pady=15)
        self.in_32_button.grid(row=3, column=2, padx= 10, pady=15)
        self.in_20_button.grid(row=4, column=0, padx= 10, pady=15)
        self.in_21_button.grid(row=4, column=1, padx= 10, pady=15)
        self.in_22_button.grid(row=4, column=2, padx= 10, pady=15)
        self.in_10_button.grid(row=5, column=0, padx= 10, pady=15)
        self.in_11_button.grid(row=5, column=1, padx= 10, pady=15)
        self.in_12_button.grid(row=5, column=2, padx= 10, pady=15)
        self.in_00_button.grid(row=6, column=0, padx= 10, pady=15)
        self.in_01_button.grid(row=6, column=1, padx= 10, pady=15)
        self.in_02_button.grid(row=6, column=2, padx= 10, pady=15)

        self.out_30_button.grid(row=3, column=4, padx= 10, pady=15)
        self.out_31_button.grid(row=3, column=5, padx= 10, pady=15)
        self.out_32_button.grid(row=3, column=6, padx= 10, pady=15)
        self.out_20_button.grid(row=4, column=4, padx= 10, pady=15)
        self.out_21_button.grid(row=4, column=5, padx= 10, pady=15)
        self.out_22_button.grid(row=4, column=6, padx= 10, pady=15)
        self.out_10_button.grid(row=5, column=4, padx= 10, pady=15)
        self.out_11_button.grid(row=5, column=5, padx= 10, pady=15)
        self.out_12_button.grid(row=5, column=6, padx= 10, pady=15)
        self.out_00_button.grid(row=6, column=4, padx= 10, pady=15)
        self.out_01_button.grid(row=6, column=5, padx= 10, pady=15)
        self.out_02_button.grid(row=6, column=6, padx= 10, pady=15)


    def state_callback(self, msg):
        stat = f'{msg.floor},{msg.door}'
        self.state_label.config(text=stat)

    def scheduling_callback(self, msg):
        in_door_scheduled = eval(msg.in_scheduled)
        out_door_scheduled = eval(msg.out_scheduled)
        
        self.update_button_colors(in_door_scheduled,out_door_scheduled)
    
    def button_cb(self,floor,door,in_door):
        # publish user order
        order_msg = Order()
        order_msg.floor = floor
        order_msg.door = door
        order_msg.in_door = in_door
        self.order_publisher.publish(order_msg)

    def update_button_colors(self,in_door_scheduled,out_door_scheduled):
        # in door buttons
        self.in_30_button.config(fg= "red",relief=tk.SUNKEN) if ([3,0] in in_door_scheduled) else self.in_30_button.config(fg= "black",relief=tk.RAISED)
        self.in_31_button.config(fg= "red",relief=tk.SUNKEN) if ([3,1] in in_door_scheduled) else self.in_31_button.config(fg= "black",relief=tk.RAISED)
        self.in_32_button.config(fg= "red",relief=tk.SUNKEN) if ([3,2] in in_door_scheduled) else self.in_32_button.config(fg= "black",relief=tk.RAISED)
        self.in_20_button.config(fg= "red",relief=tk.SUNKEN) if ([2,0] in in_door_scheduled) else self.in_20_button.config(fg= "black",relief=tk.RAISED)
        self.in_21_button.config(fg= "red",relief=tk.SUNKEN) if ([2,1] in in_door_scheduled) else self.in_21_button.config(fg= "black",relief=tk.RAISED)
        self.in_22_button.config(fg= "red",relief=tk.SUNKEN) if ([2,2] in in_door_scheduled) else self.in_22_button.config(fg= "black",relief=tk.RAISED)
        self.in_10_button.config(fg= "red",relief=tk.SUNKEN) if ([1,0] in in_door_scheduled) else self.in_10_button.config(fg= "black",relief=tk.RAISED)
        self.in_11_button.config(fg= "red",relief=tk.SUNKEN) if ([1,1] in in_door_scheduled) else self.in_11_button.config(fg= "black",relief=tk.RAISED)
        self.in_12_button.config(fg= "red",relief=tk.SUNKEN) if ([1,2] in in_door_scheduled) else self.in_12_button.config(fg= "black",relief=tk.RAISED)
        self.in_00_button.config(fg= "red",relief=tk.SUNKEN) if ([0,0] in in_door_scheduled) else self.in_00_button.config(fg= "black",relief=tk.RAISED)
        self.in_01_button.config(fg= "red",relief=tk.SUNKEN) if ([0,1] in in_door_scheduled) else self.in_01_button.config(fg= "black",relief=tk.RAISED)
        self.in_02_button.config(fg= "red",relief=tk.SUNKEN) if ([0,2] in in_door_scheduled) else self.in_02_button.config(fg= "black",relief=tk.RAISED)
        # out door buttons
        self.out_30_button.config(fg= "red",relief=tk.SUNKEN) if ([3,0] in out_door_scheduled) else self.out_30_button.config(fg= "black",relief=tk.RAISED)
        self.out_31_button.config(fg= "red",relief=tk.SUNKEN) if ([3,1] in out_door_scheduled) else self.out_31_button.config(fg= "black",relief=tk.RAISED)        
        self.out_32_button.config(fg= "red",relief=tk.SUNKEN) if ([3,2] in out_door_scheduled) else self.out_32_button.config(fg= "black",relief=tk.RAISED)        
        self.out_20_button.config(fg= "red",relief=tk.SUNKEN) if ([2,0] in out_door_scheduled) else self.out_20_button.config(fg= "black",relief=tk.RAISED)        
        self.out_21_button.config(fg= "red",relief=tk.SUNKEN) if ([2,1] in out_door_scheduled) else self.out_21_button.config(fg= "black",relief=tk.RAISED)        
        self.out_22_button.config(fg= "red",relief=tk.SUNKEN) if ([2,2] in out_door_scheduled) else self.out_22_button.config(fg= "black",relief=tk.RAISED)        
        self.out_10_button.config(fg= "red",relief=tk.SUNKEN) if ([1,0] in out_door_scheduled) else self.out_10_button.config(fg= "black",relief=tk.RAISED)        
        self.out_11_button.config(fg= "red",relief=tk.SUNKEN) if ([1,1] in out_door_scheduled) else self.out_11_button.config(fg= "black",relief=tk.RAISED)        
        self.out_12_button.config(fg= "red",relief=tk.SUNKEN) if ([1,2] in out_door_scheduled) else self.out_12_button.config(fg= "black",relief=tk.RAISED)        
        self.out_00_button.config(fg= "red",relief=tk.SUNKEN) if ([0,0] in out_door_scheduled) else self.out_00_button.config(fg= "black",relief=tk.RAISED)        
        self.out_01_button.config(fg= "red",relief=tk.SUNKEN) if ([0,1] in out_door_scheduled) else self.out_01_button.config(fg= "black",relief=tk.RAISED)        
        self.out_02_button.config(fg= "red",relief=tk.SUNKEN) if ([0,2] in out_door_scheduled) else self.out_02_button.config(fg= "black",relief=tk.RAISED)        
        

def do_spin(node):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except:
        sys.exit()
    finally:
        rclpy.try_shutdown()
        node.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    gui = GUI()
    # using threading to use TKINTER with ROS2
    ros_process_thread = threading.Thread(target= lambda: do_spin(gui))
    ros_process_thread.start()
    # tkinter mainloop
    gui.window.mainloop()


if __name__ == '__main__':
    main()

