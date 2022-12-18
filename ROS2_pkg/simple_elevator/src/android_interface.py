#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from simple_elevator.msg import State
from simple_elevator.msg import Geometry
from simple_elevator.msg import Scheduled
from simple_elevator.msg import Order
import socket
import time


class Android_interface(Node):

    def __init__(self):
        
        # create new node
        super().__init__('android_interface')

        # declare some parameters
        self.state = State()
        self.in_door_scheduled = []
        self.out_door_scheduled = []

        self.port = 8000
        self.maxConnections = 999

        # create opject for android communication
        self.listenSocket = socket.socket()
        self.hostname = socket.gethostname()
        self.dns_resolved_addr = socket.gethostbyname(self.hostname)
        self.listenSocket.bind(('',self.port))
        self.listenSocket.listen(self.maxConnections)

        # print the hostname and the port number
        self.get_logger().info(f"Server is running on: {self.dns_resolved_addr }, on port: {self.port}")

        # declares that the node publishes messages and subscribe
        self.order_publisher = self.create_publisher(Order, 'order', 10)
        self.state_subscriber = self.create_subscription(State, 'state', self.state_callback, 10)
        self.scheduled_subscriber = self.create_subscription(Scheduled, 'scheduling', self.scheduling_callback, 10)

    def state_callback(self,msg):
        # update current state
        self.state.floor , self.state.door = msg.floor , msg.door

    def scheduling_callback(self,msg):
        # update scheduled
        self.in_door_scheduled = eval(msg.in_scheduled)
        self.out_door_scheduled = eval(msg.out_scheduled)

    def publish_order(self,floor,door,in_door):
        # publish user order
        order_msg = Order()
        order_msg.floor = floor
        order_msg.door = door
        order_msg.in_door = in_door
        self.order_publisher.publish(order_msg)



def main(args=None):
    rclpy.init(args=args)

    android_interface = Android_interface()
    
    try:
        rclpy.spin(android_interface)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        android_interface.destroy_node()


if __name__ == '__main__':
    main()

