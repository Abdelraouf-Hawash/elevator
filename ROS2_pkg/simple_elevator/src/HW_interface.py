#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
from rclpy.node import Node
from std_msgs.msg import Int32
from simple_elevator.msg import State
from simple_elevator.msg import Geometry
from simple_elevator.action import Go
import serial
import time


class HardwareInterface(Node):

    def __init__(self):
        super().__init__('hardware_interface')
        
        # declare some parameters
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('floor_pos', [0.13,0.3,0.49,0.67])
        self.declare_parameter('door_pos', [2,90,178])
        self.declare_parameter('HW_update_time', 1.0)
        self.declare_parameter('max_pwm', 200) 
        self.declare_parameter('Kp', 3500.0) # pID control parameters
        self.declare_parameter('Ki', 0.09)
        self.declare_parameter('Kd', 8700.0)

        self.current_z_pos = 0.0 # floor 1 as default start point
        self.current_theta_pos = 0.0
        self.current_floor = 1
        self.current_door = 1
        self.current_motor_pwm = 0
        self.target_z_pos = 0.3
        self.target_theta_pos = 90.0
        
        # create object for serial communication
        self.arduino = serial.Serial(port= self.get_parameter('arduino_port').value, baudrate=115200, timeout=0.1) # for communication with arduino

        # declares that the node publishes messages
        self.state_publisher = self.create_publisher(State, 'state', 10)
        self.geometry_publisher = self.create_publisher(Geometry, 'elevator_geometry', 10)
        self.load_publisher = self.create_publisher(Int32, 'motor_load', 10)
        
        # instantiate a new action server
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            Go,
            'elevator_go',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
            )

        # create timer to call cb func every n time
        self.timer = self.create_timer(self.get_parameter('HW_update_time').value, self.timer_callback)

    def timer_callback(self):
        # update hardware info
        self.HW_update()

        # publish current state
        state_msg = State()
        state_msg.floor = self.current_floor
        state_msg.door = self.current_door
        self.state_publisher.publish(state_msg)
        
        # publish current elevator geometry
        geometry_msg = Geometry()
        geometry_msg.z = self.current_z_pos
        geometry_msg.theta = self.current_theta_pos
        self.geometry_publisher.publish(geometry_msg)
        
        # publish current motor load (pwm)
        load_msg = Int32()
        load_msg.data = self.current_motor_pwm
        self.load_publisher.publish(load_msg)


    def HW_update(self):
        # self.get_logger().info('working') # for test

        # write Data
        max_pwm = self.get_parameter('max_pwm').value
        Kp = self.get_parameter('Kp').value
        Ki = self.get_parameter('Ki').value
        Kd = self.get_parameter('Kd').value
        
        data = f'{self.target_z_pos},{self.target_theta_pos},{max_pwm},{Kp},{Ki},{Kd}'
        self.arduino.write(data.encode('utf-8'))
        time.sleep(0.0005)

        #read Data
        Data = self.arduino.readline()
        Data = str(Data)[2:-1]
        # Data = f'{self.target_z_pos},{self.target_theta_pos},{self.current_motor_pwm}' #for test
        Data = Data.split(',')
        if len(Data) != 3:
            return
        
        # update current state
        self.current_z_pos = float(Data[0])
        self.current_theta_pos = float(Data[1])
        self.current_motor_pwm = int(Data[2])

        # update current state
        self.current_floor = min(range(len(self.get_parameter('floor_pos').value)), key=lambda i: abs(self.get_parameter('floor_pos').value[i] - self.current_z_pos) )
        self.current_door = min(range(len(self.get_parameter('door_pos').value)), key=lambda i: abs(self.get_parameter('door_pos').value[i] - self.current_theta_pos) )

    def destroy_action_server(self):
        self._action_server.destroy()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        floor_goal = goal_handle.request.floor
        door_goal = goal_handle.request.door
        self.get_logger().info(f'Executing goal...: ({floor_goal},{door_goal})')

        # update the target
        self.target_z_pos = self.get_parameter('floor_pos').value[floor_goal]
        self.target_theta_pos = self.get_parameter('door_pos').value[door_goal]

        # Append the seeds for the Go state
        feedback_msg = Go.Feedback()

        # Start executing the action
        while (not((abs(self.current_z_pos - self.target_z_pos) <= 2) and (abs(self.current_theta_pos - self.target_theta_pos) <= 2))):
            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return Go.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Go.Result()

            # Update Go state
            feedback_msg.floor = self.current_floor
            feedback_msg.door = self.current_door

            # self.get_logger().info(f'Publishing feedback: {feedback_msg.floor},{feedback_msg.door}')

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep
            time.sleep(self.get_parameter('HW_update_time').value)

        goal_handle.succeed()

        # Populate result message
        result = Go.Result()
        result.floor = self.current_floor
        result.door = self.current_door
        self.get_logger().info(f'Returning result:({self.current_floor},{self.current_door})')

        return result



def main(args=None):
    rclpy.init(args=args)

    hardware_interface = HardwareInterface()
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(hardware_interface, executor)
    except KeyboardInterrupt:
        pass
    finally:
        hardware_interface.destroy_action_server()
        hardware_interface.destroy_node()
        rclpy.try_shutdown()
    

if __name__ == '__main__':
    main()