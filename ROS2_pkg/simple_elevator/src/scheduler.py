#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32
from simple_elevator.msg import State
from simple_elevator.msg import Geometry
from simple_elevator.msg import Scheduled
from simple_elevator.msg import Order
from simple_elevator.action import Go
import time
import anything


class Scheduler(Node):

    def __init__(self):
        
        # create new node
        super().__init__('scheduler')

        # declare some parameters
        self.declare_parameter('elevator_waiting_time', 4)
        self.declare_parameter('in_scheduling_algorithm', 'SJF')
        self.declare_parameter('out_scheduling_algorithm', 'FCFS')
        self.declare_parameter('SJF_aging_value', 0.1) # priority increasing value every second
        self.declare_parameter('door_to_floor_priority', 5)

        self.last_state = State()
        self.last_state.floor ,self.last_state.door = 1,1 # initial values
        self.in_door_scheduled = []
        self.out_door_scheduled = []

        # declares that the node publishes messages and subscribe
        self.scheduled_publisher = self.create_publisher(Scheduled, 'scheduling', 10)
        self.order_subscriber = self.create_subscription(Order, 'order', self.order_callback, 10)
        #self.state_subscriber = self.create_subscription(State, 'state', self.state_callback, 10)
        self._action_client = ActionClient(self, Go, 'elevator_go')
        # create timer to call cb func every n time
        self.scheduled_timer = self.create_timer(1,self.publish_scheduled)
        self.do_next_timer = self.create_timer(1,self.do_next)
        
        self.do_next_flag = True
        self.time_record = time.time()


    def order_callback(self,msg):
        # update order lists
        order = [msg.floor,msg.door, anything.Anything]
        if msg.in_door :
            if not(order in self.in_door_scheduled):
                order[2] = time.time()  # add Arrival time
                self.in_door_scheduled.append(order)
        else:
            if (not(order in self.out_door_scheduled)):
                order[2] = time.time()  # add Arrival time
                self.out_door_scheduled.append(order)


    def publish_scheduled(self):
        """ removing time values and publishing scheduled msg as string """
        msg = Scheduled()
        msg.in_scheduled = str([x[0:2] for x in self.in_door_scheduled])
        msg.out_scheduled = str([x[0:2] for x in self.out_door_scheduled])
        self.scheduled_publisher.publish(msg)

    def get_next(self,In_scheduling_algorithm,Out_scheduling_algorithm,SJF_aging_value,door_to_floor_priority):
        """get nex goal based on scheduling algorithm"""
        # initial goal
        floor_goal = 1
        door_goal = 1
        
        # check if there is in door order (in door orders has higher priority than out door)
        # note that smallest priority value is higher priority
        if len(self.in_door_scheduled) > 0:
            if In_scheduling_algorithm == 'FCFS':
                priorities = [ (x[2]-time.time()) for x in self.in_door_scheduled]    # get waiting time in negative 
                index = priorities.index(min(priorities)) # get index of minimal value
                floor_goal,door_goal = self.in_door_scheduled[index][0],self.in_door_scheduled[index][1]

            elif In_scheduling_algorithm == 'SJF':
                # our equation to calculate priority : abs(f-cf)+ (1/dtfp)*abs(d-cd) + g(t-ct) >>> f:floor, cf:current floor, dtfp:door_to_floor_priority, d:door, cd:current door, g:SJF_aging_value, t:time, ct:current time
                cf,cd,ct,dtfp,g = self.last_state.floor,self.last_state.door,time.time(),door_to_floor_priority,SJF_aging_value
                priorities = [abs(x[0]-cf)+(1/dtfp)*abs(x[1]-cd)+g*(x[2]-ct) for x in self.in_door_scheduled]
                index = priorities.index(min(priorities)) # get index of minimal value
                floor_goal,door_goal = self.in_door_scheduled[index][0],self.in_door_scheduled[index][1]
        else:
            if Out_scheduling_algorithm == 'FCFS':
                time_difference = [x[2]-time.time() for x in self.out_door_scheduled]    # get waiting time in negative 
                index = time_difference.index(min(time_difference)) # get index of minimal value
                floor_goal,door_goal = self.out_door_scheduled[index][0],self.out_door_scheduled[index][1]

            elif Out_scheduling_algorithm == 'SJF':
                cf,cd,ct,dtfp,g = self.last_state.floor,self.last_state.door,time.time(),door_to_floor_priority,SJF_aging_value
                priorities = [abs(x[0]-cf)+(1/dtfp)*abs(x[1]-cd)+g*(x[2]-ct) for x in self.out_door_scheduled]
                index = priorities.index(min(priorities)) # get index of minimal value
                floor_goal,door_goal = self.out_door_scheduled[index][0],self.out_door_scheduled[index][1]

        return floor_goal,door_goal

    def do_next(self):
        """doing next goal if we can"""
        # check if we can do next
        There_next = (len(self.in_door_scheduled) > 0) or (len(self.out_door_scheduled) > 0)
        acceptable_waiting = (time.time() - self.time_record) >= self.get_parameter('elevator_waiting_time').value

        if (self.do_next_flag and acceptable_waiting and There_next):
            # lock doing next until current gaol is done
            self.do_next_flag = False

            # get the next gaol to do
            goal_msg = Go.Goal()
            goal_msg.floor , goal_msg.door = self.get_next(self.get_parameter('in_scheduling_algorithm').value,self.get_parameter('out_scheduling_algorithm').value,
                                                            self.get_parameter('SJF_aging_value').value,self.get_parameter('door_to_floor_priority').value)

            # sending gaol
            self.send_goal(goal_msg)

    def destroy_action_client(self):
        self._action_client.destroy()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return

        self.get_logger().info('Goal accepted!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0},{1}'.format(feedback.feedback.floor, feedback.feedback.door))

        # update last state
        self.last_state.floor = feedback.feedback.floor
        self.last_state.door = feedback.feedback.door

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal succeeded! Result: {result.floor},{result.door}')
        else:
            self.get_logger().info(f'Goal failed with status: {result.floor},{result.door}')
        
        # update last state
        self.last_state.floor = result.floor
        self.last_state.door = result.door

        # remove done order
        current_state = [result.floor,result.door,anything.Anything]
        if current_state in self.in_door_scheduled:
            self.in_door_scheduled.remove(current_state)
        if current_state in self.out_door_scheduled:
            self.out_door_scheduled.remove(current_state)

        # accept doing next jop
        self.do_next_flag = True
        # record current time for elevator waiting time
        self.time_record = time.time()

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

    def send_goal(self,goal_msg):

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'setting Goal: ({goal_msg.floor},{goal_msg.door})')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # if we need to cancel gaol:
        # future = self._goal_handle.cancel_goal_async()
        # future.add_done_callback(self.cancel_done)


def main(args=None):
    rclpy.init(args=args)

    scheduler = Scheduler()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(scheduler,executor)
    except KeyboardInterrupt:
        pass
    finally:
        scheduler.destroy_action_client()
        rclpy.try_shutdown()
        scheduler.destroy_node()

if __name__ == '__main__':
    main()
