from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_elevator',
            executable='HW_interface.py',
            parameters=[
              {'arduino_port': '/dev/ttyACM0'},
              {'floor_pos': [0.13,0.3,0.49,0.67]},
              {"door_pos": [0,90,180]},
              {'HW_update_time': 1.0},
              {'max_pwm': 200},
              {'Kp': 3500.0},
              {"Ki": 0.09},
              {"Kd": 8700.0}
            ]
        ),

        Node(
            package='simple_elevator',
            executable='scheduler.py',
            parameters=[
              {'elevator_waiting_time': 2},
              {'in_scheduling_algorithm': 'SJF'},
              {'out_scheduling_algorithm': 'FCFS'},
              {'SJF_aging_value': 0.1},
              {'door_to_floor_priority': 5}
            ]
        ),

        Node(
            package='simple_elevator',
            executable='GUI.py'
        ),

        Node(
            package='simple_elevator',
            executable='android_interface.py'
        )
    ])