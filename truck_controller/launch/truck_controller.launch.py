import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_controllers(context):
    num = int(LaunchConfiguration('NumTrucks').perform(context))
    pkg = os.path.join(
      os.getcwd(), 'src', 'truck_controller', 'data')
    csv = os.path.join(pkg, '')

    nodes = []
    for i in range(num):
        nodes.append(Node(
            package='truck_controller',
            executable='truck_controller_node',
            name=f'truck_controller_node_{i}',
            output='screen',
            emulate_tty=True,
            arguments=[str(i)],
            parameters=[{
              'csv_path':     csv,
              'curve_lookahead_dist': 5.0,
              'straight_lookahead_dist': 7.0,
              'ACC_SPEED':     60.0,
              'SLOW_SPEED':      30.0,
              'STABLE_SPEED':      50.0,   
              'WHEEL_BASE':      16.0,
              'DESIRED_GAP':      16.8,
              'MIN_GAP':          14.0,
              'EMERGENCY_GAP':      5.0,

 
            }]
        ))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('NumTrucks', default_value='1',
                             description='Number of trucks'),
        OpaqueFunction(function=generate_controllers)
    ])
