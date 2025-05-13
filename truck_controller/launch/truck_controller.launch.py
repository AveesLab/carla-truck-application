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
              'lookahead_dist': 10.0,
              'max_speed':      0.3,
              'wheel_base':      5.0,

 
            }]
        ))
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('NumTrucks', default_value='1',
                               description='Number of trucks'),
        OpaqueFunction(function=generate_controllers)
    ])

