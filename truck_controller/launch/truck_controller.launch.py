import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_controllers(context):
    num = int(LaunchConfiguration('num_trucks').perform(context))
    pkg = os.path.join(
      os.getcwd(), 'src', 'truck_controller', 'data')
    csv0 = os.path.join(pkg, '0.csv')

    nodes = []
    for i in range(num):
        nodes.append(Node(
            package='truck_controller',
            executable='truck_controller_node',
            name=f'controller{i}',
            output='screen',
            arguments=[str(i), f'controller{i}'],
            parameters=[{
              'csv_path':     csv0,
              'lookahead_dist': 3.0,
              'max_speed':      2.5,
              'k_angular':      1.0
            }]
        ))
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_trucks', default_value='1',
                               description='Number of trucks'),
        OpaqueFunction(function=generate_controllers)
    ])

