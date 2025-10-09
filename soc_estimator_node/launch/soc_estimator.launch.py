from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    truck_id = LaunchConfiguration('truck_id').perform(context)
    mode = LaunchConfiguration('mode').perform(context)

    return [
        Node(
            package='soc_estimator_node',
            executable='soc_estimator_node_exec',
            name=f'{truck_id}_soc_estimator',
            parameters=[{
                'truck_id': truck_id, 
                'mode': mode,
                'use_sim_time': True,
                }
            ],
            output='screen'
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('truck_id', default_value='truck0'),
        DeclareLaunchArgument('mode', default_value='LV'),
        OpaqueFunction(function=launch_setup)
    ])

