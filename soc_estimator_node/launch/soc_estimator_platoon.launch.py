from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    num_trucks_str = LaunchConfiguration('NumTrucks').perform(context)
    num_trucks = int(num_trucks_str)

    if num_trucks < 1:
        raise RuntimeError("NumTrucks must be >= 1")

    pkg_share = get_package_share_directory('soc_estimator_node')
    single_launch = os.path.join(pkg_share, 'launch', 'soc_estimator.launch.py')

    actions = []

    for i in range(num_trucks):
        truck_id = f"truck{i}"
        if i == 0:
            mode = "LV"
        else:
            # truck1 -> FV1, truck2 -> FV2, ...
            mode = f"FV{i}"

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(single_launch),
                launch_arguments={
                    'truck_id': truck_id,
                    'mode': mode,
                }.items()
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'NumTrucks',
            default_value='3'
        ),
        OpaqueFunction(function=launch_setup)
    ])
