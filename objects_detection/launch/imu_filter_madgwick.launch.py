import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("objects_detection"), "config"
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter",
                output="screen",
                parameters=[os.path.join(config_dir, "imu_filter.yaml")],
                remappings=[
                    ("/imu/data_raw", "/camera/imu_fixed_time"),
                    ("/imu/data", "/camera/imu_madgwick"),
                ],
            ),
            launch_ros.actions.Node(
                package="objects_detection",
                executable="imu_time_fixer",
                name="imu_filter",
                output="screen",
            ),
        ]
    )
