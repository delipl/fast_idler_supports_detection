import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="objects_detection",
                executable="object_detection",
                output="screen",
                emulate_tty=True,
                # output='screen',
                # prefix=["gdbserver localhost:8009"],
                parameters=[{"general.debug": "1"}],
            ),
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="point_cloud_tf",
                output="log",
                arguments=[
                    "0.410",
                    "0",
                    "1.350",
                    "0",
                    "0.454",
                    "0",
                    "base_link",
                    "velodyne_link",
                ],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
