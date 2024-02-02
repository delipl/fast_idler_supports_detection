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
                prefix=["gdbserver localhost:8009"],
                parameters=[
                    {'debug': True}
                ]
            ),
        ]
    )
