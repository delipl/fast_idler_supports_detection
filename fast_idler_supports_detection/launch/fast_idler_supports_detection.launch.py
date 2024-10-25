#  Copyright 2024 Jakub Delicat
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="fast_idler_supports_detection",
                executable="fast_idler_supports_detection",
                output="screen",
                emulate_tty=True,
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
