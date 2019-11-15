# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch a talker and a listener in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    """Generate launch description with multiple components."""
    ros2_serial_example_directory = os.path.join(
        get_package_share_directory('ros2_serial_example'), 'config')

    param_config = os.path.join(ros2_serial_example_directory,
                                'default_ros2_to_serial_bridge_params.yaml')

    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['iris_0/ros2_to_serial_bridge']['ros__parameters']

    print(params)

    container = ComposableNodeContainer(
            node_name='componsable_drone_node',
            node_namespace='/iris_0',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    node_name='ros2_to_serial_bridge',
                    node_namespace='/iris_0',
                    package='ros2_serial_example',
                    node_plugin='ros2_to_serial_bridge::ROS2ToSerialBridge',
                    parameters=[params]),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
