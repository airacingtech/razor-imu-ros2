# Copyright 2022 AI Racing Tech
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

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    pkg_dir = get_package_share_directory('razor_imu_ros2')
    param_file_path = os.path.join(pkg_dir, 'param', 'razor_imu.param.yaml')

    razor_node = Node(
        package='razor_imu_ros2',
        executable='razor_imu_ros2_exe',
        output='screen',
        parameters=[param_file_path]
    )

    return LaunchDescription([
        razor_node
    ])
