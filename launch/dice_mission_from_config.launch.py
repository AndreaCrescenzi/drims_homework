# Copyright 2024 National Research Council STIIMA
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

# Launches the dice_challenge mission with parameters read from a YAML
# file (config/dice_mission_params.yaml by default) instead of typed on
# the command line every time:
#
#   ros2 launch drims_homework dice_mission_from_config.launch.py
#
# Edit config/dice_mission_params.yaml, save, relaunch -- no arguments
# needed. To use a different file (e.g. a few saved presets), pass
# config_file:
#
#   ros2 launch drims_homework dice_mission_from_config.launch.py \
#       config_file:=/path/to/my_params.yaml
#
# For the plain command-line-arguments alternative (quicker for a single
# one-off override without touching a file), see dice_mission.launch.py.
# Both call the same patch_tree() helper
# (drims_homework/mission_params.py) to rewrite dice_challenge.xml's
# values before launching -- see that module's docstring for why this
# rewrites the tree file instead of passing ROS parameters.

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from drims_homework.mission_params import patch_tree


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('drims_homework')
    tree_path = pkg_dir + '/trees/dice_challenge.xml'
    bt_config_path = pkg_dir + '/config/dice_challenge_config.yaml'

    params_path = LaunchConfiguration('config_file').perform(context)
    with open(params_path, 'r') as f:
        params = yaml.safe_load(f) or {}

    patch_tree(tree_path, params)

    bt_executer_node = Node(
        package='easy_motion_behavior_tree',
        executable='bt_executer_node',
        name='bt_executer_node',
        output='screen',
        parameters=[bt_config_path],
    )

    return [bt_executer_node]


def generate_launch_description():
    pkg_dir = get_package_share_directory('drims_homework')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=pkg_dir + '/config/dice_mission_params.yaml',
        description='Path to a YAML file with target_face/place_x/'
                     'place_y/place_z/lift_height (any subset -- see '
                     'config/dice_mission_params.yaml for the format).')

    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=launch_setup),
    ])
