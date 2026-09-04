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

# Launches the dice_challenge mission with every mission parameter
# selectable from the command line, e.g.:
#
#   ros2 launch drims_homework dice_mission.launch.py target_face:=3 \
#       place_x:=0.5 place_y:=0.0 place_z:=0.03 lift_height:=0.08
#
# For a YAML-file-driven alternative (edit a file instead of typing
# arguments every time), see dice_mission_from_config.launch.py. Both
# call the same patch_tree() helper (drims_homework/mission_params.py)
# to rewrite dice_challenge.xml's values before launching -- see that
# module's docstring for why this rewrites the tree file instead of
# passing ROS parameters.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from drims_homework.mission_params import patch_tree


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('drims_homework')
    tree_path = pkg_dir + '/trees/dice_challenge.xml'
    config_path = pkg_dir + '/config/dice_challenge_config.yaml'

    patch_tree(tree_path, {
        'target_face': LaunchConfiguration('target_face').perform(context),
        'place_x': LaunchConfiguration('place_x').perform(context),
        'place_y': LaunchConfiguration('place_y').perform(context),
        'place_z': LaunchConfiguration('place_z').perform(context),
        'lift_height': LaunchConfiguration('lift_height').perform(context),
        'tilt_deg': LaunchConfiguration('tilt_deg').perform(context),
        'place_x_vertical': LaunchConfiguration('place_x_vertical').perform(context),
        'place_y_vertical': LaunchConfiguration('place_y_vertical').perform(context),
        'place_z_vertical': LaunchConfiguration('place_z_vertical').perform(context),
        'lift_height_vertical': LaunchConfiguration('lift_height_vertical').perform(context),
    })

    bt_executer_node = Node(
        package='easy_motion_behavior_tree',
        executable='bt_executer_node',
        name='bt_executer_node',
        output='screen',
        parameters=[config_path],
    )

    return [bt_executer_node]


def generate_launch_description():
    target_face_arg = DeclareLaunchArgument(
        'target_face',
        default_value='1',
        description='Face number (1-6) the die should end up showing.')
    place_x_arg = DeclareLaunchArgument(
        'place_x',
        default_value='0.365',
        description='Final placement X, in base_link (also used for the '
                     'pre-rotation relocation away from the camera bar). '
                     '0.45 also clears the bar but left some post-rotation '
                     'wrist configurations unable to reach it in a single '
                     'straight-line placement move (observed on 3->5); '
                     '0.55 verified clear of the bar and more reachable in '
                     'that case. Not exhaustively verified for every '
                     'current/target pair -- if placement starts failing '
                     'after changing this, try a nearby value.')
    place_y_arg = DeclareLaunchArgument(
        'place_y',
        default_value='0.265',
        description='Final placement Y, in base_link (also used for the '
                     'pre-rotation relocation away from the camera bar).')
    place_z_arg = DeclareLaunchArgument(
        'place_z',
        default_value='0.012',
        description="Final placement Z, in base_link -- the die's own "
                     'resting height when flat on the table.')
    lift_height_arg = DeclareLaunchArgument(
        'lift_height',
        default_value='0.035',
        description='How far (m) the die is lifted while being relocated '
                     'to the safe zone, before rotating. Lower means the '
                     'arm rises less, but too low risks NO_IK_SOLUTION on '
                     'the in-place rotation itself. 0.07 verified working '
                     '(with the corrected grasp geometry) on an opposite-'
                     'face, two-cycle rotation; not exhaustively tested '
                     'below that -- watch RViz closely the first time you '
                     'lower this further.')

    tilt_deg_arg = DeclareLaunchArgument(
        'tilt_deg',
        default_value='auto',
        description="Pick approach angle about the gripper's closing "
                     "axis. 'auto' (default): geometry-derived sign, "
                     '~45deg tilted approach -- lets the final placement '
                     "bring the die closer to the table than a vertical "
                     "approach would. '0': plain vertical approach (no "
                     "tilt). '45'/'-45' (or any value): explicit "
                     'override, mainly for diagnosing which sign a given '
                     'current/target face pair wants, or comparing '
                     'vertical vs. tilted reachability near the edge of '
                     'the workspace.')
    place_x_vertical_arg = DeclareLaunchArgument(
        'place_x_vertical',
        default_value='0.365',
        description='place_x variant used ONLY when tilt_deg resolves to '
                     'exactly 0. NOT YET EMPIRICALLY VERIFIED -- defaults '
                     'to the same value as place_x.')
    place_y_vertical_arg = DeclareLaunchArgument(
        'place_y_vertical',
        default_value='0.265',
        description='place_y variant used ONLY when tilt_deg resolves to '
                     'exactly 0. NOT YET EMPIRICALLY VERIFIED -- defaults '
                     'to the same value as place_y.')
    place_z_vertical_arg = DeclareLaunchArgument(
        'place_z_vertical',
        default_value='0.012',
        description='Release height used ONLY when tilt_deg resolves to '
                     'exactly 0 (a vertical approach needs a bit more '
                     'clearance above the table at release than the '
                     'tilted approach place_z is tuned for). ABSOLUTE '
                     'target, not added to lift_height_vertical -- keep '
                     'it noticeably below place_z + lift_height_vertical '
                     'or the placement move rises instead of descending. '
                     'NOT YET EMPIRICALLY VERIFIED -- a starting guess; '
                     'watch RViz closely the first time a vertical '
                     'placement runs.')
    lift_height_vertical_arg = DeclareLaunchArgument(
        'lift_height_vertical',
        default_value='0.035',
        description='lift_height variant used ONLY when tilt_deg '
                     'resolves to exactly 0. NOT YET EMPIRICALLY '
                     'VERIFIED -- defaults to the earlier-verified 0.07 '
                     'rather than the more aggressive 0.05 now used for '
                     'the tilted case.')

    return LaunchDescription([
        target_face_arg,
        place_x_arg,
        place_y_arg,
        place_z_arg,
        lift_height_arg,
        tilt_deg_arg,
        place_x_vertical_arg,
        place_y_vertical_arg,
        place_z_vertical_arg,
        lift_height_vertical_arg,
        OpaqueFunction(function=launch_setup),
    ])
