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

"""Shared logic for patching dice_challenge.xml's mission parameters.

dice_challenge.xml has no ROS-parameter input for target_face, the final
placement pose, or the pre-rotation lift height -- they're set via a
<Script code="target_face:=N"/> at the top of MainTree and attributes on
the two ComputeXYCorrection nodes, read by bt_executer_node straight from
the tree file. Rather than adding new BT nodes just to read a handful of
ROS parameters into the blackboard, both dice_mission.launch.py (manual
CLI arguments) and dice_mission_from_config.launch.py (a YAML file) call
patch_tree() here to rewrite those values in the INSTALLED copy of
dice_challenge.xml before launching -- colcon build here doesn't
symlink-install, so the installed copy is a plain file safe to overwrite,
and the source in the repo is never touched.

place_x/place_y patch BOTH ComputeXYCorrection calls (the pre-rotation
relocation away from the cell's camera-mount crossbar, and the final
placement) -- deliberately kept at the same point by design (see
dice_challenge.xml's own comments), so moving one without the other would
defeat that. place_z only patches the final placement's target_z (the
pre-rotation relocation uses a relative clearance lift, lift_height, not
an absolute target -- not something "final pose" means).

Every regex matches the attribute NAME, not today's literal value, so it
stays correct across repeated launches with different values (a naive
search-and-replace on specific numbers would silently stop matching after
the first launch that changed them).
"""

import re

PARAM_PATTERNS = {
    'target_face': (r'target_face:=\d+', "target_face:={value}",
                     "a 'target_face:=N' assignment"),
    'place_x': (r'target_x="[-\d.]+"', 'target_x="{value}"',
                "a 'target_x=\"...\"' attribute"),
    'place_y': (r'target_y="[-\d.]+"', 'target_y="{value}"',
                "a 'target_y=\"...\"' attribute"),
    'place_z': (r'target_z="[-\d.]+"', 'target_z="{value}"',
                "a 'target_z=\"...\"' attribute"),
    # lift_height: the pre-rotation relocation's z_offset -- how far the
    # die is lifted, while being moved to the safe zone, before rotating.
    # Lower values mean the arm rises less, but too low risks reopening
    # NO_IK_SOLUTION on the rotation itself (an in-place ~90deg
    # reorientation needs *some* clearance from the table -- see
    # ReorientCycle's own history in dice_challenge.xml). Not verified
    # below 0.10m with the corrected grasp geometry; test on RViz before
    # trusting a lower value.
    'lift_height': (r'z_offset="[-\d.]+"', 'z_offset="{value}"',
                     "a 'z_offset=\"...\"' attribute"),
}


def patch_tree(tree_path: str, params: dict) -> None:
    """Rewrite dice_challenge.xml's mission parameters in place.

    params: a dict with any subset of the keys in PARAM_PATTERNS (missing
    keys are left untouched). Raises RuntimeError if a requested
    parameter's pattern isn't found in the file, so a future rename or
    refactor of dice_challenge.xml fails loudly here instead of silently
    launching with stale/wrong values.
    """
    with open(tree_path, 'r') as f:
        text = f.read()

    for key, value in params.items():
        if value is None:
            continue
        pattern, replacement_template, what = PARAM_PATTERNS[key]
        replacement = replacement_template.format(value=value)
        text, count = re.subn(pattern, replacement, text)
        if count == 0:
            raise RuntimeError(
                f"Could not find {what} in {tree_path} -- has "
                "dice_challenge.xml changed?")

    with open(tree_path, 'w') as f:
        f.write(text)
