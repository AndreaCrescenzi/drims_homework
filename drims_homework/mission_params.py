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

tilt_deg is handled separately (see _patch_tilt_deg below), not through
PARAM_PATTERNS: unlike the others it's not always present as an attribute
in dice_challenge.xml to begin with -- GetGraspOrientation's tilt_deg
input port is normally left unset so it auto-derives the tilt sign from
the upcoming rotation (see get_grasp_orientation.hpp). Valid values here:
'auto' (or unset/empty -- the default, geometry-derived sign, tilts the
pick ~45deg so the final placement can bring the die closer to the
table), '0' (a plain vertical approach, no tilt), or an explicit signed
angle like '45'/'-45' (mainly for diagnosing which sign a given
current/target face pair actually wants, or comparing a vertical vs.
tilted pick's reachability near the edge of the workspace).

place_x_vertical/place_y_vertical/place_z_vertical/lift_height_vertical
(see _resolve_vertical_overrides below): variants of the corresponding
base parameter, used ONLY when tilt_deg resolves to exactly 0. A
vertical approach releases the die from straight above and needs
different clearances than the ~45deg tilted approach the base
place_x/y/z/lift_height values are tuned for (see the "tilt_deg lets
placement bring the die closer to the table" comment above, and
dice_challenge.xml's own grasp comment) -- reusing the tilted-case
values for a vertical move risks the gripper hitting the table, or
missing a safe-zone position that was only verified reachable with the
tilted wrist configuration. None of the four are consumed by
PARAM_PATTERNS itself: when tilt_deg==0, each present '*_vertical' key
is substituted in for its base key BEFORE the generic loop runs, so it
still ends up patching the exact same XML attribute.
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


def _patch_tilt_deg(text: str, value) -> str:
    """Insert, override, or remove GetGraspOrientation's tilt_deg attribute.

    Unlike PARAM_PATTERNS' entries, tilt_deg isn't always present in the
    tree to begin with, so a plain value-substitution regex doesn't fit --
    this adds/removes the whole attribute instead.
    """
    # Strip any tilt_deg a previous patch_tree call left behind first, so
    # switching back to 'auto' (or to a different explicit value) doesn't
    # leave a stale attribute around or match the wrong occurrence below.
    text = re.sub(r'\s+tilt_deg="[-\d.]+"', '', text)

    if value is None:
        return text
    normalized = str(value).strip().lower()
    if normalized in ('', 'auto'):
        return text  # absent -> GetGraspOrientation auto-derives the sign

    pattern = re.compile(r'(<GetGraspOrientation\b[^>]*?)(/>)')
    text, count = pattern.subn(
        lambda m: f'{m.group(1)} tilt_deg="{value}"{m.group(2)}', text)
    if count == 0:
        raise RuntimeError(
            "Could not find a '<GetGraspOrientation .../>' tag to set "
            "tilt_deg on -- has dice_challenge.xml changed?")
    return text


def _is_vertical_tilt(tilt_deg) -> bool:
    normalized = str(tilt_deg).strip().lower() if tilt_deg is not None else 'auto'
    if normalized in ('', 'auto'):
        return False
    return float(normalized) == 0.0


VERTICAL_OVERRIDE_KEYS = ('place_x', 'place_y', 'place_z', 'lift_height')


def _resolve_vertical_overrides(params: dict) -> dict:
    """Swap in each '<key>_vertical' for '<key>' when tilt_deg==0.

    Returns a new dict (the caller's params is never mutated); pops every
    '*_vertical' key either way since none of them are XML attribute
    names PARAM_PATTERNS knows about -- only the base keys (the real
    target_x/target_y/target_z/z_offset attributes) should reach the
    generic loop in patch_tree().
    """
    params = dict(params)
    is_vertical = _is_vertical_tilt(params.get('tilt_deg'))
    for key in VERTICAL_OVERRIDE_KEYS:
        override = params.pop(f'{key}_vertical', None)
        if override is not None and is_vertical:
            params[key] = override
    return params


def patch_tree(tree_path: str, params: dict) -> None:
    """Rewrite dice_challenge.xml's mission parameters in place.

    params: a dict with any subset of the keys in PARAM_PATTERNS plus
    'tilt_deg' and the '*_vertical' overrides in VERTICAL_OVERRIDE_KEYS
    (missing keys are left untouched). Raises RuntimeError if a requested
    parameter's pattern isn't found in the file, so a future rename or
    refactor of dice_challenge.xml fails loudly here instead of silently
    launching with stale/wrong values.
    """
    params = _resolve_vertical_overrides(params)

    with open(tree_path, 'r') as f:
        text = f.read()

    for key, value in params.items():
        if key == 'tilt_deg':
            continue  # handled separately below
        if value is None:
            continue
        pattern, replacement_template, what = PARAM_PATTERNS[key]
        replacement = replacement_template.format(value=value)
        text, count = re.subn(pattern, replacement, text)
        if count == 0:
            raise RuntimeError(
                f"Could not find {what} in {tree_path} -- has "
                "dice_challenge.xml changed?")

    if 'tilt_deg' in params:
        text = _patch_tilt_deg(text, params['tilt_deg'])

    with open(tree_path, 'w') as f:
        f.write(text)
