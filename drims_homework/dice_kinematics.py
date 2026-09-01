"""Pure geometric logic for the DRIMS2 dice challenge.

No ROS2 dependency on purpose: this module answers "given the die's current
face and the requested target face, do we need to move at all and, if so,
with which grasp yaw" using only the die's face layout. It is meant to be
called from the GetFaceRotation BT node (or its C++ port) rather than
duplicated there.

Face numbering and local-frame normals match drims2_dice_simulator's
dice_spawner.py `face_normals` (opposite faces sum to 7, as on a standard
die): 1 <-> -z, 6 <-> +z, 2 <-> -x, 5 <-> +x, 3 <-> +y, 4 <-> -y.
"""

FACE_NORMALS = {
    1: (0, 0, -1),
    2: (-1, 0, 0),
    3: (0, 1, 0),
    4: (0, -1, 0),
    5: (1, 0, 0),
    6: (0, 0, 1),
}

_AXIS_PAIRS = ((1, 6), (2, 5), (3, 4))

# Which of the two side-face pairs returned by side_face_pairs() is pinched
# by the gripper at yaw=0 depends on how the pick pose orientation is wired
# to the physical gripper fingers (see the pick pose used by GetFaceRotation
# / the BT tree). Not yet verified against the simulator/robot: flip to 1
# if grasp_yaw_for_target turns out backwards in practice (see TESTING.md).
YAW0_PINCHES_PAIR_INDEX = 0


def opposite_face(face: int) -> int:
    """Return the face opposite to `face` (they always sum to 7)."""
    return 7 - face


def side_face_pairs(current_face: int) -> tuple:
    """Return the two opposite-face pairs that are *not* current_face's axis.

    These are the four "equator" faces around the die when current_face is
    up: current_face itself and its opposite are excluded, since a
    horizontally-closing gripper never pinches them.
    """
    current_pair = {current_face, opposite_face(current_face)}
    side_pairs = tuple(pair for pair in _AXIS_PAIRS if set(pair) != current_pair)
    assert len(side_pairs) == 2, f"unexpected face layout for face {current_face}"
    return side_pairs


def needs_pick(current_face: int, target_face: int) -> bool:
    """Whether a pick-rotate-place cycle is needed at all."""
    return current_face != target_face


def grasp_yaw_for_target(current_face: int, target_face: int) -> int:
    """Pick a grasp yaw (0 or 90 degrees) so the gripper never covers target_face.

    A parallel gripper closing at yaw=0 pinches one pair of side faces; at
    yaw=90 it pinches the other pair (see YAW0_PINCHES_PAIR_INDEX). Since
    current_face and its opposite are never pinched regardless of yaw, this
    only has to actively choose when target_face is one of the four side
    faces. Combined with needs_pick(), this guarantees at most one
    pick-rotate-place cycle for any (current_face, target_face) pair.

    Returns 0 or 90; when target_face is current_face or its opposite,
    either yaw works and 0 is returned.
    """
    pairs = side_face_pairs(current_face)
    pinched_at_yaw0 = pairs[YAW0_PINCHES_PAIR_INDEX]
    pinched_at_yaw90 = pairs[1 - YAW0_PINCHES_PAIR_INDEX]

    if target_face in pinched_at_yaw0:
        return 90
    if target_face in pinched_at_yaw90:
        return 0
    # target_face is current_face itself or its opposite.
    return 0


def plan_move(current_face: int, target_face: int) -> dict:
    """Full move plan for going from current_face to target_face.

    Returns {"needs_pick": bool, "grasp_yaw_deg": int}.
    """
    if not needs_pick(current_face, target_face):
        return {"needs_pick": False, "grasp_yaw_deg": 0}
    return {
        "needs_pick": True,
        "grasp_yaw_deg": grasp_yaw_for_target(current_face, target_face),
    }
