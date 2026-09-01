import itertools

import pytest

from drims_homework.dice_kinematics import (
    grasp_yaw_for_target,
    needs_pick,
    opposite_face,
    plan_move,
    side_face_pairs,
)

ALL_FACES = range(1, 7)


@pytest.mark.parametrize("face,expected", [(1, 6), (6, 1), (2, 5), (5, 2), (3, 4), (4, 3)])
def test_opposite_face(face, expected):
    assert opposite_face(face) == expected


@pytest.mark.parametrize("face", ALL_FACES)
def test_side_face_pairs_excludes_own_axis(face):
    pair_a, pair_b = side_face_pairs(face)
    own_axis = {face, opposite_face(face)}

    assert set(pair_a).isdisjoint(own_axis)
    assert set(pair_b).isdisjoint(own_axis)
    # the four side faces, plus the die's own axis, must cover all six faces
    assert set(pair_a) | set(pair_b) | own_axis == set(ALL_FACES)


def test_needs_pick():
    for face in ALL_FACES:
        assert needs_pick(face, face) is False
    assert needs_pick(1, 2) is True
    assert needs_pick(6, 1) is True


@pytest.mark.parametrize("current,target", itertools.product(ALL_FACES, ALL_FACES))
def test_grasp_yaw_never_pinches_target(current, target):
    """Core correctness guarantee: the chosen yaw never covers target_face,
    unless target_face is current_face itself or its opposite (never pinched
    at any yaw, so no rotation is even needed there).
    """
    if target in (current, opposite_face(current)):
        return

    pair_a, pair_b = side_face_pairs(current)
    pinched_by_yaw = {0: pair_a, 90: pair_b}

    yaw = grasp_yaw_for_target(current, target)
    assert yaw in (0, 90)
    assert target not in pinched_by_yaw[yaw]


@pytest.mark.parametrize("current,target", itertools.product(ALL_FACES, ALL_FACES))
def test_plan_move_at_most_one_pick(current, target):
    plan = plan_move(current, target)

    assert plan["needs_pick"] == (current != target)
    if not plan["needs_pick"]:
        assert plan["grasp_yaw_deg"] == 0
    else:
        assert plan["grasp_yaw_deg"] in (0, 90)
