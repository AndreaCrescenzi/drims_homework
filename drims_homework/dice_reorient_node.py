"""Python spike that actually executes a dice_kinematics.plan_move() plan on
the (simulated) robot: pick the die with a grasp yaw chosen to keep
target_face uncovered, apply the exact rotation read from TF between
face{current}_tf and face{target}_tf, then place it back down.

This is a fast prototype to validate the geometry end-to-end before porting
the same logic into the GetFaceRotation BT node (C++). See TESTING.md and
the plan in .claude/plans for context.

Usage (with the simulator + dice already spawned):
    ros2 run drims_homework dice_reorient_node --ros-args -p target_face:=5
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from easy_motion.motion_client import MotionClient
from easy_motion_msgs.srv import DiceIdentification
from moveit_msgs.msg import MoveItErrorCodes
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler, quaternion_multiply

from drims_homework.dice_kinematics import plan_move

GRIPPER_ACTION_NAME = '/gripper_action_controller/gripper_cmd'
GRIPPER_OPEN = 0.045
GRIPPER_CLOSED = 0.0

# Same fixed "point straight down" pick orientation used by demo_node.py,
# expressed relative to dice_tf (the frame aligned with whichever face is
# currently up). grasp_yaw rotates this about dice_tf's own Z axis.
BASE_PICK_ORIENTATION = (1.0, 0.0, 0.0, 0.0)


def yaw_quaternion(deg: float):
    return quaternion_from_euler(0.0, 0.0, math.radians(deg))


def spin_for(node, seconds: float):
    """Pump callbacks (incl. TF) for a bit without a background executor thread."""
    end_time = node.get_clock().now() + Duration(seconds=seconds)
    while node.get_clock().now() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    rclpy.init()

    motion_client = MotionClient(gripper_action_name=GRIPPER_ACTION_NAME)
    node = rclpy.create_node('dice_reorient_node')
    logger = node.get_logger()

    node.declare_parameter('target_face', 3)
    target_face = node.get_parameter('target_face').get_parameter_value().integer_value

    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    dice_client = node.create_client(DiceIdentification, 'dice_identification')
    if not dice_client.wait_for_service(timeout_sec=5.0):
        logger.error('dice_identification service not available')
        return

    def identify():
        future = dice_client.call_async(DiceIdentification.Request())
        rclpy.spin_until_future_complete(node, future)
        return future.result()

    result = identify()
    if result is None or not result.success:
        logger.error('Dice identification failed')
        return

    current_face = result.face_number
    plan = plan_move(current_face, target_face)
    logger.info(f'current_face={current_face} target_face={target_face} plan={plan}')

    if not plan['needs_pick']:
        logger.info('Already showing the target face, nothing to do.')
        motion_client.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        return

    # --- Pick, with grasp yaw chosen so target_face stays uncovered ---
    pick_orientation = quaternion_multiply(
        yaw_quaternion(plan['grasp_yaw_deg']), BASE_PICK_ORIENTATION)

    pick_pose = PoseStamped()
    pick_pose.header.frame_id = 'dice_tf'
    pick_pose.pose.orientation.x = pick_orientation[0]
    pick_pose.pose.orientation.y = pick_orientation[1]
    pick_pose.pose.orientation.z = pick_orientation[2]
    pick_pose.pose.orientation.w = pick_orientation[3]

    logger.info(f"Moving to pick pose (grasp_yaw={plan['grasp_yaw_deg']} deg)...")
    res = motion_client.move_to_pose(pick_pose, cartesian_motion=False)
    if res.val != MoveItErrorCodes.SUCCESS:
        logger.error(f'Failed to reach pick pose: {res.val}')
        return

    logger.info('Closing gripper and attaching...')
    motion_client.gripper_command(position=GRIPPER_CLOSED)
    motion_client.attach_object('dice', 'tool0')

    # Give the TF listener a moment to fill its buffer before looking up.
    spin_for(node, 1.0)

    try:
        transform = tf_buffer.lookup_transform(
            f'face{target_face}_tf', f'face{current_face}_tf', Time())
    except Exception as exc:  # noqa: BLE001 - just want to log and bail out
        logger.error(f'TF lookup face{current_face}_tf <- face{target_face}_tf failed: {exc}')
        return

    rotate_pose = PoseStamped()
    rotate_pose.header.frame_id = f'face{current_face}_tf'
    # A pure in-place reorientation (zero translation) can fall outside the
    # wrist's reachable orientation set (NO_IK_SOLUTION); a small lift along
    # face{current_face}_tf's own Z (which coincides with world-up while
    # that face is up) gives IK enough room to find a solution.
    rotate_pose.pose.position.z = 0.10
    rotate_pose.pose.orientation = transform.transform.rotation

    logger.info('Applying rotation to expose target face...')
    res = motion_client.move_to_pose(rotate_pose, cartesian_motion=False, relative_motion=True)
    if res.val != MoveItErrorCodes.SUCCESS:
        logger.error(f'Failed to rotate: {res.val}')
        return

    logger.info('Opening gripper and detaching...')
    motion_client.gripper_command(position=GRIPPER_OPEN)
    motion_client.detach_object('dice')

    result = identify()
    if result is not None and result.success:
        logger.info(
            f'Face after reorientation: {result.face_number} (target was {target_face})'
        )
    else:
        logger.warn('Could not re-identify the die after reorientation.')

    motion_client.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
