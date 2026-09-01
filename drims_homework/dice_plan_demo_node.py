"""Quick spike: read the real face from the simulated die and show what
dice_kinematics.plan_move() would decide, without moving the robot yet.

Usage (with the simulator + dice already spawned, see TESTING.md):
    ros2 run drims_homework dice_plan_demo_node --ros-args -p target_face:=5
"""

import rclpy
from rclpy.node import Node
from easy_motion_msgs.srv import DiceIdentification

from drims_homework.dice_kinematics import plan_move


def main():
    rclpy.init()
    node = Node('dice_plan_demo_node')
    node.declare_parameter('target_face', 3)
    target_face = node.get_parameter('target_face').get_parameter_value().integer_value

    client = node.create_client(DiceIdentification, 'dice_identification')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('dice_identification service not available')
        node.destroy_node()
        rclpy.shutdown()
        return

    future = client.call_async(DiceIdentification.Request())
    rclpy.spin_until_future_complete(node, future)
    result = future.result()

    if result is None or not result.success:
        node.get_logger().error('Dice identification failed')
        node.destroy_node()
        rclpy.shutdown()
        return

    current_face = result.face_number
    plan = plan_move(current_face, target_face)

    node.get_logger().info(
        f'Current face read from simulator: {current_face}'
    )
    node.get_logger().info(
        f'Target face (param): {target_face}'
    )

    if not plan['needs_pick']:
        node.get_logger().info(
            'plan_move() says: die already shows the target face, no move needed.'
        )
    else:
        node.get_logger().info(
            f"plan_move() says: pick with grasp_yaw={plan['grasp_yaw_deg']} deg, "
            f'then rotate to expose face {target_face}.'
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
