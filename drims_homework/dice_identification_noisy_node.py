import math
import random

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from easy_motion_msgs.srv import DiceIdentification

# Faces whose pip pattern has exact 90deg rotational symmetry -- a single
# top-down 2D image cannot tell the true yaw from the 3 other 90deg-step
# hypotheses (face 1's single center pip is symmetric under ANY rotation;
# faces 4/5's four-corner layout is symmetric under 90deg steps). Confirmed
# against the simulator's actual pip layout (drims_dice_simulator's
# dice_spawner.py face_layouts) and against the plan/project memory
# recorded for this challenge -- not guessed. Faces 2/3/6 only have 180deg
# symmetry, which a top-down view's square silhouette (mod 90deg) already
# resolves down to an unambiguous yaw, so they're left untouched here.
AMBIGUOUS_FACES = (1, 4, 5)
HYPOTHESIS_DEG_CHOICES = (0, 90, 180, 270)


def _quat_mul(q1, q2):
    """Hamilton product q1 (x,y,z,w) * q2 (x,y,z,w) -> (x,y,z,w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _z_rotation_quat(deg):
    half = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


class DiceIdentificationNoisy(Node):
    """Development/test stand-in for a real top-down vision pipeline.

    Wraps the simulator's real (ground-truth) /dice_identification and
    serves /dice_identification_noisy with the same interface, but for
    AMBIGUOUS_FACES it replaces the true orientation with one of the 4
    physically-indistinguishable 90deg-step hypotheses -- simulating
    exactly what a real 2D top-down camera could actually measure, so the
    tip-and-relook disambiguation logic (RobustDiceIdentification) can be
    developed and tested against a known ground truth without a real
    camera. face_number itself is always the true one: pip counting is not
    the ambiguous part, yaw is.

    The true orientation is deliberately NEVER exposed on this service's
    response for an ambiguous face -- code under test must not be able to
    cheat by reading it back out.
    """

    def __init__(self):
        super().__init__('dice_identification_noisy_node')

        self.declare_parameter('forced_hypothesis_deg', -1)
        # -1 (default): pick one of the 4 hypotheses at random on every
        # call -- realistic, but non-reproducible. Set to 0/90/180/270 to
        # force that specific hypothesis every time, for deterministic
        # per-case testing (12 cases total: 3 ambiguous faces x 4 true
        # yaws, one at a time).

        client_group = MutuallyExclusiveCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()
        # Client and service MUST be in different callback groups: the
        # service callback below blocks on self.client.call(), which only
        # returns once the executor processes the client's response
        # callback -- if both were in the same (or the node's default)
        # group, a MultiThreadedExecutor would still serialize them on one
        # thread and this would deadlock forever. Two groups + a
        # MultiThreadedExecutor (see main()) let the response be handled
        # on a second thread while this one is blocked waiting.
        self.client = self.create_client(
            DiceIdentification, 'dice_identification', callback_group=client_group)
        self.srv = self.create_service(
            DiceIdentification, 'dice_identification_noisy', self._callback,
            callback_group=service_group)

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /dice_identification...')

        self.get_logger().info('dice_identification_noisy ready.')

    def _callback(self, request, response):
        real_response = self.client.call(DiceIdentification.Request())

        response.face_number = real_response.face_number
        response.success = real_response.success
        response.pose = real_response.pose

        if not real_response.success:
            return response

        if real_response.face_number in AMBIGUOUS_FACES:
            forced = self.get_parameter(
                'forced_hypothesis_deg').get_parameter_value().integer_value
            hyp_deg = (
                forced if forced in HYPOTHESIS_DEG_CHOICES
                else random.choice(HYPOTHESIS_DEG_CHOICES))

            q_true = (
                real_response.pose.pose.orientation.x,
                real_response.pose.pose.orientation.y,
                real_response.pose.pose.orientation.z,
                real_response.pose.pose.orientation.w,
            )
            # World-frame (extrinsic) rotation about Z: pre-multiply.
            qx, qy, qz, qw = _quat_mul(_z_rotation_quat(hyp_deg), q_true)
            response.pose.pose.orientation.x = qx
            response.pose.pose.orientation.y = qy
            response.pose.pose.orientation.z = qz
            response.pose.pose.orientation.w = qw

            self.get_logger().info(
                f'Face {real_response.face_number} has 90deg pip symmetry: '
                f'reporting a yaw {hyp_deg}deg off the true value (true '
                'value withheld -- this simulates what a real top-down '
                'camera could actually measure).')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DiceIdentificationNoisy()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
