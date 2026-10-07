#!/usr/bin/env python3
"""native_joint_trajectory_controller example."""

import math
import os
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from example_common import (  # noqa: E402
    DURATION_S,
    JOINTS,
    activate_exclusive,
    cleanup_motion_controllers,
    ensure_servo,
    joint5_moved_deg,
    read_joint_positions,
    report_joint5_motion,
    wait_controller,
    wait_msg,
)
from sensor_msgs.msg import JointState  # noqa: E402

OFFSET_DEG = 5.0

ACTION_NAME = 'native_joint_trajectory_controller/follow_joint_trajectory'


def build_trajectory(start, target):
    traj = JointTrajectory()
    traj.joint_names = list(JOINTS)
    p0 = JointTrajectoryPoint()
    p0.positions = list(start)
    p0.time_from_start = Duration(sec=0, nanosec=0)
    p1 = JointTrajectoryPoint()
    p1.positions = list(target)
    p1.time_from_start = Duration(sec=int(DURATION_S), nanosec=0)
    traj.points = [p0, p1]
    return traj


def wait_action_server(node, client, timeout=15.0):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if client.wait_for_server(timeout_sec=0.0):
            return True
        rclpy.spin_once(node, timeout_sec=0.2)
    return False


def main():
    rclpy.init()
    node = Node('native_jtc_example')
    activated = False
    try:
        q = read_joint_positions(node)
        if q is None:
            node.get_logger().error('no driver status — is bringup running on real hardware?')
            return 1
        if not wait_controller(node, 'native_joint_trajectory_controller'):
            node.get_logger().error('native_joint_trajectory_controller is not loaded yet')
            return 1

        if not ensure_servo(node):
            return 1

        client = ActionClient(node, FollowJointTrajectory, ACTION_NAME)

        if not activate_exclusive(node, 'native_joint_trajectory_controller'):
            node.get_logger().error('could not switch to native_joint_trajectory_controller')
            return 1
        activated = True

        if not wait_action_server(node, client):
            node.get_logger().error(
                f'action {ACTION_NAME} is not up — rebuild symphony_controllers and check '
                'bringup log for NativeJointTrajectoryController configured')
            return 1

        start = [q[j] for j in JOINTS]
        target = list(start)
        target[-1] += math.radians(OFFSET_DEG)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = build_trajectory(start, target)

        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send, timeout_sec=5.0)
        if not send.done():
            node.get_logger().error('goal send timed out')
            return 1
        handle = send.result()
        if handle is None or not handle.accepted:
            node.get_logger().error('goal rejected')
            return 1
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(node, done, timeout_sec=DURATION_S + 60.0)
        if not done.done():
            node.get_logger().error('trajectory timed out')
            return 1
        wrapped = done.result()
        action_ok = wrapped is not None and wrapped.result.error_code == 0
        if not action_ok and wrapped is not None:
            node.get_logger().error(
                f'trajectory failed: code={wrapped.result.error_code} '
                f'msg={wrapped.result.error_string}')

        after = wait_msg(node, '/joint_states', JointState, timeout=1.0)
        if after is None:
            node.get_logger().error('no joint_states after trajectory')
            return 1
        q_after = {n: p for n, p in zip(after.name, after.position)}
        moved_deg = joint5_moved_deg(q, q_after)
        ok = report_joint5_motion(
            node, 'native_jtc', moved_deg, OFFSET_DEG, action_ok=action_ok)
        return 0 if ok else 1
    finally:
        if activated:
            cleanup_motion_controllers(node, ['native_joint_trajectory_controller'])
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
