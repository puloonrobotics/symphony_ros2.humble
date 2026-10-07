#!/usr/bin/env python3
"""joint_trajectory_controller example."""

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
    OFFSET_DEG,
    WAYPOINT_STEP_DEG,
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


def _duration(seconds):
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


def build_trajectory(start, target):
    traj = JointTrajectory()
    traj.joint_names = list(JOINTS)
    max_delta_deg = max(
        abs(math.degrees(t - s)) for s, t in zip(start, target))
    n_segments = max(1, int(math.ceil(max_delta_deg / WAYPOINT_STEP_DEG)))
    points = []
    for i in range(n_segments + 1):
        alpha = i / n_segments
        point = JointTrajectoryPoint()
        point.positions = [s + alpha * (t - s) for s, t in zip(start, target)]
        point.time_from_start = _duration(alpha * DURATION_S)
        points.append(point)
    traj.points = points
    return traj


def main():
    rclpy.init()
    node = Node('jtc_example')
    activated = False
    try:
        q = read_joint_positions(node)
        if q is None:
            node.get_logger().error('no driver status — is bringup running on real hardware?')
            return 1
        if not wait_controller(node, 'joint_trajectory_controller'):
            node.get_logger().error('joint_trajectory_controller is not loaded yet')
            return 1

        if not ensure_servo(node, set_speed=1.0):
            return 1

        if not activate_exclusive(node, 'joint_trajectory_controller'):
            node.get_logger().error('could not switch to joint_trajectory_controller')
            return 1
        activated = True

        hold_until = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < hold_until:
            rclpy.spin_once(node, timeout_sec=0.05)
        q_now = read_joint_positions(node, timeout=2.0)
        if q_now is None:
            node.get_logger().error('no joint_states after JTC activate')
            return 1
        q = q_now

        start = [q[j] for j in JOINTS]
        target = list(start)
        target[-1] += math.radians(OFFSET_DEG)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = build_trajectory(start, target)

        client = ActionClient(
            node, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error('follow_joint_trajectory action is not up')
            return 1

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
        rclpy.spin_until_future_complete(node, done, timeout_sec=DURATION_S + 8.0)
        action_ok = False
        if not done.done():
            node.get_logger().error(
                'trajectory timed out — JTC never returned (goal not reached)')
        else:
            wrapped = done.result()
            action_ok = wrapped is not None and wrapped.result.error_code == 0
            if not action_ok and wrapped is not None:
                node.get_logger().error(
                    f'trajectory failed: code={wrapped.result.error_code}')

        after = wait_msg(node, '/joint_states', JointState, timeout=1.0)
        if after is None:
            node.get_logger().error('no joint_states after trajectory')
            return 1
        q_after = {n: p for n, p in zip(after.name, after.position)}
        moved_deg = joint5_moved_deg(q, q_after)
        ok = report_joint5_motion(
            node, 'jtc', moved_deg, OFFSET_DEG, action_ok=action_ok)
        return 0 if ok else 1
    finally:
        if activated:
            cleanup_motion_controllers(node, ['joint_trajectory_controller'])
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
