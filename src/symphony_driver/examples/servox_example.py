#!/usr/bin/env python3
"""cartesian_pose_controller (RT servoX)."""

from __future__ import annotations

import copy
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import UInt8

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from example_common import (  # noqa: E402
    activate_exclusive,
    cleanup_motion_controllers,
    deactivate_if_active,
    ensure_servo,
    wait_controller,
    wait_msg,
)

OPMODE_EXT = 3
CTRL = 'cartesian_pose_controller'
POSE_TOPIC = '/cartesian_pose_controller/pose'
STEP_MM = 0.09
STEP_M = STEP_MM / 1000.0
TRAVEL_M = 0.050
N_STEPS = max(1, int(round(TRAVEL_M / STEP_M)))
DT = 0.01


def _publish(node, pub, pose, frame_id):
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.pose = pose
    pub.publish(msg)


def _stream_x(node, pub, pose, frame_id, sign):
    for _ in range(N_STEPS):
        pose.position.x += sign * STEP_M
        _publish(node, pub, pose, frame_id)
        time.sleep(DT)
        rclpy.spin_once(node, timeout_sec=0.0)


def main():
    rclpy.init()
    node = Node('servox_example')
    pub = None
    try:
        mode = wait_msg(node, '/status_controller/op_mode', UInt8, timeout=5.0)
        if mode is None or int(mode.data) != OPMODE_EXT:
            node.get_logger().error(
                'servox needs External mode (op_mode=3), got '
                + ('none' if mode is None else str(int(mode.data))))
            return 1
        if not ensure_servo(node, 1.0):
            return 1
        if not wait_controller(node, CTRL):
            node.get_logger().error(f'{CTRL} is not loaded')
            return 1

        current = wait_msg(node, '/tcp_pose_broadcaster/pose', PoseStamped, timeout=5.0)
        if current is None:
            node.get_logger().error('no /tcp_pose_broadcaster/pose')
            return 1
        frame_id = current.header.frame_id or 'base'
        start = current.pose
        origin_x = start.position.x
        node.get_logger().info(
            f'start TCP x={start.position.x:.4f} y={start.position.y:.4f} '
            f'z={start.position.z:.4f} frame={frame_id}')

        pub = node.create_publisher(PoseStamped, POSE_TOPIC, 10)
        if not activate_exclusive(node, CTRL):
            node.get_logger().error(f'could not activate {CTRL}')
            return 1

        deadline = time.monotonic() + 2.0
        while rclpy.ok() and pub.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if pub.get_subscription_count() < 1:
            node.get_logger().error(f'{CTRL} did not subscribe to {POSE_TOPIC}')
            return 1

        time.sleep(3.0)  # >= rt_settle
        pose = copy.deepcopy(start)
        _publish(node, pub, pose, frame_id)
        time.sleep(0.2)
        _stream_x(node, pub, pose, frame_id, +1.0)
        time.sleep(0.5)
        mid = wait_msg(node, '/tcp_pose_broadcaster/pose', PoseStamped, timeout=2.0)
        if mid is None:
            node.get_logger().error('no TCP pose during servoX')
            return 1
        out_mm = (mid.pose.position.x - origin_x) * 1000.0
        if out_mm < 25.0:
            node.get_logger().error(
                f'servoX outbound too small (dx={out_mm:.1f} mm, want ~50)')
            return 1
        _stream_x(node, pub, pose, frame_id, -1.0)
        time.sleep(0.5)

        after = wait_msg(node, '/tcp_pose_broadcaster/pose', PoseStamped, timeout=2.0)
        if after is None:
            node.get_logger().error('no TCP pose after servoX')
            return 1
        dx_mm = (after.pose.position.x - origin_x) * 1000.0
        if abs(dx_mm) > 25.0:
            node.get_logger().error(
                f'servoX did not return near the start (dx={dx_mm:.1f} mm)')
            return 1
        node.get_logger().info(f'servoX ok (returned, dx={dx_mm:.1f} mm)')
        return 0
    finally:
        try:
            deactivate_if_active(node, CTRL)
            cleanup_motion_controllers(node, [CTRL])
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
