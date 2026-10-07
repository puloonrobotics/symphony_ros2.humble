#!/usr/bin/env python3
"""Relative servoQ then relative servoX."""

from __future__ import annotations

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt8
from symphony_msgs.srv import SetMotionOptions

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from example_common import (  # noqa: E402
    JOINTS,
    activate_exclusive,
    call,
    cleanup_motion_controllers,
    deactivate_if_active,
    ensure_servo,
    read_joint_positions,
    wait_controller,
    wait_msg,
)

OPMODE_EXT = 3
SERVOQ = 'forward_position_controller'
SERVOX = 'cartesian_pose_controller'
JOINT_TOPIC = '/forward_position_controller/commands'
POSE_TOPIC = '/cartesian_pose_controller/pose'
OPTIONS_TOPIC = '/motion_options_controller/options'
JOINT5 = 5
STEP_DEG = 0.05  # same per-tick step as absolute servoQ
TRAVEL_DEG = 10.0
N_JOINT_STEPS = max(1, int(round(TRAVEL_DEG / STEP_DEG)))
STEP_MM = 0.09
STEP_M = STEP_MM / 1000.0
TRAVEL_M = 0.050
N_Z_STEPS = max(1, int(round(TRAVEL_M / STEP_M)))
DT = 0.01


def set_servo_relative(node, enabled, coord):
    client = node.create_client(
        SetMotionOptions, '/motion_options_controller/set_motion_options')
    req = SetMotionOptions.Request()
    req.work = -1
    req.tool = -1
    req.rel = False
    req.fixedspeed = False
    req.cfg = -1
    req.coord = int(coord)
    req.weaving = False
    req.fixedorient = False
    req.ext = 0
    req.tol = -1
    req.servo_relative = bool(enabled)
    result = call(node, client, req, timeout=5.0)
    if not result or not result.success:
        node.get_logger().error(
            'set_motion_options failed: '
            + (result.message if result else 'timeout'))
        return False
    deadline = time.monotonic() + 2.0
    while rclpy.ok() and time.monotonic() < deadline:
        msg = wait_msg(node, OPTIONS_TOPIC, Float64MultiArray, timeout=0.5)
        if msg is not None and len(msg.data) >= 11:
            rel_on = msg.data[10] > 0.5
            coord_now = int(round(msg.data[5]))
            if rel_on == bool(enabled) and (not enabled or coord_now == int(coord)):
                return True
    node.get_logger().error('servo_relative was not applied')
    return False


def wait_subscribers(node, pub, timeout=2.0):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if pub.get_subscription_count() >= 1:
            return True
    return False


def stream_joint5(node, pub, sign):
    cmd = [0.0] * len(JOINTS)
    cmd[JOINT5] = sign * math.radians(STEP_DEG)
    for _ in range(N_JOINT_STEPS):
        msg = Float64MultiArray()
        msg.data = list(cmd)
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(DT)


def run_servoq(node):
    q0 = read_joint_positions(node)
    if q0 is None:
        node.get_logger().error('no /joint_states')
        return False
    pub = node.create_publisher(Float64MultiArray, JOINT_TOPIC, 10)
    if not activate_exclusive(node, SERVOQ):
        node.get_logger().error(f'could not activate {SERVOQ}')
        return False
    if not wait_subscribers(node, pub):
        node.get_logger().error(f'{SERVOQ} did not subscribe')
        return False
    time.sleep(3.0)  # >= rt_settle
    stream_joint5(node, pub, +1.0)
    time.sleep(0.5)
    q1 = read_joint_positions(node)
    if q1 is None:
        node.get_logger().error('no /joint_states after servoQ')
        return False
    moved = math.degrees(q1['joint5'] - q0['joint5'])
    if moved < TRAVEL_DEG * 0.5:
        node.get_logger().error(
            f'relative servoQ joint5 moved {moved:.2f} deg (want ~{TRAVEL_DEG:.1f})')
        return False
    stream_joint5(node, pub, -1.0)
    time.sleep(0.5)
    q2 = read_joint_positions(node)
    if q2 is None:
        node.get_logger().error('no /joint_states after servoQ return')
        return False
    back = math.degrees(q2['joint5'] - q0['joint5'])
    if abs(back) > TRAVEL_DEG * 0.5:
        node.get_logger().error(
            f'relative servoQ did not return (err {back:.2f} deg)')
        return False
    node.get_logger().info(
        f'relative servoQ ok (joint5 {moved:.2f} deg, return err {back:.2f} deg)')
    return True


def stream_z(node, pub, sign):
    for _ in range(N_Z_STEPS):
        msg = PoseStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.pose = Pose()
        msg.pose.position.z = sign * STEP_M
        msg.pose.orientation.w = 1.0
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(DT)


def run_servox(node):
    current = wait_msg(node, '/tcp_pose_broadcaster/pose', PoseStamped, timeout=5.0)
    if current is None:
        node.get_logger().error('no /tcp_pose_broadcaster/pose')
        return False
    z0 = current.pose.position.z
    pub = node.create_publisher(PoseStamped, POSE_TOPIC, 10)
    if not activate_exclusive(node, SERVOX):
        node.get_logger().error(f'could not activate {SERVOX}')
        return False
    if not wait_subscribers(node, pub):
        node.get_logger().error(f'{SERVOX} did not subscribe')
        return False
    time.sleep(3.0)  # >= rt_settle
    stream_z(node, pub, +1.0)
    time.sleep(0.5)
    mid = wait_msg(node, '/tcp_pose_broadcaster/pose', PoseStamped, timeout=2.0)
    if mid is None:
        node.get_logger().error('no TCP pose after servoX outbound')
        return False
    out_mm = (mid.pose.position.z - z0) * 1000.0
    if out_mm < TRAVEL_M * 1000.0 * 0.5:
        node.get_logger().error(
            f'relative servoX Z moved {out_mm:.1f} mm (want ~{TRAVEL_M * 1000.0:.0f})')
        return False
    stream_z(node, pub, -1.0)
    time.sleep(0.5)
    after = wait_msg(node, '/tcp_pose_broadcaster/pose', PoseStamped, timeout=2.0)
    if after is None:
        node.get_logger().error('no TCP pose after servoX return')
        return False
    back_mm = (after.pose.position.z - z0) * 1000.0
    if abs(back_mm) > TRAVEL_M * 1000.0 * 0.5:
        node.get_logger().error(
            f'relative servoX did not return (err {back_mm:.1f} mm)')
        return False
    node.get_logger().info(
        f'relative servoX ok (Z {out_mm:.1f} mm, return err {back_mm:.1f} mm)')
    return True


def main():
    rclpy.init()
    node = Node('servo_relative_example')
    rc = 1
    try:
        mode = wait_msg(node, '/status_controller/op_mode', UInt8, timeout=5.0)
        if mode is None or int(mode.data) != OPMODE_EXT:
            node.get_logger().error(
                'needs External mode (op_mode=3), got '
                + ('none' if mode is None else str(int(mode.data))))
            return 1
        if not ensure_servo(node, 1.0):
            return 1
        if not wait_controller(node, SERVOQ) or not wait_controller(node, SERVOX):
            node.get_logger().error('servo controllers are not loaded')
            return 1
        if not set_servo_relative(node, True, 2):
            return 1
        if not run_servoq(node):
            return 1
        if not deactivate_if_active(node, SERVOQ):
            node.get_logger().error(f'could not deactivate {SERVOQ}')
            return 1
        time.sleep(0.3)
        if not run_servox(node):
            return 1
        rc = 0
        return 0
    finally:
        try:
            deactivate_if_active(node, SERVOQ)
            deactivate_if_active(node, SERVOX)
            cleanup_motion_controllers(node, [SERVOQ, SERVOX])
            set_servo_relative(node, False, -1)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(rc)


if __name__ == '__main__':
    main()
