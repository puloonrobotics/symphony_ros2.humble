#!/usr/bin/env python3
"""Absolute servoQ, then relative servoQ."""

from __future__ import annotations

import math
import os
import sys
import time

import rclpy
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
JOINT_TOPIC = '/forward_position_controller/commands'
OPTIONS_TOPIC = '/motion_options_controller/options'
JOINT5 = 5
STEP_DEG = 0.05
TRAVEL_DEG = 5.0
N_STEPS = max(1, int(round(TRAVEL_DEG / STEP_DEG)))
DT = 0.01
SETTLE_S = 2.0


def set_servo_relative(node, enabled):
    client = node.create_client(
        SetMotionOptions, '/motion_options_controller/set_motion_options')
    req = SetMotionOptions.Request()
    req.work = -1
    req.tool = -1
    req.rel = False
    req.fixedspeed = False
    req.cfg = -1
    req.coord = -1
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
        if msg is not None and len(msg.data) >= 11 and (msg.data[10] > 0.5) == bool(enabled):
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


def publish_steps(node, pub, cmd):
    msg = Float64MultiArray()
    msg.data = list(cmd)
    for _ in range(N_STEPS):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(DT)


def joint5_deg(q1, q0):
    return math.degrees(q1['joint5'] - q0['joint5'])


def main():
    rclpy.init()
    node = Node('servoq_abs_rel_example')
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
        if not wait_controller(node, SERVOQ):
            node.get_logger().error('servo controller is not loaded')
            return 1
        if not set_servo_relative(node, False):
            return 1
        pub = node.create_publisher(Float64MultiArray, JOINT_TOPIC, 10)
        if not activate_exclusive(node, SERVOQ):
            node.get_logger().error(f'could not activate {SERVOQ}')
            return 1
        if not wait_subscribers(node, pub):
            node.get_logger().error(f'{SERVOQ} did not subscribe')
            return 1
        time.sleep(3.0)  # >= rt_settle
        q0 = read_joint_positions(node)
        if q0 is None:
            node.get_logger().error('no /joint_states')
            return 1
        cmd = [q0[name] for name in JOINTS]
        step = math.radians(STEP_DEG)
        for _ in range(N_STEPS):
            cmd[JOINT5] += step
            msg = Float64MultiArray()
            msg.data = list(cmd)
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(DT)
        time.sleep(SETTLE_S)
        q1 = read_joint_positions(node)
        if q1 is None:
            node.get_logger().error('no /joint_states after absolute servoQ')
            return 1
        abs_moved = joint5_deg(q1, q0)
        if abs_moved < TRAVEL_DEG * 0.5:
            node.get_logger().error(
                f'absolute servoQ joint5 moved {abs_moved:.2f} deg '
                f'(want ~{TRAVEL_DEG:.1f})')
            return 1
        node.get_logger().info(f'absolute servoQ ok (joint5 {abs_moved:.2f} deg)')
        if not set_servo_relative(node, True):
            return 1
        time.sleep(3.0)  # >= rt_settle
        rel = [0.0] * len(JOINTS)
        rel[JOINT5] = step
        publish_steps(node, pub, rel)
        time.sleep(0.5)
        q2 = read_joint_positions(node)
        if q2 is None:
            node.get_logger().error('no /joint_states after relative servoQ')
            return 1
        rel_moved = joint5_deg(q2, q1)
        if rel_moved < TRAVEL_DEG * 0.5:
            node.get_logger().error(
                f'relative servoQ joint5 moved {rel_moved:.2f} deg '
                f'(want ~{TRAVEL_DEG:.1f})')
            return 1
        node.get_logger().info(
            f'relative servoQ ok (joint5 {rel_moved:.2f} deg, '
            f'total {joint5_deg(q2, q0):.2f} deg)')
        rc = 0
        return 0
    finally:
        try:
            deactivate_if_active(node, SERVOQ)
            cleanup_motion_controllers(node, [SERVOQ])
            set_servo_relative(node, False)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(rc)


if __name__ == '__main__':
    main()
