#!/usr/bin/env python3
"""motion_primitive_controller : moveLIN (Cartesian) - Relative."""

from __future__ import annotations

import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, UInt8
from std_srvs.srv import SetBool, Trigger
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from symphony_msgs.action import ExecuteMotionPrimitiveSequence
from symphony_msgs.msg import MotionArgument, MotionPrimitive
from symphony_msgs.srv import SetMotionOptions

SPEED_PERCENT = 15.0
DEFAULT_DX_MM = -30.0
DEFAULT_DY_MM = -30.0
DEFAULT_DZ_MM = 30.0


def wait_msg(node, topic, msg_type, timeout=15.0):
    box = []
    sub = node.create_subscription(msg_type, topic, box.append, 10)
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if box:
            node.destroy_subscription(sub)
            return box[-1]
    node.destroy_subscription(sub)
    return None


def call(node, client, request, timeout=5.0):
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        return None
    return future.result()


def _controller_ready(controller, name):
    if controller.name != name:
        return False
    state = getattr(controller, 'state', '')
    if state in ('inactive', 'active'):
        return True
    try:
        return int(state) in (2, 3)
    except (TypeError, ValueError):
        return False


def wait_controller(node, name, timeout=60.0):
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    deadline = time.monotonic() + timeout
    last_names = []
    last_log = 0.0
    while rclpy.ok() and time.monotonic() < deadline:
        if not client.wait_for_service(timeout_sec=0.5):
            continue
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        if not future.done():
            rclpy.spin_once(node, timeout_sec=0.3)
            continue
        result = future.result()
        if result is not None:
            last_names = [f'{c.name}:{getattr(c, "state", "")}' for c in result.controller]
            names_only = [c.name for c in result.controller]
            for c in result.controller:
                if _controller_ready(c, name):
                    return True
            if name not in names_only and 'native_joint_trajectory_controller' in names_only:
                node.get_logger().error(
                    f'{name} never loaded. It is the in-tree '
                    'motion_primitive_controller, not an apt package.')
                return False
        now = time.monotonic()
        if now - last_log >= 5.0:
            last_log = now
            node.get_logger().debug(
                'still waiting; loaded: ' + (', '.join(last_names) or '(none)'))
        rclpy.spin_once(node, timeout_sec=0.3)
    node.get_logger().error('controllers currently loaded: ' + (', '.join(last_names) or '(none)'))
    return False


def list_controllers(node):
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    if not client.wait_for_service(timeout_sec=2.0):
        return []
    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if not future.done():
        return []
    result = future.result()
    return list(result.controller) if result is not None else []


def is_active_state(state):
    if state == 'active':
        return True
    try:
        return int(state) == 3
    except (TypeError, ValueError):
        return False


def switch(node, activate, deactivate):
    client = node.create_client(SwitchController, '/controller_manager/switch_controller')
    req = SwitchController.Request()
    if hasattr(req, 'activate_controllers'):
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
    else:
        req.start_controllers = activate
        req.stop_controllers = deactivate
    req.strictness = getattr(SwitchController.Request, 'BEST_EFFORT', 1)
    timeout = getattr(req, 'timeout', None)
    if timeout is not None and hasattr(timeout, 'sec'):
        from builtin_interfaces.msg import Duration
        req.timeout = Duration(sec=5, nanosec=0)
    result = call(node, client, req, timeout=10.0)
    return result is not None and getattr(result, 'ok', False)


def set_motion_options(node, *, rel: bool):
    client = node.create_client(
        SetMotionOptions, '/motion_options_controller/set_motion_options')
    req = SetMotionOptions.Request()
    req.work = -1
    req.tool = -1
    req.rel = bool(rel)
    req.fixedspeed = False
    req.cfg = -1
    req.coord = -1  # auto → XYZ for LIN
    req.weaving = False
    req.fixedorient = False
    req.ext = 0
    req.tol = -1
    req.servo_relative = False
    result = call(node, client, req, timeout=5.0)
    if not result or not result.success:
        node.get_logger().error(
            'set_motion_options failed: '
            + (result.message if result else 'timeout'))
        return False
    node.get_logger().info(f'set_motion_options(rel={rel}) accepted')
    if not wait_rel_applied(node, rel):
        node.get_logger().error(
            f'rel={rel} was not applied on motion_options before moveLIN '
            '(refusing to send — a relative delta as absolute would jump)')
        return False
    return True


def wait_rel_applied(node, want_rel, timeout=2.0):
    """Index 2 of ~/options is the sticky rel flag (see motion_options_controller)."""
    box = []
    sub = node.create_subscription(
        Float64MultiArray, '/motion_options_controller/options', box.append, 10)
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not box:
                continue
            data = box[-1].data
            if len(data) > 2 and ((data[2] > 0.5) == bool(want_rel)):
                return True
    finally:
        node.destroy_subscription(sub)
    return False


def reset_motion_options(node):
    client = node.create_client(
        Trigger, '/motion_options_controller/reset_motion_options')
    result = call(node, client, Trigger.Request(), timeout=5.0)
    if result and result.success:
        node.get_logger().info('reset_motion_options ok')
    return result is not None and getattr(result, 'success', False)


def as_stamped(pose: Pose) -> PoseStamped:
    stamped = PoseStamped()
    stamped.header.frame_id = 'base'
    stamped.pose = pose
    return stamped


def identity_pose(dx_m=0.0, dy_m=0.0, dz_m=0.0) -> Pose:
    pose = Pose()
    pose.position.x = float(dx_m)
    pose.position.y = float(dy_m)
    pose.position.z = float(dz_m)
    pose.orientation.w = 1.0
    return pose


def add_delta(base: Pose, dx_m: float, dy_m: float, dz_m: float) -> Pose:
    out = Pose()
    out.position.x = float(base.position.x + dx_m)
    out.position.y = float(base.position.y + dy_m)
    out.position.z = float(base.position.z + dz_m)
    out.orientation = base.orientation
    return out


def main():
    rclpy.init()
    node = Node('movel_example')
    activated = False
    options_set = False

    node.declare_parameter('mode', 'relative')  # relative | absolute
    node.declare_parameter('dx_mm', DEFAULT_DX_MM)
    node.declare_parameter('dy_mm', DEFAULT_DY_MM)
    node.declare_parameter('dz_mm', DEFAULT_DZ_MM)
    node.declare_parameter('speed_percent', SPEED_PERCENT)

    motion_mode = str(node.get_parameter('mode').value).strip().lower()
    dx_mm = float(node.get_parameter('dx_mm').value)
    dy_mm = float(node.get_parameter('dy_mm').value)
    dz_mm = float(node.get_parameter('dz_mm').value)
    speed = float(node.get_parameter('speed_percent').value)
    dx_m, dy_m, dz_m = dx_mm / 1000.0, dy_mm / 1000.0, dz_mm / 1000.0

    try:
        joints = wait_msg(node, '/joint_states', JointState)
        servo = wait_msg(node, '/status_controller/servo_enable', Bool)
        op_mode = wait_msg(node, '/status_controller/op_mode', UInt8, timeout=5.0)
        if joints is None or servo is None:
            node.get_logger().error('no driver status — is bringup running?')
            return 1
        if op_mode is None or int(op_mode.data) != 3:
            node.get_logger().error(
                'pendant must be External (op_mode=3) before moveLIN')
            return 1

        if not wait_controller(node, 'motion_primitive_controller'):
            node.get_logger().error('motion_primitive_controller not loaded')
            return 1
        if not wait_controller(node, 'motion_options_controller', timeout=30.0):
            node.get_logger().error('motion_options_controller not loaded')
            return 1

        if not servo.data:
            req = SetBool.Request()
            req.data = True
            result = call(
                node, node.create_client(SetBool, '/status_controller/set_servo_enable'), req)
            if not result or not result.success:
                node.get_logger().error('set_servo_enable failed')
                return 1

        deactivate = []
        for c in list_controllers(node):
            if c.name in (
                'joint_trajectory_controller',
                'forward_position_controller',
                'forward_velocity_controller',
                'native_joint_trajectory_controller',
                'cartesian_pose_controller',
            ) and is_active_state(getattr(c, 'state', '')):
                deactivate.append(c.name)
        if not switch(node, ['motion_primitive_controller'], deactivate):
            node.get_logger().error('could not switch to motion_primitive_controller')
            return 1
        activated = True

        use_rel = motion_mode in ('relative', 'rel', 'r')
        if motion_mode not in ('relative', 'rel', 'r', 'absolute', 'abs', 'a'):
            node.get_logger().error(f'unknown mode={motion_mode!r} (use relative|absolute)')
            return 1

        if use_rel:
            if not set_motion_options(node, rel=True):
                return 1
            options_set = True
            goal_pose = identity_pose(dx_m, dy_m, dz_m)
            node.get_logger().info(
                f'moveLIN relative Δxyz=({dx_mm:.1f}, {dy_mm:.1f}, {dz_mm:.1f}) mm')
        else:
            tcp = wait_msg(node, '/tcp_pose_broadcaster/pose', PoseStamped, timeout=5.0)
            if tcp is None:
                node.get_logger().error(
                    'no /tcp_pose_broadcaster/pose — cannot build absolute target')
                return 1
            if not set_motion_options(node, rel=False):
                return 1
            options_set = True
            before = tcp.pose
            goal_pose = add_delta(before, dx_m, dy_m, dz_m)
            node.get_logger().info(
                'moveLIN absolute: '
                f'from ({before.position.x:.4f}, {before.position.y:.4f}, '
                f'{before.position.z:.4f}) + Δmm=({dx_mm:.1f}, {dy_mm:.1f}, {dz_mm:.1f})')

        stamped = PoseStamped()
        stamped.pose = goal_pose
        primitive = MotionPrimitive()
        primitive.type = MotionPrimitive.LINEAR_CARTESIAN
        primitive.poses = [stamped]
        primitive.blend_radius = 0.0
        speed_arg = MotionArgument()
        speed_arg.name = 'velocity'
        speed_arg.value = float(speed)
        primitive.additional_arguments = [speed_arg]
        goal = ExecuteMotionPrimitiveSequence.Goal()
        goal.trajectory.motions = [primitive]
        client = ActionClient(
            node,
            ExecuteMotionPrimitiveSequence,
            '/motion_primitive_controller/motion_primitive')
        if not client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error('motion_primitive action is not up')
            return 1
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send, timeout_sec=5.0)
        goal_handle = send.result() if send.done() else None
        if goal_handle is None or not goal_handle.accepted:
            node.get_logger().error('motion_primitive goal was rejected')
            return 1
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=90.0)
        wrapped = result_future.result() if result_future.done() else None
        result = wrapped.result if wrapped is not None else None
        if wrapped is None:
            node.get_logger().error('moveLIN timed out')
            return 1
        action_ok = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED and
            result is not None and result.error_code == 0)
        if not action_ok:
            node.get_logger().error(
                f'motion_primitive: {result.error_string if result is not None else "no result"}')
            return 1

        settle = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < settle:
            rclpy.spin_once(node, timeout_sec=0.1)
        after = wait_msg(node, '/tcp_pose_broadcaster/pose', PoseStamped, timeout=2.0)
        if after is not None:
            node.get_logger().info(
                f'movel ok — TCP now '
                f'({after.pose.position.x * 1000.0:.2f}, {after.pose.position.y * 1000.0:.2f}, '
                f'{after.pose.position.z * 1000.0:.2f}) mm')
        else:
            node.get_logger().info('movel ok (no TCP pose to print)')
        return 0
    finally:
        if options_set:
            reset_motion_options(node)
        if activated:
            switch(node, [], ['motion_primitive_controller'])
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
