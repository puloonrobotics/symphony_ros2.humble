#!/usr/bin/env python3
"""motion_primitive_controller : movePTP."""

import math
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from symphony_msgs.action import ExecuteMotionPrimitiveSequence
from symphony_msgs.msg import MotionArgument, MotionPrimitive

JOINTS = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
OFFSET_DEG = 5.0
SPEED_PERCENT = 25.0


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
    node.get_logger().debug(f'waiting for {name}')
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
                    node.get_logger().debug(f'{name} is {c.state}')
                    return True
            if name not in names_only and 'native_joint_trajectory_controller' in names_only:
                node.get_logger().error(
                    f'{name} never loaded. It is the in-tree '
                    'motion_primitive_controller, not an apt package.')
                node.get_logger().error(
                    'controllers currently loaded: ' + (', '.join(last_names) or '(none)'))
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
        node.get_logger().warn('list_controllers service not ready')
        return []
    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if not future.done():
        node.get_logger().warn('list_controllers timed out')
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


def main():
    rclpy.init()
    node = Node('movep_example')
    activated = False
    try:
        joints = wait_msg(node, '/joint_states', JointState)
        servo = wait_msg(node, '/status_controller/servo_enable', Bool)
        if joints is None or servo is None:
            node.get_logger().error('no driver status — is bringup running on real hardware?')
            return 1

        q = {n: p for n, p in zip(joints.name, joints.position)}
        if not wait_controller(node, 'motion_primitive_controller'):
            node.get_logger().error(
                'motion_primitive_controller is not loaded yet')
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
                'cartesian_pose_controller',
            ) and is_active_state(getattr(c, 'state', '')):
                deactivate.append(c.name)
        if not switch(node, ['motion_primitive_controller'], deactivate):
            node.get_logger().error('could not switch to motion_primitive_controller')
            return 1
        activated = True

        target = [q[j] for j in JOINTS]
        target[-1] += math.radians(OFFSET_DEG)

        primitive = MotionPrimitive()
        primitive.type = MotionPrimitive.LINEAR_JOINT
        primitive.joint_positions = target
        primitive.blend_radius = 0.0
        speed = MotionArgument()
        speed.name = 'velocity'
        speed.value = float(SPEED_PERCENT)
        primitive.additional_arguments = [speed]
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
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=60.0)
        if not result_future.done():
            node.get_logger().error('movePTP timed out')
            return 1
        wrapped = result_future.result()
        result = wrapped.result if wrapped is not None else None
        action_ok = (
            wrapped is not None and wrapped.status == GoalStatus.STATUS_SUCCEEDED and
            result is not None and result.error_code == 0)
        if result is not None and not action_ok:
            node.get_logger().error(f'motion_primitive: {result.error_string}')

        after = wait_msg(node, '/joint_states', JointState, timeout=1.0)
        if after is None:
            node.get_logger().error('no joint_states after move')
            return 1
        q_after = {n: p for n, p in zip(after.name, after.position)}
        moved_deg = abs(math.degrees(q_after['joint5'] - q['joint5']))
        need = abs(OFFSET_DEG)
        moved_ok = moved_deg >= (need * 0.5)
        if not action_ok:
            node.get_logger().error(
                f'movep: controller failed; joint5 moved {moved_deg:.2f} deg '
                f'(want ~{need:.1f})')
            return 1
        if not moved_ok:
            node.get_logger().error(
                f'movep: controller success but joint5 moved {moved_deg:.2f} deg '
                f'(want ~{need:.1f})')
            return 1
        node.get_logger().info(f'movep ok (joint5 moved {moved_deg:.2f} deg)')
        return 0
    finally:
        if activated:
            switch(node, [], ['motion_primitive_controller'])
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
