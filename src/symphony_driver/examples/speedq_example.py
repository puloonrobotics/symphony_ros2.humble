#!/usr/bin/env python3
"""forward_velocity_controller (RT speedQ)."""

import math
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, UInt8
from std_srvs.srv import SetBool
from symphony_msgs.srv import SetSpeed

OPMODE_EXT = 3

JOINTS = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
VEL_DEG_S = 0.25  # [deg]/s -> rad/s for ROS
VEL_RAD_S = math.radians(VEL_DEG_S)
DT = 0.01
TRAVEL_DEG = 5.0
N_STEPS = max(1, int(round(TRAVEL_DEG / (VEL_DEG_S * DT))))
AFTER_START_S = 3.0  # >= rt_settle
AFTER_LEG_S = 1.0
BEFORE_STOP_S = 2.0
RAMP_ZERO_STEPS = 50
AXIS = 5  # joint5 (J6)
SET_SPEED = 1.0


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


def ensure_set_speed(node, speed):
    client = node.create_client(SetSpeed, '/status_controller/set_speed')
    req = SetSpeed.Request()
    req.speed = float(speed)
    result = call(node, client, req)
    if not result or not result.success:
        node.get_logger().error(
            f'set_speed({speed}) failed: '
            + (result.message if result else 'service timeout'))
        return False
    node.get_logger().info(f'setSpeed={speed:.2f}')
    return True


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
            if name not in names_only:
                node.get_logger().error(
                    f'{name} never loaded. Install: sudo apt install '
                    'ros-humble-forward-command-controller')
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
        req.timeout = Duration(sec=15, nanosec=0)
    result = call(node, client, req, timeout=20.0)
    return result is not None and getattr(result, 'ok', False)


def _active_motion_controllers(node):
    exclusive = {
        'joint_trajectory_controller',
        'motion_primitive_controller',
        'native_joint_trajectory_controller',
        'forward_position_controller',
        'forward_velocity_controller',
        'cartesian_pose_controller',
    }
    names = []
    for c in list_controllers(node):
        if c.name in exclusive and is_active_state(getattr(c, 'state', '')):
            names.append(c.name)
    return names


def stop_rt(node, pub):
    zero = [0.0] * len(JOINTS)
    msg = Float64MultiArray()
    msg.data = zero
    for _ in range(RAMP_ZERO_STEPS):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(DT)
    if not switch(node, [], ['forward_velocity_controller']):
        node.get_logger().error('switch failed while stopping RT speedQ')
        return False
    for c in list_controllers(node):
        if c.name == 'forward_velocity_controller' and is_active_state(getattr(c, 'state', '')):
            node.get_logger().error(
                'forward_velocity_controller still active — RT speedQ may still be on')
            return False
    return True


def wait_subscribers(node, pub, timeout=5.0):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if pub.get_subscription_count() > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return False


def hold_zero(node, pub, seconds):
    msg = Float64MultiArray()
    msg.data = [0.0] * len(JOINTS)
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=DT)


def stream_axis_vel(node, pub, sign):
    cmd = [0.0] * len(JOINTS)
    cmd[AXIS] = sign * VEL_RAD_S
    msg = Float64MultiArray()
    msg.data = cmd
    for _ in range(N_STEPS):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(DT)


def main():
    rclpy.init()
    node = Node('speedq_example')

    joints = wait_msg(node, '/joint_states', JointState)
    servo = wait_msg(node, '/status_controller/servo_enable', Bool)
    mode = wait_msg(node, '/status_controller/op_mode', UInt8)
    if joints is None or servo is None:
        node.get_logger().error('no driver status — is bringup running on real hardware?')
        rclpy.shutdown()
        return 1
    opmode = int(mode.data) if mode is not None else -1
    if opmode != OPMODE_EXT:
        node.get_logger().error(
            'pendant is not External (op_mode must be 3). '
            'RT speedQ may be ignored outside External mode')
        rclpy.shutdown()
        return 1

    q = {n: p for n, p in zip(joints.name, joints.position)}

    if not wait_controller(node, 'forward_velocity_controller'):
        node.get_logger().error('forward_velocity_controller is not loaded yet')
        rclpy.shutdown()
        return 1

    if not servo.data:
        req = SetBool.Request()
        req.data = True
        result = call(
            node, node.create_client(SetBool, '/status_controller/set_servo_enable'), req)
        if not result or not result.success:
            node.get_logger().error('set_servo_enable failed')
            rclpy.shutdown()
            return 1

    if not ensure_set_speed(node, SET_SPEED):
        rclpy.shutdown()
        return 1

    joints = wait_msg(node, '/joint_states', JointState, timeout=2.0)
    if joints is None:
        node.get_logger().error('no joint_states before startRtSpeedQControl')
        rclpy.shutdown()
        return 1
    q = {n: p for n, p in zip(joints.name, joints.position)}
    if any(j not in q for j in JOINTS):
        node.get_logger().error('joint_states missing a commanded joint')
        rclpy.shutdown()
        return 1

    pub = node.create_publisher(
        Float64MultiArray, '/forward_velocity_controller/commands', 10)
    started = False
    streamed = False
    try:
        if not switch(node, ['forward_velocity_controller'], _active_motion_controllers(node)):
            node.get_logger().error('could not start RT speedQ')
            return 1
        started = True
        if not wait_subscribers(node, pub):
            node.get_logger().error(
                'forward_velocity_controller did not subscribe to commands')
            return 1

        hold_zero(node, pub, AFTER_START_S)
        stream_axis_vel(node, pub, +1.0)
        streamed = True
        hold_zero(node, pub, AFTER_LEG_S)

        after = wait_msg(node, '/joint_states', JointState, timeout=1.0)
        if after is None:
            node.get_logger().error('no joint_states after outbound speedQ')
            return 1
        q_after = {n: p for n, p in zip(after.name, after.position)}
        moved_deg = abs(math.degrees(q_after['joint5'] - q['joint5']))
        outbound_ok = moved_deg >= (TRAVEL_DEG * 0.5)
        if not outbound_ok:
            node.get_logger().error(
                f'speedq: joint5 moved {moved_deg:.2f} deg (want ~{TRAVEL_DEG:.1f})')
            return 1
        node.get_logger().info(f'speedq ok (joint5 moved {moved_deg:.2f} deg)')
        return 0
    finally:
        if started:
            if streamed:
                hold_zero(node, pub, BEFORE_STOP_S)
            stop_rt(node, pub)
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
