"""Shared helpers for symphony_driver Python examples."""

import math
import time

import rclpy

from controller_manager_msgs.srv import ListControllers, SwitchController
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from symphony_msgs.srv import SetRtStream, SetSpeed

JOINTS = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
OFFSET_DEG = 5.0
DURATION_S = 1.0  # 5 deg at 5 deg/s; waypoint step stays 0.05 deg
WAYPOINT_STEP_DEG = 0.05

EXCLUSIVE_MOTION_CONTROLLERS = {
    'joint_trajectory_controller',
    'motion_primitive_controller',
    'native_joint_trajectory_controller',
    'forward_position_controller',
    'forward_velocity_controller',
    'cartesian_pose_controller',
}


def fmt_deg(rad):
    return f'{math.degrees(rad):.2f}'


def joint5_moved_deg(q_before, q_after):
    """Absolute joint5 travel in degrees between two joint maps."""
    return abs(math.degrees(q_after['joint5'] - q_before['joint5']))


def report_joint5_motion(node, label, moved_deg, want_deg, *, action_ok=None):
    """Unified pass/fail log for joint5 travel. Returns True if motion ok.

    action_ok:
      True  — controller/action reported success (highlight fake success)
      False — controller/action already failed
      None  — stream-only examples (servoq/speedq)
    """
    need = abs(float(want_deg))
    moved_ok = moved_deg >= (need * 0.5)
    if action_ok is False:
        node.get_logger().error(
            f'{label}: controller failed; joint5 moved {moved_deg:.2f} deg '
            f'(want ~{need:.1f})')
        return False
    if not moved_ok:
        if action_ok is True:
            node.get_logger().error(
                f'{label}: controller success but joint5 moved {moved_deg:.2f} deg '
                f'(want ~{need:.1f})')
        else:
            node.get_logger().error(
                f'{label}: joint5 moved {moved_deg:.2f} deg (want ~{need:.1f})')
        return False
    node.get_logger().info(f'{label} ok (joint5 moved {moved_deg:.2f} deg)')
    return True


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
    node.get_logger().debug(f'waiting for {name} (inactive or active)')
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
            for c in result.controller:
                if _controller_ready(c, name):
                    node.get_logger().debug(f'{name} is {c.state}')
                    return True
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


def active_motion_controllers(node):
    names = []
    for c in list_controllers(node):
        if c.name in EXCLUSIVE_MOTION_CONTROLLERS and is_active_state(getattr(c, 'state', '')):
            names.append(c.name)
    return names


def read_joint_positions(node, timeout=15.0):
    joints = wait_msg(node, '/joint_states', JointState, timeout=timeout)
    if joints is None:
        return None
    q = {n: p for n, p in zip(joints.name, joints.position)}
    if any(j not in q for j in JOINTS):
        return None
    return q


def ensure_rt_stream(node, period, filter_gain):
    client = node.create_client(SetRtStream, '/status_controller/set_rt_stream')
    req = SetRtStream.Request()
    req.period = float(period)
    req.filter = int(filter_gain)
    result = call(node, client, req)
    if not result or not result.success:
        node.get_logger().error(
            f'set_rt_stream(ts={period}, filter={filter_gain}) failed: '
            + (result.message if result else 'service timeout'))
        return False
    node.get_logger().info(f'set_rt_stream ts={period:.3f} filter={int(filter_gain)}')
    time.sleep(0.2)
    return True


def ensure_set_speed(node, speed=1.0):
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


def ensure_servo(node, set_speed=1.0):
    servo = wait_msg(node, '/status_controller/servo_enable', Bool)
    if servo is None:
        node.get_logger().error('no /status_controller/servo_enable')
        return False
    if not servo.data:
        req = SetBool.Request()
        req.data = True
        result = call(
            node, node.create_client(SetBool, '/status_controller/set_servo_enable'), req)
        if not result or not result.success:
            node.get_logger().error('set_servo_enable failed')
            return False
    if set_speed is not None and not ensure_set_speed(node, set_speed):
        return False
    return True


def activate_exclusive(node, controller_name):
    deactivate = [c for c in active_motion_controllers(node) if c != controller_name]
    if is_active_state(getattr(
            next((c for c in list_controllers(node) if c.name == controller_name), None),
            'state', '')):
        return True
    return switch(node, [controller_name], deactivate)


def deactivate_if_active(node, controller_name):
    for c in list_controllers(node):
        if c.name == controller_name and is_active_state(getattr(c, 'state', '')):
            return switch(node, [], [controller_name])
    return True


def cleanup_motion_controllers(node, names=None):
    """Deactivate exclusive motion controllers (Ctrl+C / early exit)."""
    try:
        targets = list(names) if names is not None else active_motion_controllers(node)
        if not targets:
            return True
        node.get_logger().debug('cleanup: deactivate ' + ', '.join(targets))
        return switch(node, [], targets)
    except Exception as exc:  # noqa: BLE001 — example cleanup must not raise
        try:
            node.get_logger().warn(f'cleanup_motion_controllers failed: {exc}')
        except Exception:  # noqa: BLE001
            pass
        return False
