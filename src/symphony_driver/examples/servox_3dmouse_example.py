#!/usr/bin/env python3
"""3D mouse (SpaceMouse) → absolute RT servoX in the base frame."""

from __future__ import annotations

import math
import os
import select
import struct
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8, UInt32

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from example_common import (  # noqa: E402
    activate_exclusive,
    call,
    deactivate_if_active,
    ensure_servo,
    wait_controller,
    wait_msg,
)

try:
    from symphony_msgs.srv import SetSpeed
except ImportError:  # pragma: no cover
    SetSpeed = None

try:
    import hid
except ImportError:  # pragma: no cover
    hid = None

OPMODE_EXT = 3
CTRL = 'cartesian_pose_controller'
POSE_TOPIC = '/cartesian_pose_controller/pose'
TCP_TOPIC = '/tcp_pose_broadcaster/pose'
SAMPLE_DT = 0.002
AFTER_START_S = 3.2
BEFORE_STOP_S = 2.0
SET_SPEED = 1.0
MOUSE_FULL_SCALE = 350.0
MAX_LINEAR_SPEED = 0.110
MAX_ANGULAR_SPEED = math.radians(30.0)
MAX_LINEAR_ACCEL = 0.60
MAX_ANGULAR_ACCEL = math.radians(120.0)
RELEASE_S = 0.08

DEFAULT_VID = 0x046D
DEFAULT_PID = 0xC628

EV_KEY, EV_REL, EV_ABS = 1, 2, 3
DEFAULT_EVDEV = '/dev/input/by-id/usb-3Dconnexion_SpaceNavigator_for_Notebooks-event-if00'
# FAULT, STO, SS1, SS2, COLLISION, EMERGENCY. Same mask as the driver.
UNSAFE_ARM = (1 << 0) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14) | (1 << 18)


def ensure_set_speed(node, speed):
    if SetSpeed is None:
        node.get_logger().warn('symphony_msgs/SetSpeed missing — skip set_speed')
        return True
    client = node.create_client(SetSpeed, '/status_controller/set_speed')
    req = SetSpeed.Request()
    req.speed = float(speed)
    result = call(node, client, req, timeout=5.0)
    if not result or not getattr(result, 'success', False):
        node.get_logger().error(f'set_speed({speed}) failed')
        return False
    node.get_logger().info(f'setSpeed={speed:.2f}')
    return True


def wait_subscribers(node, pub, timeout=5.0):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if pub.get_subscription_count() > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return False


def quat_to_rpy(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def clone_pose(src):
    dst = Pose()
    dst.position.x = src.position.x
    dst.position.y = src.position.y
    dst.position.z = src.position.z
    dst.orientation.x = src.orientation.x
    dst.orientation.y = src.orientation.y
    dst.orientation.z = src.orientation.z
    dst.orientation.w = src.orientation.w
    return dst


def apply_step(pose, dx, dy, dz, droll, dpitch, dyaw):
    pose.position.x += dx
    pose.position.y += dy
    pose.position.z += dz
    q = pose.orientation
    dq = rpy_to_quat(droll, dpitch, dyaw)
    qx, qy, qz, qw = quat_multiply(dq, (q.x, q.y, q.z, q.w))
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    q.x, q.y, q.z, q.w = qx, qy, qz, qw
    return pose


def publish_pose(node, pub, pose):
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'base'
    msg.pose = pose
    pub.publish(msg)


def hold(node, pub, latest, seconds):
    n = max(1, int(round(seconds / SAMPLE_DT)))
    for _ in range(n):
        if latest[0] is not None:
            publish_pose(node, pub, clone_pose(latest[0].pose))
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(SAMPLE_DT)


def normalize_axes(raw, deadzone):
    span = max(1.0, MOUSE_FULL_SCALE - float(deadzone))
    out = []
    for value in raw:
        magnitude = abs(float(value))
        if magnitude <= deadzone:
            out.append(0.0)
        else:
            out.append(math.copysign(min(1.0, (magnitude - deadzone) / span), value))
    return out


def mouse_to_base_velocity(axes, linear_speed, angular_speed):
    # raw index: 0 tx, 1 ty, 2 tz, 3 rx, 4 ry, 5 rz
    return (
        -axes[1] * linear_speed,
        -axes[0] * linear_speed,
        -axes[2] * linear_speed,
        -axes[4] * angular_speed,
        -axes[3] * angular_speed,
        -axes[5] * angular_speed,
    )


def slew(current, target, limit):
    if target > current + limit:
        return current + limit
    if target < current - limit:
        return current - limit
    return target


class HidBackend:
    def __init__(self, device):
        self.device = device
        self.raw = [0, 0, 0, 0, 0, 0]
        self.seen = [0.0] * 6
        self.freeze = False

    def poll(self):
        now = time.monotonic()
        for _ in range(8):
            data = self.device.read(7)
            if not data:
                break
            rid = data[0]
            if rid == 1 and len(data) >= 7:
                self.raw[0] = int.from_bytes(bytes(data[1:3]), 'little', signed=True)
                self.raw[1] = int.from_bytes(bytes(data[3:5]), 'little', signed=True)
                self.raw[2] = int.from_bytes(bytes(data[5:7]), 'little', signed=True)
                for i in range(3):
                    self.seen[i] = now
            elif rid == 2 and len(data) >= 7:
                self.raw[3] = int.from_bytes(bytes(data[1:3]), 'little', signed=True)
                self.raw[4] = int.from_bytes(bytes(data[3:5]), 'little', signed=True)
                self.raw[5] = int.from_bytes(bytes(data[5:7]), 'little', signed=True)
                for i in range(3, 6):
                    self.seen[i] = now
            elif rid == 3 and len(data) >= 2:
                self.freeze = (data[1] & 1) != 0
        for i in range(6):
            if now - self.seen[i] > RELEASE_S:
                self.raw[i] = 0
        return list(self.raw), self.freeze

    def close(self):
        try:
            self.device.close()
        except Exception:
            pass


class EvdevBackend:
    """Read SpaceNavigator via /dev/input/event* (no hid module)."""

    def __init__(self, path):
        self.path = path
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.fmt = 'llHHi'
        self.size = struct.calcsize(self.fmt)
        self.raw = [0, 0, 0, 0, 0, 0]
        self.seen = [0.0] * 6
        self.freeze = False

    def poll(self):
        now = time.monotonic()
        deadline = now + 0.002
        while time.monotonic() < deadline:
            r, _, _ = select.select([self.fd], [], [], 0.0)
            if not r:
                break
            data = os.read(self.fd, self.size * 64)
            for off in range(0, len(data), self.size):
                if len(data) - off < self.size:
                    break
                _a, _b, typ, code, val = struct.unpack_from(self.fmt, data, off)
                if typ == EV_KEY and code in (0, 1, 256, 257, 272, 273):
                    self.freeze = val != 0
                elif typ == EV_REL and 0 <= code <= 5:
                    self.raw[code] = int(val)
                    self.seen[code] = now
        for i in range(6):
            if now - self.seen[i] > RELEASE_S:
                self.raw[i] = 0
        return list(self.raw), self.freeze

    def close(self):
        try:
            os.close(self.fd)
        except Exception:
            pass


def open_backend(node, vid, pid, evdev_path):
    if hid is not None:
        device = hid.device()
        try:
            device.open(vid, pid)
            device.set_nonblocking(True)
            try:
                product = device.get_product_string()
            except Exception:
                product = '?'
            node.get_logger().info(
                f'HID OK ({product}) VID=0x{vid:04x} PID=0x{pid:04x}')
            return HidBackend(device)
        except OSError as exc:
            node.get_logger().warn(f'HID open failed: {exc} — try evdev')

    path = evdev_path
    if not os.path.exists(path):
        node.get_logger().error(
            f'No HID and no evdev at {path}. '
            'Check lsusb; add user to `input` group; install python3-hidapi.')
        return None
    try:
        be = EvdevBackend(path)
    except PermissionError as exc:
        node.get_logger().error(
            f'evdev Permission denied ({exc}). Run: sudo usermod -aG input $USER '
            'then log out/in (device is in group `input`).')
        return None
    node.get_logger().info(f'evdev OK ({path})')
    return be


def arm_is_unsafe(state):
    return (int(state) & UNSAFE_ARM) != 0


def clear_robot_fault(node, warning):
    from symphony_msgs.srv import ClearFault
    client = node.create_client(ClearFault, '/status_controller/clear_fault')
    req = ClearFault.Request()
    req.warning = bool(warning)
    result = call(node, client, req, timeout=5.0)
    return result is not None and getattr(result, 'success', False)


def restart_servox(node):
    """Clear the latched stop, then start the cartesian stream again."""
    clear_robot_fault(node, True)
    time.sleep(0.3)
    clear_robot_fault(node, False)
    time.sleep(0.3)
    if not deactivate_if_active(node, CTRL):
        return False
    return activate_exclusive(node, CTRL)


def force_absolute_servo(node):
    from symphony_msgs.srv import SetMotionOptions
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
    req.servo_relative = False
    result = call(node, client, req, timeout=5.0)
    if not result or not getattr(result, 'success', False):
        node.get_logger().error(
            'could not force absolute servoX: '
            + (getattr(result, 'message', '') if result else 'timeout'))
        return False
    return True


def main():
    rclpy.init()
    node = Node('servox_3dmouse_example')

    node.declare_parameter('vid', DEFAULT_VID)
    node.declare_parameter('pid', DEFAULT_PID)
    node.declare_parameter('evdev_path', DEFAULT_EVDEV)
    node.declare_parameter('max_linear_speed', MAX_LINEAR_SPEED)
    node.declare_parameter('max_angular_speed_deg', math.degrees(MAX_ANGULAR_SPEED))
    node.declare_parameter('deadzone', 100)

    vid = int(node.get_parameter('vid').value)
    pid = int(node.get_parameter('pid').value)
    evdev_path = str(node.get_parameter('evdev_path').value)
    linear_speed = float(node.get_parameter('max_linear_speed').value)
    angular_speed = math.radians(
        float(node.get_parameter('max_angular_speed_deg').value))
    deadzone = int(node.get_parameter('deadzone').value)

    servo = wait_msg(node, '/status_controller/servo_enable', Bool)
    mode = wait_msg(node, '/status_controller/robot_mode', UInt8, timeout=3.0)
    if mode is None:
        mode = wait_msg(node, '/status_controller/op_mode', UInt8, timeout=3.0)
    if servo is None:
        node.get_logger().error('no driver status — is bringup running?')
        rclpy.shutdown()
        return 1
    opmode = int(mode.data) if mode is not None else -1
    node.get_logger().info(f'op/robot_mode={opmode} (need 3=External)')
    if opmode != OPMODE_EXT:
        node.get_logger().error('pendant must be External (mode=3)')
        rclpy.shutdown()
        return 1

    if not wait_controller(node, CTRL):
        node.get_logger().error(f'{CTRL} not loaded')
        rclpy.shutdown()
        return 1
    if not ensure_servo(node):
        rclpy.shutdown()
        return 1
    if not ensure_set_speed(node, SET_SPEED):
        rclpy.shutdown()
        return 1
    if not force_absolute_servo(node):
        rclpy.shutdown()
        return 1

    latest = [None]

    def on_pose(msg):
        latest[0] = msg

    node.create_subscription(PoseStamped, TCP_TOPIC, on_pose, 10)
    if wait_msg(node, TCP_TOPIC, PoseStamped, timeout=5.0) is None and latest[0] is None:
        node.get_logger().error(f'no {TCP_TOPIC}')
        rclpy.shutdown()
        return 1

    backend = open_backend(node, vid, pid, evdev_path)
    if backend is None:
        rclpy.shutdown()
        return 1

    node.get_logger().info(
        f'tune dt={SAMPLE_DT:.3f}s deadzone={deadzone} '
        f'max_speed={linear_speed * 1000.0:.0f}mm/s/'
        f'{math.degrees(angular_speed):.0f}deg/s '
        f'max_accel={MAX_LINEAR_ACCEL:.2f}m/s2/'
        f'{math.degrees(MAX_ANGULAR_ACCEL):.0f}deg/s2')

    pub = node.create_publisher(PoseStamped, POSE_TOPIC, 10)
    arm = {'state': 0, 'seen': False}

    def on_arm(msg):
        arm['state'] = int(msg.data)
        arm['seen'] = True

    node.create_subscription(UInt32, '/status_controller/arm_state', on_arm, 10)
    started = False
    velocity = [0.0] * 6
    target_pose = None
    try:
        told_stop = False
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not arm['seen'] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not arm['seen']:
            node.get_logger().error('no /status_controller/arm_state')
            return 1
        while rclpy.ok() and arm_is_unsafe(arm['state']):
            if not told_stop:
                node.get_logger().error(
                    f'protective stop arm_state=0x{arm["state"]:08x} — '
                    'clear it on the pendant, then the stream restarts')
                told_stop = True
            rclpy.spin_once(node, timeout_sec=0.1)
        if not rclpy.ok():
            return 0
        told_stop = False
        if not restart_servox(node):
            node.get_logger().error('could not start RT servoX')
            return 1
        started = True
        if not wait_subscribers(node, pub):
            node.get_logger().error(f'{CTRL} not subscribed')
            return 1

        node.get_logger().info(f'servoX settle {AFTER_START_S:.1f}s')
        hold(node, pub, latest, AFTER_START_S)
        if latest[0] is not None:
            target_pose = clone_pose(latest[0].pose)
        node.get_logger().info(
            'SpaceMouse teleop, base frame: '
            'X=-ty Y=-tx Z=-tz RX=-ry RY=-rx RZ=-rz; '
            'Ctrl+C stop; left button = freeze')

        last_log = 0.0
        blocked = False
        told_release = False
        last_tick = time.monotonic()
        next_tick = last_tick
        while rclpy.ok():
            now = time.monotonic()
            dt = min(0.02, max(0.0005, now - last_tick))
            last_tick = now
            raw, freeze = backend.poll()
            axes = normalize_axes(raw, deadzone)
            unsafe = arm['seen'] and arm_is_unsafe(arm['state'])
            if unsafe:
                blocked = True
                velocity = [0.0] * 6
                target_pose = None
                if not told_stop:
                    node.get_logger().error(
                        f'protective stop arm_state=0x{arm["state"]:08x} — '
                        'clear it on the pendant and release the mouse')
                    told_stop = True
            elif blocked:
                if any(v != 0 for v in raw):
                    if not told_release:
                        node.get_logger().info('release the mouse to restart servoX')
                        told_release = True
                elif restart_servox(node):
                    node.get_logger().info(
                        f'safety clear — servoX restart, settle {AFTER_START_S:.1f}s')
                    hold(node, pub, latest, AFTER_START_S)
                    velocity = [0.0] * 6
                    target_pose = clone_pose(latest[0].pose) if latest[0] is not None else None
                    blocked = False
                    told_stop = False
                    told_release = False
                else:
                    node.get_logger().error('servoX restart failed')
                    time.sleep(1.0)
            current = latest[0]
            if current is not None and not blocked:
                desired = [0.0] * 6
                if not freeze:
                    desired = list(mouse_to_base_velocity(
                        axes, linear_speed, angular_speed))
                for i in range(3):
                    velocity[i] = slew(
                        velocity[i], desired[i], MAX_LINEAR_ACCEL * dt)
                for i in range(3, 6):
                    velocity[i] = slew(
                        velocity[i], desired[i], MAX_ANGULAR_ACCEL * dt)
                if target_pose is None:
                    target_pose = clone_pose(current.pose)
                target_pose = apply_step(
                    target_pose,
                    velocity[0] * dt, velocity[1] * dt, velocity[2] * dt,
                    velocity[3] * dt, velocity[4] * dt, velocity[5] * dt)
                publish_pose(node, pub, clone_pose(target_pose))
            if now - last_log >= 1.0:
                last_log = now
                node.get_logger().info(
                    'raw=' + ','.join(f'{int(v):+d}' for v in raw) +
                    ' axis=' + ','.join(f'{v:+.2f}' for v in axes) +
                    ' vel=' + ','.join(f'{v:+.3f}' for v in velocity) +
                    (' FREEZE' if freeze else '') +
                    (' BLOCKED' if blocked else ''))
            rclpy.spin_once(node, timeout_sec=0.0)
            next_tick += SAMPLE_DT
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            elif delay < -0.02:
                next_tick = time.monotonic()
        return 0
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt — stopping')
        return 0
    finally:
        backend.close()
        if started:
            hold(node, pub, latest, BEFORE_STOP_S)
            deactivate_if_active(node, CTRL)
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
