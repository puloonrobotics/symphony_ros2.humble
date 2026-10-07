#!/usr/bin/env python3
"""tool_modbus_controller + OnRobot 2FG7 Modbus register sequence (Tool IO path)."""

from __future__ import annotations

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from example_common import call, wait_controller, wait_msg  # noqa: E402

from symphony_msgs.msg import ToolModbusStatus  # noqa: E402
from symphony_msgs.srv import (  # noqa: E402
    OpenToolModbus,
    ToolModbusWrite,
)

DEVICE = 65
REG_WIDTH = 0
REG_FORCE = 1
REG_SPEED = 2
REG_CMD = 3
CMD_EXTERNAL = 1
CMD_INTERNAL = 2
FORCE_N = 50
SPEED_PCT = 50
WIDTH_GRIP = 400
WIDTH_RELEASE = 700

TOOL_STATUS_ERROR = 0x0002
TOOL_STATUS_MODBUS = 0x0008

NS = '/tool_modbus_controller'


def _ok(res):
    return res is not None and bool(getattr(res, 'success', False))


def _msg(res):
    if res is None:
        return 'timeout/no service'
    return getattr(res, 'message', '') or ''


def log_step(node, name, ok, detail=''):
    text = f'{name}: {"ok" if ok else "FAIL"}'
    if detail:
        text = f'{text} ({detail})'
    if ok:
        node.get_logger().info(text)
    else:
        node.get_logger().error(text)
    return ok


def write_regs(node, address, values):
    req = ToolModbusWrite.Request()
    req.type = ToolModbusWrite.Request.TYPE_REGISTER
    req.address = int(address)
    req.values = [int(v) for v in values]
    return call(
        node,
        node.create_client(ToolModbusWrite, f'{NS}/write'),
        req,
        timeout=5.0)


def get_status(node, timeout=2.0):
    return wait_msg(node, f'{NS}/modbus_status', ToolModbusStatus, timeout=timeout)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cycles', type=int, default=1)
    p.add_argument('--skip-motion', action='store_true')
    p.add_argument('--device', type=int, default=DEVICE)
    p.add_argument(
        '--internal', action='store_true',
        help='reg3=2 internal; default reg3=1 external')
    return p.parse_args()


def main():
    args = parse_args()
    grip_type = CMD_INTERNAL if args.internal else CMD_EXTERNAL
    rclpy.init()
    node = Node('tool_modbus_2fg7_example')
    opened = False
    results = []

    def step(name, ok, detail=''):
        results.append((name, ok))
        return log_step(node, name, ok, detail)

    try:
        if not wait_controller(node, 'tool_modbus_controller', timeout=30.0):
            node.get_logger().error(
                'tool_modbus_controller not loaded — is bringup running?')
            return 1

        node.get_logger().info(
            'write service success ≠ fingers moved. '
            '0x0008 / modbus_ok is logged only (not required). '
            f'grip_type={grip_type}')

        status = get_status(node, timeout=3.0)
        step(
            'topic ~/modbus_status',
            status is not None,
            '' if status is None else
            f'modbus_ok={status.modbus_ok} io=0x{status.io_status:04X}')

        req_open = OpenToolModbus.Request()
        req_open.device = int(args.device)
        res_open = call(
            node,
            node.create_client(OpenToolModbus, f'{NS}/open_modbus'),
            req_open,
            timeout=5.0)
        opened = _ok(res_open)
        step(
            f'service ~/open_modbus(device={args.device})',
            opened,
            _msg(res_open) + ' [engine accepted open cmd]')

        time.sleep(2.5)
        status_link = get_status(node, timeout=2.0)
        io = int(getattr(status_link, 'io_status', 0) or 0) if status_link else 0
        err_bit = (io & TOOL_STATUS_ERROR) != 0
        modbus_ok = bool(status_link and status_link.modbus_ok) or (
            (io & TOOL_STATUS_MODBUS) != 0)

        step(
            'status after open',
            status_link is not None and not err_bit,
            '' if status_link is None else
            f'modbus_ok={status_link.modbus_ok} io=0x{io:04X} '
            '(0x0008 logged only, not required)')

        do_write = not args.skip_motion and opened and not err_bit

        if do_write:
            for i in range(max(0, args.cycles)):
                for label, width in (('grip', WIDTH_GRIP), ('release', WIDTH_RELEASE)):
                    ok_batch = _ok(write_regs(
                        node, REG_WIDTH, [width, FORCE_N, SPEED_PCT]))
                    time.sleep(0.3)
                    ok_c = _ok(write_regs(node, REG_CMD, [grip_type]))
                    svc_ok = ok_batch and ok_c
                    detail = (
                        f'width={width} force={FORCE_N} speed={SPEED_PCT} '
                        f'type={grip_type} write_svc={svc_ok} '
                        f'modbus_ok={modbus_ok}')
                    if svc_ok and not modbus_ok:
                        detail += ' — 0x0008 not set (logged only)'
                    step(f'2fg7 {label} cycle={i + 1}', svc_ok, detail)
                    time.sleep(4.0)
        elif args.skip_motion:
            node.get_logger().info('skip-motion: no grip/release writes')
        elif not opened:
            node.get_logger().error('skip grip/release: open_modbus failed')
        elif err_bit:
            node.get_logger().error('skip grip/release: Tool IO ERROR bit')

        time.sleep(0.5)
        status2 = get_status(node, timeout=2.0)
        step('topic ~/modbus_status (after ops)', status2 is not None)

        if opened:
            res_close = call(
                node,
                node.create_client(Trigger, f'{NS}/close_modbus'),
                Trigger.Request(),
                timeout=5.0)
            step('service ~/close_modbus', _ok(res_close), _msg(res_close))
            opened = False
            time.sleep(0.5)

        return 0 if all(ok for _, ok in results) else 1
    finally:
        if opened:
            res_close = call(
                node,
                node.create_client(Trigger, f'{NS}/close_modbus'),
                Trigger.Request(),
                timeout=5.0)
            log_step(node, 'service ~/close_modbus (finally)', _ok(res_close), _msg(res_close))
        failed = [n for n, ok in results if not ok]
        if failed:
            node.get_logger().error('failed steps: ' + ', '.join(failed))
        else:
            node.get_logger().info(f'done  passed={len(results)}/{len(results)}')
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
