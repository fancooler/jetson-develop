#!/usr/bin/env python3
"""cli.py — arm_client 命令行入口（arm_cli）

把常用机械臂操作做成子命令，替代手敲 ros2 action send_goal / service call。
跨机控制 Jetson 上的 arm_node 时，本机需先 source 工作区并设好同样的
ROS_DOMAIN_ID（与 Jetson 一致，如 ROBOT_ID=robot1 → 1）。

例：
    ros2 run arm_client arm_cli status
    ros2 run arm_client arm_cli home both
    ros2 run arm_client arm_cli joints right --right 10 20 30 -40 50 -10 5
    ros2 run arm_client arm_cli pose right --pos 0.3 -0.2 0.4 --rpy 0 0 0
    ros2 run arm_client arm_cli connect
    ros2 run arm_client arm_cli estop
    ros2 run arm_client arm_cli set-mode position
"""

import argparse
import sys

from arm_client.client import ArmClient


def _print_result(label: str, result):
    ok, msg = result
    mark = '✅' if ok else '❌'
    print(f'{mark} {label}: {msg}')
    return 0 if ok else 1


def _cmd_status(arm: ArmClient, args) -> int:
    if not arm.wait_for_servers(timeout=args.server_timeout):
        print('❌ 未发现 arm_node（检查节点是否运行、ROS_DOMAIN_ID 是否一致）', file=sys.stderr)
        return 1
    import time
    time.sleep(0.5)   # 等一帧状态/关节缓存进来
    st = arm.status
    if st is None:
        print('（暂无 /arm/status）')
    else:
        print(f'connected={st.connected}  estopped={st.estopped}  busy={st.busy}  '
              f'streaming={st.streaming}  ctrl_mode={st.ctrl_mode}')
    jd = arm.joints_dict()
    if jd:
        import math
        for side in ('left', 'right'):
            deg = [round(math.degrees(v), 1) for v in jd[side]]
            print(f'{side:>5} (°): {deg}')
    else:
        print('（暂无 /arm/joint_states）')
    return 0


def _feedback_printer():
    def cb(max_err_deg: float):
        print(f'  …运动中 max_err={max_err_deg:.2f}°', end='\r', flush=True)
    return cb


def _cmd_home(arm: ArmClient, args) -> int:
    fb = _feedback_printer() if args.feedback else None
    r = arm.go_home(args.arm, timeout=args.timeout, feedback_cb=fb)
    if args.feedback:
        print()
    return _print_result(f'go_home({args.arm})', r)


def _cmd_joints(arm: ArmClient, args) -> int:
    if not args.left and not args.right:
        print('❌ 至少给 --left 或 --right 一组 7 个关节角', file=sys.stderr)
        return 2
    fb = _feedback_printer() if args.feedback else None
    r = arm.move_to_joints(args.arm, left=args.left, right=args.right,
                           timeout=args.timeout, feedback_cb=fb)
    if args.feedback:
        print()
    return _print_result(f'move_to_joints({args.arm})', r)


def _cmd_pose(arm: ArmClient, args) -> int:
    fb = _feedback_printer() if args.feedback else None
    r = arm.move_to_pose(args.arm, position=args.pos,
                         quaternion=args.quat, rpy=args.rpy,
                         safe=not args.unsafe, timeout=args.timeout, feedback_cb=fb)
    if args.feedback:
        print()
    return _print_result(f'move_to_pose({args.arm})', r)


def _simple(method_name: str, label: str):
    def run(arm: ArmClient, args) -> int:
        return _print_result(label, getattr(arm, method_name)())
    return run


def _cmd_setmode(arm: ArmClient, args) -> int:
    return _print_result(f'set_ctrl_mode({args.mode})', arm.set_ctrl_mode(args.mode))


def _cmd_stream(arm: ArmClient, args) -> int:
    if not args.left and not args.right:
        print('❌ 至少给 --left 或 --right 一组 7 个目标关节角', file=sys.stderr)
        return 2
    ok, msg = arm.enable_streaming(True)
    if not ok:
        print(f'❌ 开启流式失败: {msg}', file=sys.stderr)
        return 1
    try:
        r = arm.stream_to(args.arm, target_left=args.left, target_right=args.right,
                          duration=args.duration, rate=args.rate)
    finally:
        arm.enable_streaming(False)
    return _print_result(f'stream_to({args.arm}, {args.duration}s@{args.rate:.0f}Hz)', r)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='arm_cli', description='通用机械臂客户端命令行')
    p.add_argument('--ns', default='/arm', help='话题命名空间（默认 /arm）')
    p.add_argument('--server-timeout', type=float, default=10.0,
                   help='等待 arm_node 就绪超时(秒)')
    sub = p.add_subparsers(dest='cmd', required=True)

    def add_motion_opts(sp):
        sp.add_argument('--timeout', type=float, default=0.0,
                        help='动作超时(秒)；<=0 用节点默认')
        sp.add_argument('--feedback', action='store_true',
                        help='运动中打印关节误差反馈')

    s = sub.add_parser('status', help='打印状态与关节角'); s.set_defaults(func=_cmd_status)

    s = sub.add_parser('home', help='回 HOME'); s.set_defaults(func=_cmd_home)
    s.add_argument('arm', nargs='?', default='both', choices=['left', 'right', 'both'])
    add_motion_opts(s)

    s = sub.add_parser('joints', help='移到绝对关节角(度)'); s.set_defaults(func=_cmd_joints)
    s.add_argument('arm', choices=['left', 'right', 'both'])
    s.add_argument('--left', type=float, nargs=7, metavar='J', help='左臂 7 关节角(°)')
    s.add_argument('--right', type=float, nargs=7, metavar='J', help='右臂 7 关节角(°)')
    add_motion_opts(s)

    s = sub.add_parser('pose', help='移末端到位姿(base_link)'); s.set_defaults(func=_cmd_pose)
    s.add_argument('arm', choices=['left', 'right'])
    s.add_argument('--pos', type=float, nargs=3, required=True, metavar=('X', 'Y', 'Z'))
    g = s.add_mutually_exclusive_group()
    g.add_argument('--rpy', type=float, nargs=3, metavar=('R', 'P', 'Y'), help='姿态 RPY(rad)')
    g.add_argument('--quat', type=float, nargs=4, metavar=('X', 'Y', 'Z', 'W'))
    s.add_argument('--unsafe', action='store_true', help='跳过 IK 软限位检查（危险）')
    add_motion_opts(s)

    for name, label, method in [
        ('connect', '连接', 'connect'),
        ('release', '释放', 'release'),
        ('estop', '软急停', 'estop'),
        ('enter-pos', '进入位置模式', 'enter_position_mode'),
    ]:
        s = sub.add_parser(name, help=label)
        s.set_defaults(func=_simple(method, label))

    s = sub.add_parser('set-mode', help='切控制模式'); s.set_defaults(func=_cmd_setmode)
    s.add_argument('mode', choices=['position', 'impedance'])

    s = sub.add_parser('stream', help='流式插值到目标关节角(度)'); s.set_defaults(func=_cmd_stream)
    s.add_argument('arm', choices=['left', 'right', 'both'])
    s.add_argument('--left', type=float, nargs=7, metavar='J', help='左臂目标 7 关节角(°)')
    s.add_argument('--right', type=float, nargs=7, metavar='J', help='右臂目标 7 关节角(°)')
    s.add_argument('--duration', type=float, default=2.0, help='插值时长(秒)')
    s.add_argument('--rate', type=float, default=20.0, help='下发频率(Hz)')

    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    arm = ArmClient(ns=args.ns)   # 内部负责 rclpy.init/shutdown
    try:
        # 非 status 命令也先确认服务端在
        if args.cmd != 'status' and not arm.wait_for_servers(timeout=args.server_timeout):
            print('❌ 未发现 arm_node（检查节点是否运行、ROS_DOMAIN_ID 是否一致）',
                  file=sys.stderr)
            return 1
        return args.func(arm, args)
    except TimeoutError as e:
        print(f'❌ 超时: {e}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        arm.shutdown()


if __name__ == '__main__':
    sys.exit(main())
