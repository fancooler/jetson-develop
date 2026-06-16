#!/usr/bin/env python3
"""gripper_node.py — Xense 双夹爪 ROS2 驱动节点（独立于机械臂节点）

薄包装 jetson-work 仓 app/gripper.py 的 XenseGripper（真机）/ MockGripper（无硬件），
对外提供通用 gripper_interfaces。单节点管左右两爪，每爪可独立 mock。

与机械臂节点（tj_marvin_arm）的关系：
  **完全独立、可并行**。Xense 夹爪是网络设备（按 MAC 寻址），与天机臂控制器无共享资源，
  两个节点各跑各的、话题命名空间分开（/arm/* vs /gripper/*）。系统级急停应分别调
  /arm/estop 与 /gripper/estop。

参数：
  mock_left / mock_right (bool, true)  各爪 true→MockGripper（不碰硬件）
  app_dir   (str)   含 gripper.py 的 app 目录（默认 ~/work/app；ThinkBook 上是 ~/work/jetson-work/app）
  mac_left / mac_right (str)  Xense MAC（无冒号小写）
  vmax (float,80) fmax (float,27) tol (float,2.0)  速度/力/到位容差
  publish_rate (float,25) auto_connect (bool,true) default_timeout (float,10)

发布：/gripper/status   gripper_interfaces/GripperStatus（两爪 position/force/.. + estopped）
订阅：/gripper/command  gripper_interfaces/GripperCommand（流式目标位置 mm，BestEffort/depth10）
动作：/gripper/grip     gripper_interfaces/Grip（阻塞到位）
服务：/gripper/connect /gripper/estop /gripper/open /gripper/close   std_srvs/Trigger

并发：动作串行（MutuallyExclusive），状态/服务/流式并发（Reentrant/独立组），
      **每个夹爪一把锁串行化其 SDK 访问**——避免状态轮询与下发并发调用 SDK
      （xensegripper 底层非线程安全时的偶发报错根因之一）。
"""

import math
import os
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Header
from std_srvs.srv import Trigger

from gripper_interfaces.msg import GripperState, GripperStatus, GripperCommand
from gripper_interfaces.action import Grip

SIDES = ('left', 'right')
NAN = float('nan')

# depth 提到 10：一个控制周期可能连发 left+right 两条，depth=1 会让先到的被后到的覆盖
# （曾导致先发的 left 指令被 right 挤掉、左爪不跟随）。仍 BestEffort：最新优先、不重传。
COMMAND_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)


class XenseGripperNode(Node):

    def __init__(self):
        super().__init__('xense_gripper')

        self.declare_parameter('mock_left', True)
        self.declare_parameter('mock_right', True)
        self.declare_parameter('app_dir', os.path.expanduser('~/work/app'))
        self.declare_parameter('mac_left', '3ad820773a85')
        self.declare_parameter('mac_right', '72a7da225db7')
        self.declare_parameter('vmax', 80.0)
        self.declare_parameter('fmax', 27.0)
        self.declare_parameter('tol', 2.0)
        self.declare_parameter('publish_rate', 25.0)
        self.declare_parameter('auto_connect', True)
        self.declare_parameter('default_timeout', 10.0)

        gp = self.get_parameter
        mock = {'left':  gp('mock_left').value, 'right': gp('mock_right').value}
        mac = {'left':  gp('mac_left').value,  'right': gp('mac_right').value}
        self._app_dir = os.path.expanduser(gp('app_dir').value)
        self._vmax = float(gp('vmax').value)
        self._fmax = float(gp('fmax').value)
        self._tol = float(gp('tol').value)
        rate = float(gp('publish_rate').value)
        auto_connect = bool(gp('auto_connect').value)
        self._default_timeout = float(gp('default_timeout').value)

        # 引入 jetson-work 的 gripper.py（XenseGripper 在 connect 时才懒加载 xensegripper SDK）
        if self._app_dir not in sys.path:
            sys.path.insert(0, self._app_dir)
        try:
            import gripper as gmod
        except Exception as e:
            self.get_logger().fatal(f"无法从 app_dir={self._app_dir} 导入 gripper.py: {e}")
            raise
        self._gmod = gmod
        self.POS_OPEN = float(getattr(gmod, 'POS_OPEN', 85.0))
        self.POS_CLOSE = float(getattr(gmod, 'POS_CLOSE', 2.0))

        self._estop = False
        self._locks = {s: threading.Lock() for s in SIDES}
        self._connected = {s: False for s in SIDES}
        self._last_target = {s: None for s in SIDES}

        self._grippers = {}
        for s in SIDES:
            if mock[s]:
                self._grippers[s] = gmod.MockGripper(name=s)
                self.get_logger().info(f"[{s}] MockGripper（无硬件）")
            else:
                self._grippers[s] = gmod.XenseGripper(
                    mac=mac[s], name=s, vmax=self._vmax, fmax=self._fmax, tol=self._tol)
                self.get_logger().info(f"[{s}] XenseGripper MAC={mac[s]}")

        io_grp = ReentrantCallbackGroup()
        motion_grp = MutuallyExclusiveCallbackGroup()
        stream_grp = MutuallyExclusiveCallbackGroup()

        self._pub_status = self.create_publisher(GripperStatus, '/gripper/status', 10)

        self.create_service(Trigger, '/gripper/connect', self._srv_connect, callback_group=io_grp)
        self.create_service(Trigger, '/gripper/estop', self._srv_estop, callback_group=io_grp)
        self.create_service(Trigger, '/gripper/open', self._srv_open, callback_group=io_grp)
        self.create_service(Trigger, '/gripper/close', self._srv_close, callback_group=io_grp)

        self.create_subscription(GripperCommand, '/gripper/command', self._on_command,
                                 COMMAND_QOS, callback_group=stream_grp)

        self._act_grip = ActionServer(
            self, Grip, '/gripper/grip', self._exec_grip,
            callback_group=motion_grp, cancel_callback=lambda gh: CancelResponse.ACCEPT)

        period = 1.0 / rate if rate > 0 else 0.04
        self.create_timer(period, self._publish_status, callback_group=io_grp)

        if auto_connect:
            self._do_connect()

    # ── 连接 ──────────────────────────────────────────────────────────────────

    def _do_connect(self) -> bool:
        all_ok = True
        for s in SIDES:
            with self._locks[s]:
                try:
                    ok = bool(self._grippers[s].connect())
                except Exception as e:
                    self.get_logger().error(f"[{s}] connect 异常: {e}")
                    ok = False
                self._connected[s] = ok
            all_ok = all_ok and ok
            self.get_logger().info(f"[{s}] {'已连接' if ok else '连接失败'}")
        if all_ok:
            self._estop = False
        return all_ok

    # ── SDK 访问（均加锁串行化）────────────────────────────────────────────────

    def _set_position(self, side: str, pos_mm: float, blocking: bool, timeout: float) -> bool:
        with self._locks[side]:
            try:
                ok = bool(self._grippers[side].set_position(pos_mm, blocking=blocking, timeout=timeout))
            except Exception as e:
                self.get_logger().warn(f"[{side}] set_position 异常: {e}", throttle_duration_sec=2.0)
                return False
        self._last_target[side] = float(pos_mm)
        return ok

    def _get_position(self, side: str) -> float:
        with self._locks[side]:
            try:
                return float(self._grippers[side].get_position())
            except Exception as e:
                self.get_logger().warn(f"[{side}] get_position 异常: {e}", throttle_duration_sec=5.0)
                return -1.0

    def _read_state(self, side: str) -> dict:
        """读一帧状态 dict（position/velocity/force/temperature）。
        优先用 gripper.py 的 get_status()（单次 SDK 调用拿全量，对真机网络/并发更友好）；
        没有就退回 get_position() 只拿位置，其余键缺省。"""
        g = self._grippers[side]
        fn = getattr(g, 'get_status', None)
        with self._locks[side]:
            try:
                if callable(fn):
                    st = fn()
                    if isinstance(st, dict) and st:
                        return st
                return {'position': float(g.get_position())}
            except Exception as e:
                self.get_logger().warn(f"[{side}] 读状态异常: {e}", throttle_duration_sec=5.0)
                return {}

    # ── 状态发布 ────────────────────────────────────────────────────────────────

    def _publish_status(self):
        msg = GripperStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.estopped = self._estop
        for s in SIDES:
            gs = GripperState()
            gs.name = s
            gs.connected = self._connected[s]
            st = self._read_state(s) if self._connected[s] else {}
            pos = float(st.get('position', NAN))
            gs.position = pos
            gs.velocity = float(st.get('velocity', NAN))
            gs.force = float(st.get('force', NAN))
            gs.temperature = float(st.get('temperature', NAN))
            tgt = self._last_target[s]
            gs.moving = bool(tgt is not None and not math.isnan(pos) and pos >= 0
                             and abs(pos - tgt) > self._tol)
            msg.grippers.append(gs)
        self._pub_status.publish(msg)

    # ── 服务 ────────────────────────────────────────────────────────────────────

    def _srv_connect(self, req, resp):
        ok = self._do_connect()
        resp.success = ok
        resp.message = "两爪已连接" if ok else "部分/全部连接失败"
        return resp

    def _srv_estop(self, req, resp):
        self._estop = True
        self.get_logger().warn("⛔ 夹爪 ESTOP：停止下发新指令")
        resp.success = True
        resp.message = "estop 已触发；/gripper/connect 可清除"
        return resp

    def _preset_both(self, pos_mm: float, label: str):
        if self._estop:
            return False, "estop 中（先 /gripper/connect 清除）"
        any_ok = False
        for s in SIDES:
            if self._connected[s]:
                any_ok = self._set_position(s, pos_mm, blocking=False, timeout=self._default_timeout) or any_ok
        return (any_ok, f"已下发{label}") if any_ok else (False, "无已连接夹爪")

    def _srv_open(self, req, resp):
        resp.success, resp.message = self._preset_both(self.POS_OPEN, "张开")
        return resp

    def _srv_close(self, req, resp):
        resp.success, resp.message = self._preset_both(self.POS_CLOSE, "闭合")
        return resp

    # ── 流式指令 ──────────────────────────────────────────────────────────────

    def _on_command(self, msg: GripperCommand):
        if self._estop:
            return
        sides = self._parse_sides(msg.side)
        if not sides:
            self.get_logger().warn(f"流式指令 side 非法: {msg.side!r}", throttle_duration_sec=2.0)
            return
        pos = self._clip(msg.position)
        for s in sides:
            if self._connected[s]:
                self._set_position(s, pos, blocking=False, timeout=self._default_timeout)

    # ── 动作：阻塞到位 ──────────────────────────────────────────────────────────

    def _exec_grip(self, goal_handle):
        g = goal_handle.request
        if self._estop:
            return self._finish_grip(goal_handle, False, "estop 中（先 /gripper/connect）", [])
        sides = self._parse_sides(g.side)
        if not sides:
            return self._finish_grip(goal_handle, False, f"side 非法: {g.side!r}", [])
        if not all(self._connected[s] for s in sides):
            return self._finish_grip(goal_handle, False, "目标夹爪未连接", [])

        pos = self._clip(g.position)
        timeout = g.timeout if g.timeout > 0 else self._default_timeout
        for s in sides:
            self._set_position(s, pos, blocking=False, timeout=timeout)

        deadline = time.time() + timeout
        while rclpy.ok():
            if self._estop:
                return self._finish_grip(goal_handle, False, "estop 中止", [self._get_position(s) for s in sides])
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                r = Grip.Result(); r.success = False; r.message = "已取消"
                r.position = [self._get_position(s) for s in sides]
                return r
            cur = {s: self._get_position(s) for s in sides}
            max_err = max(abs(cur[s] - pos) for s in sides)
            fb = Grip.Feedback(); fb.max_error_mm = float(max_err)
            goal_handle.publish_feedback(fb)
            if max_err <= self._tol:
                return self._finish_grip(goal_handle, True, f"到位 max_err={max_err:.2f}mm",
                                         [cur[s] for s in sides])
            if time.time() > deadline:
                return self._finish_grip(goal_handle, False, f"超时 max_err={max_err:.2f}mm",
                                         [cur[s] for s in sides])
            time.sleep(0.05)
        return self._finish_grip(goal_handle, False, "节点已关闭", [])

    def _finish_grip(self, goal_handle, ok, msg, positions):
        r = Grip.Result()
        r.success = ok
        r.message = msg
        r.position = [float(p) for p in positions]
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(f"[grip] {'ok' if ok else 'fail'}: {msg}")
        return r

    # ── 辅助 ────────────────────────────────────────────────────────────────────

    @staticmethod
    def _parse_sides(side: str):
        side = (side or '').strip().lower()
        if side == 'both':
            return list(SIDES)
        if side in SIDES:
            return [side]
        return []

    def _clip(self, pos_mm: float) -> float:
        return float(max(self.POS_CLOSE, min(self.POS_OPEN, float(pos_mm))))


def main(args=None):
    rclpy.init(args=args)
    node = XenseGripperNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for s in SIDES:
            try:
                node._grippers[s].disconnect()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
