#!/usr/bin/env python3
"""client.py — 通用机械臂客户端封装（ThinkBook 侧，厂商无关）

对着通用 /arm/* 话题/动作/服务（arm_interfaces）封装出一组**阻塞式同步**方法，
让上层应用（脚本、状态机、推理 runner 等）像调函数一样控制机械臂，
不用手敲 ros2 action send_goal / service call。

设计：
  - 内部自建 Node + 后台 SingleThreadedExecutor 线程，上层无需关心 spin。
  - 服务/动作均用 *_async + future，再用 done_callback+Event 等待 → 不与
    后台 executor 抢 spin，可在任意线程调用（勿在 ROS 回调里调，会自死锁）。
  - 不依赖任何厂商 SDK，只 import arm_interfaces + 标准消息；可在 ThinkBook
    跑，通过跨机 DDS 控制 Jetson 上的 arm_node（domain 要一致）。

用法：
    from arm_client import ArmClient
    arm = ArmClient()
    arm.wait_for_servers(timeout=10.0)
    print(arm.connect())                       # (True, '已连接')
    print(arm.go_home('both'))                 # (True, '到位 max_err=0.00°')
    print(arm.move_to_joints('right', right=[10,20,30,-40,50,-10,5]))
    print(arm.status)                          # 最新 ArmStatus（缓存）
    arm.shutdown()
或用作上下文管理器：
    with ArmClient() as arm:
        arm.connect(); arm.go_home('both')
"""

import math
import threading
import time
from typing import Callable, Optional, Sequence, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, SetBool

from arm_interfaces.action import MoveToJoints, MoveToPose, GoHome
from arm_interfaces.srv import SetCtrlMode
from arm_interfaces.msg import ArmStatus, JointCommand

# 流式关节指令 QoS：须与 arm_node 的订阅一致（BestEffort, KeepLast 1）
COMMAND_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)

Result = Tuple[bool, str]               # (success, message) 统一返回
FeedbackCb = Optional[Callable[[float], None]]   # 收到 max_error_deg 时回调


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """RPY（弧度，固定轴 XYZ）→ 四元数 (x, y, z, w)。与 arm_node 同一约定。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ArmClient:
    """通用机械臂客户端。线程安全的阻塞式封装。"""

    def __init__(self, node_name: str = 'arm_client', ns: str = '/arm'):
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self.node: Node = rclpy.create_node(node_name)
        ns = ns.rstrip('/')

        # 状态缓存（后台线程写，主线程读；GIL 下读最新引用足够安全）
        self._status: Optional[ArmStatus] = None
        self._joints: Optional[JointState] = None
        self._ee = {'left': None, 'right': None}

        self.node.create_subscription(ArmStatus, f'{ns}/status', self._on_status, 10)
        self.node.create_subscription(JointState, f'{ns}/joint_states', self._on_joints, 10)
        self.node.create_subscription(
            PoseStamped, f'{ns}/ee_pose_left', lambda m: self._ee.__setitem__('left', m), 10)
        self.node.create_subscription(
            PoseStamped, f'{ns}/ee_pose_right', lambda m: self._ee.__setitem__('right', m), 10)

        # 服务客户端
        self._srv = {
            'connect':       self.node.create_client(Trigger, f'{ns}/connect'),
            'release':       self.node.create_client(Trigger, f'{ns}/release'),
            'enter_position_mode': self.node.create_client(Trigger, f'{ns}/enter_position_mode'),
            'estop':         self.node.create_client(Trigger, f'{ns}/estop'),
        }
        self._srv_setmode = self.node.create_client(SetCtrlMode, f'{ns}/set_ctrl_mode')
        self._srv_stream = self.node.create_client(SetBool, f'{ns}/enable_streaming')

        # 流式关节指令发布器
        self._pub_cmd = self.node.create_publisher(JointCommand, f'{ns}/joint_command', COMMAND_QOS)

        # 动作客户端
        from rclpy.action import ActionClient
        self._act_joints = ActionClient(self.node, MoveToJoints, f'{ns}/move_to_joints')
        self._act_pose = ActionClient(self.node, MoveToPose, f'{ns}/move_to_pose')
        self._act_home = ActionClient(self.node, GoHome, f'{ns}/go_home')

        # 后台 executor 线程负责所有回调/future 完成
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()

    # ── 生命周期 ──────────────────────────────────────────────────────────────

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.shutdown()

    def shutdown(self):
        if getattr(self, '_closed', False):
            return
        self._closed = True
        # 先让 executor 停下并等 spin 线程真正退出，再拆 node/rclpy，
        # 否则后台线程还在 spin 时销毁 context 会触发 C++ std::terminate。
        try:
            self._executor.shutdown()
        except Exception:
            pass
        if self._spin.is_alive():
            self._spin.join(timeout=2.0)
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if self._owns_rclpy and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    # ── future 等待（不抢 spin）────────────────────────────────────────────────

    @staticmethod
    def _wait(future, timeout: Optional[float]):
        ev = threading.Event()
        future.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            raise TimeoutError('等待响应超时')
        return future.result()

    # ── 就绪等待 ────────────────────────────────────────────────────────────────

    def wait_for_servers(self, timeout: float = 10.0) -> bool:
        """等待 connect 服务 + 三个动作服务全部就绪。超时返回 False。"""
        deadline_each = max(0.1, timeout)
        ok = self._srv['connect'].wait_for_service(timeout_sec=deadline_each)
        ok &= self._act_joints.wait_for_server(timeout_sec=deadline_each)
        ok &= self._act_pose.wait_for_server(timeout_sec=deadline_each)
        ok &= self._act_home.wait_for_server(timeout_sec=deadline_each)
        return bool(ok)

    # ── 状态读取（缓存最新一帧）─────────────────────────────────────────────────

    def _on_status(self, msg: ArmStatus):
        self._status = msg

    def _on_joints(self, msg: JointState):
        self._joints = msg

    @property
    def status(self) -> Optional[ArmStatus]:
        """最新收到的 ArmStatus（connected/estopped/busy/ctrl_mode），无则 None。
        来自 ~25Hz 监控流，可能有几十 ms 滞后。"""
        return self._status

    @property
    def joint_states(self) -> Optional[JointState]:
        """最新收到的 JointState（14 关节，rad）。同 status 是 ~25Hz 监控快照。
        **运动是否到位以 move_*/go_home 返回的结果为准**（驱动直读关节判定），
        不要靠刚 move 完立刻读这里——监控帧可能比动作结果晚到几十 ms。"""
        return self._joints

    def joints_dict(self) -> dict:
        """最新关节角，{'left': [..7 rad..], 'right': [..]}，无数据则空 dict。
        语义同 joint_states（监控快照，非动作完成判据）。"""
        js = self._joints
        if js is None:
            return {}
        pos = list(js.position)
        return {'left': pos[0:7], 'right': pos[7:14]} if len(pos) >= 14 else {}

    def ee_pose(self, arm: str) -> Optional[PoseStamped]:
        """最新末端位姿（base_link 系）；mock 下无 EE 返回 None。"""
        return self._ee.get(arm)

    # ── 服务 ────────────────────────────────────────────────────────────────────

    def _call_trigger(self, name: str, timeout: float) -> Result:
        cli = self._srv[name]
        if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=timeout):
            return False, f'服务 {name} 不可用'
        resp = self._wait(cli.call_async(Trigger.Request()), timeout)
        return bool(resp.success), str(resp.message)

    def connect(self, timeout: float = 10.0) -> Result:
        return self._call_trigger('connect', timeout)

    def release(self, timeout: float = 10.0) -> Result:
        return self._call_trigger('release', timeout)

    def enter_position_mode(self, timeout: float = 10.0) -> Result:
        return self._call_trigger('enter_position_mode', timeout)

    def estop(self, timeout: float = 10.0) -> Result:
        """软急停：停止下发新指令并中止当前动作。connect 可清除。"""
        return self._call_trigger('estop', timeout)

    def set_ctrl_mode(self, mode: str, timeout: float = 10.0) -> Result:
        """切控制模式 'position' | 'impedance'（impedance 由驱动决定是否支持）。"""
        cli = self._srv_setmode
        if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=timeout):
            return False, '服务 set_ctrl_mode 不可用'
        req = SetCtrlMode.Request()
        req.mode = mode
        resp = self._wait(cli.call_async(req), timeout)
        return bool(resp.success), str(resp.message)

    # ── 动作 ────────────────────────────────────────────────────────────────────

    def _send_action(self, action_client, goal, feedback_cb: FeedbackCb,
                     timeout: float) -> Result:
        if not action_client.server_is_ready() and \
                not action_client.wait_for_server(timeout_sec=timeout):
            return False, '动作服务不可用'

        fb_wrap = None
        if feedback_cb is not None:
            fb_wrap = lambda fb: feedback_cb(float(fb.feedback.max_error_deg))

        send_future = action_client.send_goal_async(goal, feedback_callback=fb_wrap)
        goal_handle = self._wait(send_future, timeout)
        if not goal_handle.accepted:
            return False, '目标被拒绝'
        # 结果等待用 None 超时（动作内部有自己的 timeout 字段控制时长）
        result = self._wait(goal_handle.get_result_async(), None)
        r = result.result
        return bool(r.success), str(r.message)

    def go_home(self, arm: str = 'both',
                timeout: float = 0.0, feedback_cb: FeedbackCb = None,
                connect_timeout: float = 10.0) -> Result:
        """回 HOME。arm: left|right|both。timeout<=0 用节点默认。"""
        g = GoHome.Goal()
        g.arm = arm
        g.timeout = float(timeout)
        return self._send_action(self._act_home, g, feedback_cb, connect_timeout)

    def move_to_joints(self, arm: str,
                       left: Optional[Sequence[float]] = None,
                       right: Optional[Sequence[float]] = None,
                       timeout: float = 0.0, feedback_cb: FeedbackCb = None,
                       connect_timeout: float = 10.0) -> Result:
        """移动到绝对关节角（**度**）。arm 含 left/both 需 left[7]，含 right/both 需 right[7]。"""
        g = MoveToJoints.Goal()
        g.arm = arm
        g.joints_left = [float(v) for v in (left or [])]
        g.joints_right = [float(v) for v in (right or [])]
        g.timeout = float(timeout)
        return self._send_action(self._act_joints, g, feedback_cb, connect_timeout)

    def move_to_pose(self, arm: str,
                     position: Sequence[float],
                     quaternion: Optional[Sequence[float]] = None,
                     rpy: Optional[Sequence[float]] = None,
                     safe: bool = True,
                     timeout: float = 0.0, feedback_cb: FeedbackCb = None,
                     connect_timeout: float = 10.0) -> Result:
        """移动末端到位姿（base_link 系）。arm: left|right。position=[x,y,z](m)；
        姿态给 quaternion=[x,y,z,w] 或 rpy=[r,p,y](rad) 二选一（默认无旋转）。"""
        g = MoveToPose.Goal()
        g.arm = arm
        g.pose.position.x = float(position[0])
        g.pose.position.y = float(position[1])
        g.pose.position.z = float(position[2])
        if quaternion is not None:
            qx, qy, qz, qw = quaternion
        elif rpy is not None:
            qx, qy, qz, qw = rpy_to_quat(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        else:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        g.pose.orientation.x = float(qx)
        g.pose.orientation.y = float(qy)
        g.pose.orientation.z = float(qz)
        g.pose.orientation.w = float(qw)
        g.safe = bool(safe)
        g.timeout = float(timeout)
        return self._send_action(self._act_pose, g, feedback_cb, connect_timeout)

    # ── 流式控制（Phase 2）──────────────────────────────────────────────────────

    def enable_streaming(self, enable: bool = True, timeout: float = 10.0) -> Result:
        """开/关流式模式。开启后用 stream_joints/stream_to 高频下发关节目标；
        开启时离散运动动作会被驱动拒绝（二者互斥）。estop 会自动关流式。"""
        cli = self._srv_stream
        if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=timeout):
            return False, '服务 enable_streaming 不可用'
        req = SetBool.Request()
        req.data = bool(enable)
        resp = self._wait(cli.call_async(req), timeout)
        return bool(resp.success), str(resp.message)

    def stream_joints(self, arm: str,
                      left: Optional[Sequence[float]] = None,
                      right: Optional[Sequence[float]] = None) -> None:
        """发布一帧流式关节目标（**度**）。需先 enable_streaming(True)。非阻塞、不等待回执。
        arm 含 left/both 须给 left[7]，含 right/both 须给 right[7]。"""
        msg = JointCommand()
        msg.arm = arm
        msg.joints_left = [float(v) for v in (left or [])]
        msg.joints_right = [float(v) for v in (right or [])]
        self._pub_cmd.publish(msg)

    def stream_to(self, arm: str,
                  target_left: Optional[Sequence[float]] = None,
                  target_right: Optional[Sequence[float]] = None,
                  duration: float = 2.0, rate: float = 20.0) -> Result:
        """从当前关节角线性插值到目标（度），按 rate Hz 流式下发，duration 秒走完（阻塞）。
        需先 enable_streaming(True)。用于平滑过渡 / 验证流式通路。"""
        cur = self.joints_dict()
        if not cur:
            return False, '无当前关节角（确认已连接、joint_states 在发）'
        cur_deg = {k: [math.degrees(v) for v in cur[k]] for k in cur}
        goals = {}
        if arm in ('left', 'both'):
            if target_left is None:
                return False, 'arm 含 left 需 target_left'
            goals['left'] = [float(v) for v in target_left]
        if arm in ('right', 'both'):
            if target_right is None:
                return False, 'arm 含 right 需 target_right'
            goals['right'] = [float(v) for v in target_right]
        if not goals:
            return False, f'arm 非法: {arm!r}'
        n = max(1, int(duration * rate))
        period = 1.0 / rate if rate > 0 else 0.05
        for step in range(1, n + 1):
            a = step / n
            left_cmd = ([(1 - a) * c + a * g for c, g in zip(cur_deg['left'], goals['left'])]
                        if 'left' in goals else None)
            right_cmd = ([(1 - a) * c + a * g for c, g in zip(cur_deg['right'], goals['right'])]
                         if 'right' in goals else None)
            self.stream_joints(arm, left=left_cmd, right=right_cmd)
            time.sleep(period)
        return True, f'流式插值完成（{n} 帧 @ {rate:.0f}Hz）'
