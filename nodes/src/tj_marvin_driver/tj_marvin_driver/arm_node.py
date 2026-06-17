#!/usr/bin/env python3
"""arm_node.py — 天机 MaRVIN 双臂 ROS2 驱动节点（Phase 1 离散高层目标 + Phase 2 流式关节控制）

薄包装 jetson-work 仓 app/arm_utils.py 的 DualArm（真机）/ MockDualArm（无硬件），
对外实现通用 arm_interfaces。安全（软限位/禁区/钳位）全部沿用 DualArm 内部逻辑。

参数：
  use_mock        (bool, true)   true→MockDualArm（不碰真机）；false→真机 DualArm
  app_dir         (str)          jetson-work 的 app 目录（含 arm_utils/config_dual），默认 ~/work/app
  publish_rate    (float, 25.0)  状态发布频率 Hz
  auto_connect    (bool, true)   启动即 connect（仅设控制模式，不运动）
  reach_tol_deg   (float, 1.0)   到位判据：最大关节误差（度）
  default_timeout (float, 30.0)  动作默认超时（秒）

发布：
  /arm/joint_states          sensor_msgs/JointState   （14 关节，rad）
  /arm/ee_pose_left|right     geometry_msgs/PoseStamped（base_link）
  /arm/status                arm_interfaces/ArmStatus
订阅：
  /arm/joint_command         arm_interfaces/JointCommand（流式关节目标，度；BestEffort/depth1）
动作：
  /arm/move_to_joints  MoveToJoints   /arm/move_to_pose  MoveToPose   /arm/go_home  GoHome
服务：
  /arm/connect /arm/release /arm/enter_position_mode /arm/estop   std_srvs/Trigger
  /arm/set_ctrl_mode   arm_interfaces/SetCtrlMode
  /arm/enable_streaming   std_srvs/SetBool（开/关流式；开启时离散动作被拒，二者互斥）

流式：enable_streaming 开启后，高频发布 /arm/joint_command 即透传到非阻塞 move_joints
      （对接 20Hz 推理/遥操作）；estop 立即关流式。流式与离散动作互斥（开流式时动作被拒）。

并发：运动动作串行（MutuallyExclusive），状态/服务用 Reentrant 组并发，
      故 estop / 状态发布能在运动过程中响应（运动循环每拍释放 SDK 锁）。
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

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_srvs.srv import Trigger, SetBool

from arm_interfaces.action import MoveToJoints, MoveToPose, GoHome
from arm_interfaces.srv import SetCtrlMode
from arm_interfaces.msg import ArmStatus, JointCommand

# 流式关节指令 QoS：最新即有效、丢旧帧（与 sensor 类似），发布端须一致
COMMAND_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)

JOINT_NAMES = ([f'left_joint{i}' for i in range(1, 8)] +
               [f'right_joint{i}' for i in range(1, 8)])


def rpy_to_quat(roll, pitch, yaw):
    """RPY（弧度，固定轴 XYZ）→ 四元数 (x, y, z, w)。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


def quat_to_rpy(x, y, z, w):
    """四元数 (x, y, z, w) → RPY（弧度）。"""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class ArmNode(Node):

    def __init__(self):
        super().__init__('tj_marvin_arm')

        self.declare_parameter('use_mock', True)
        self.declare_parameter('app_dir',     os.path.expanduser('~/develop/apps'))
        self.declare_parameter('drivers_dir', os.path.expanduser('~/develop/drivers'))
        self.declare_parameter('publish_rate', 25.0)
        self.declare_parameter('auto_connect', True)
        self.declare_parameter('reach_tol_deg', 1.0)
        self.declare_parameter('default_timeout', 30.0)

        gp = self.get_parameter
        self._use_mock = gp('use_mock').get_parameter_value().bool_value
        self._app_dir     = gp('app_dir').get_parameter_value().string_value
        self._drivers_dir = gp('drivers_dir').get_parameter_value().string_value
        rate = gp('publish_rate').get_parameter_value().double_value
        auto_connect = gp('auto_connect').get_parameter_value().bool_value
        self._tol = gp('reach_tol_deg').get_parameter_value().double_value
        self._default_timeout = gp('default_timeout').get_parameter_value().double_value

        # arm_utils 在 drivers/，config_dual 在 apps/，分别加路径
        for p in (self._drivers_dir, self._app_dir):
            if p not in sys.path:
                sys.path.insert(0, p)
        try:
            import arm_utils
            import config_dual
        except Exception as e:
            self.get_logger().fatal(
                f"无法导入 arm_utils(drivers_dir={self._drivers_dir}) "
                f"或 config_dual(app_dir={self._app_dir}): {e}")
            raise
        self._cfg = config_dual

        self._lock = threading.Lock()      # 保护一切 DualArm 访问
        self._estop = False
        self._connected = False
        self._busy = False
        self._streaming = False            # 流式控制模式（与离散动作互斥）
        self._ctrl_mode = getattr(config_dual, 'CTRL_MODE', 'position')

        ArmCls = arm_utils.MockDualArm if self._use_mock else arm_utils.DualArm
        self._da = ArmCls()
        self.get_logger().info(
            f"后端: {'MockDualArm（无硬件）' if self._use_mock else 'DualArm（真机）'}")

        motion_grp = MutuallyExclusiveCallbackGroup()   # 运动串行
        io_grp = ReentrantCallbackGroup()                # 状态/服务并发
        stream_grp = MutuallyExclusiveCallbackGroup()    # 流式指令串行（自身），与动作逻辑互斥靠 _streaming 门控

        self._pub_js = self.create_publisher(JointState, '/arm/joint_states', 10)
        self._pub_ee = {
            'left':  self.create_publisher(PoseStamped, '/arm/ee_pose_left', 10),
            'right': self.create_publisher(PoseStamped, '/arm/ee_pose_right', 10),
        }
        self._pub_wrench = {
            'left':  self.create_publisher(WrenchStamped, '/arm/wrench_left', 10),
            'right': self.create_publisher(WrenchStamped, '/arm/wrench_right', 10),
        }
        self._pub_status = self.create_publisher(ArmStatus, '/arm/status', 10)

        self.create_service(Trigger, '/arm/connect', self._srv_connect, callback_group=io_grp)
        self.create_service(Trigger, '/arm/release', self._srv_release, callback_group=io_grp)
        self.create_service(Trigger, '/arm/enter_position_mode', self._srv_enter_pos, callback_group=io_grp)
        self.create_service(Trigger, '/arm/estop', self._srv_estop, callback_group=io_grp)
        self.create_service(SetCtrlMode, '/arm/set_ctrl_mode', self._srv_set_mode, callback_group=io_grp)
        self.create_service(SetBool, '/arm/enable_streaming', self._srv_enable_streaming, callback_group=io_grp)

        self.create_subscription(JointCommand, '/arm/joint_command', self._on_joint_command,
                                 COMMAND_QOS, callback_group=stream_grp)

        self._act_joints = ActionServer(
            self, MoveToJoints, '/arm/move_to_joints', self._exec_joints,
            callback_group=motion_grp, cancel_callback=lambda gh: CancelResponse.ACCEPT)
        self._act_pose = ActionServer(
            self, MoveToPose, '/arm/move_to_pose', self._exec_pose,
            callback_group=motion_grp, cancel_callback=lambda gh: CancelResponse.ACCEPT)
        self._act_home = ActionServer(
            self, GoHome, '/arm/go_home', self._exec_home,
            callback_group=motion_grp, cancel_callback=lambda gh: CancelResponse.ACCEPT)

        period = 1.0 / rate if rate > 0 else 0.04
        self.create_timer(period, self._publish_state, callback_group=io_grp)

        if auto_connect:
            self._do_connect()

    # ── 连接 ──────────────────────────────────────────────────────────────────

    def _do_connect(self) -> bool:
        with self._lock:
            # 幂等：已连接则直接返回成功，不 release、不重连。
            # 天机 SDK 同进程内 release() 不会同步释放端口/shm（端口要进程退出才回收），
            # 重复 connect 若 release+reconnect 会 "port bind failure"。要全新连接请重启节点。
            if self._connected:
                self.get_logger().info("已连接（重复 connect 跳过，幂等）")
                return True
            try:
                ok = bool(self._da.connect())
            except Exception as e:
                self.get_logger().error(f"connect 异常: {e}")
                self._connected = False
                return False
            self._connected = ok
            if ok:
                self._estop = False
        self.get_logger().info("已连接" if self._connected else "连接失败")
        return self._connected

    # ── 状态发布 ────────────────────────────────────────────────────────────

    def _publish_state(self):
        now = self.get_clock().now().to_msg()
        if self._connected:
            states = None
            try:
                with self._lock:
                    states = self._da.read_all_states()
            except Exception as e:
                self.get_logger().warn(f"read_all_states 异常: {e}",
                                       throttle_duration_sec=5.0)
            if states is not None:
                js = JointState()
                js.header.stamp = now
                js.name = list(JOINT_NAMES)
                pos = []
                for arm in ('left', 'right'):
                    pos += [math.radians(float(v)) for v in states['joints'][arm]]
                js.position = pos
                self._pub_js.publish(js)
                for arm in ('left', 'right'):
                    p, rpy = states['ee'][arm]
                    if p is None or rpy is None:
                        continue
                    ps = PoseStamped()
                    ps.header.stamp = now
                    ps.header.frame_id = 'base_link'
                    ps.pose.position.x = float(p[0])
                    ps.pose.position.y = float(p[1])
                    ps.pose.position.z = float(p[2])
                    qx, qy, qz, qw = rpy_to_quat(float(rpy[0]), float(rpy[1]), float(rpy[2]))
                    ps.pose.orientation.x = qx
                    ps.pose.orientation.y = qy
                    ps.pose.orientation.z = qz
                    ps.pose.orientation.w = qw
                    self._pub_ee[arm].publish(ps)
                for arm in ('left', 'right'):
                    w6 = states.get('wrench', {}).get(arm)
                    if w6 is None:
                        continue
                    ws = WrenchStamped()
                    ws.header.stamp = now
                    ws.header.frame_id = f'{arm}_ee'
                    ws.wrench.force.x  = float(w6[0])
                    ws.wrench.force.y  = float(w6[1])
                    ws.wrench.force.z  = float(w6[2])
                    ws.wrench.torque.x = float(w6[3])
                    ws.wrench.torque.y = float(w6[4])
                    ws.wrench.torque.z = float(w6[5])
                    self._pub_wrench[arm].publish(ws)

        st = ArmStatus()
        st.header.stamp = now
        st.connected = self._connected
        st.estopped = self._estop
        st.busy = self._busy
        st.streaming = self._streaming
        st.ctrl_mode = self._ctrl_mode
        self._pub_status.publish(st)

    # ── 服务 ────────────────────────────────────────────────────────────────

    def _srv_connect(self, req, resp):
        ok = self._do_connect()
        resp.success = ok
        resp.message = "已连接" if ok else "连接失败"
        return resp

    def _srv_release(self, req, resp):
        with self._lock:
            try:
                self._da.release()
            except Exception as e:
                resp.success = False
                resp.message = f"release 异常: {e}"
                return resp
            self._connected = False
        resp.success = True
        resp.message = "已释放"
        return resp

    def _srv_enter_pos(self, req, resp):
        ok, msg = self._enter_position_mode()
        resp.success = ok
        resp.message = msg
        return resp

    def _enter_position_mode(self):
        with self._lock:
            if not self._connected:
                return False, "未连接"
            try:
                ok = bool(self._da.enter_position_mode('both'))
            except Exception as e:
                return False, f"异常: {e}"
        if ok:
            self._ctrl_mode = 'position'
        return ok, ("已切位置跟随模式" if ok else "切换失败")

    def _srv_estop(self, req, resp):
        self._estop = True
        self._streaming = False
        self.get_logger().warn("⛔ ESTOP：停止下发新指令并中止当前动作")
        resp.success = True
        resp.message = "estop 已触发；调用 /arm/connect 可清除"
        return resp

    def _srv_set_mode(self, req, resp):
        mode = (req.mode or '').strip().lower()
        if mode == 'position':
            ok, msg = self._enter_position_mode()
            resp.success = ok
            resp.message = msg
        elif mode == 'impedance':
            resp.success = False
            resp.message = "Phase 1 暂不支持运行时切 impedance（由 config.CTRL_MODE 在 connect 时决定）"
        else:
            resp.success = False
            resp.message = f"未知模式: {req.mode}（应为 position|impedance）"
        return resp

    # ── 流式控制 ──────────────────────────────────────────────────────────────

    def _srv_enable_streaming(self, req, resp):
        """开/关流式模式（std_srvs/SetBool）。开启时离散运动动作会被拒（互斥）。"""
        if req.data:
            if not self._connected:
                resp.success = False
                resp.message = "未连接，无法开启流式"
                return resp
            if self._estop:
                resp.success = False
                resp.message = "estop 中，先 /arm/connect 清除"
                return resp
            if self._busy:
                resp.success = False
                resp.message = "有动作正在执行，稍后再开启流式"
                return resp
            self._streaming = True
            self.get_logger().info("▶ 流式模式开启（/arm/joint_command 生效；离散动作将被拒）")
            resp.success = True
            resp.message = "流式模式已开启"
        else:
            self._streaming = False
            self.get_logger().info("⏹ 流式模式关闭")
            resp.success = True
            resp.message = "流式模式已关闭"
        return resp

    def _on_joint_command(self, msg: JointCommand):
        """流式关节目标回调：仅在流式模式 + 已连接 + 非 estop 时透传到非阻塞 move_joints。"""
        if not self._streaming or not self._connected or self._estop:
            return
        targets, err = self._parse_joint_targets(msg.arm, msg.joints_left, msg.joints_right)
        if err:
            self.get_logger().warn(f"流式指令忽略：{err}", throttle_duration_sec=2.0)
            return
        with self._lock:
            for arm, j in targets.items():
                try:
                    self._da.move_joints(arm, list(j), safe=True)
                except Exception as e:
                    self.get_logger().warn(f"流式 move_joints({arm}) 异常: {e}",
                                           throttle_duration_sec=2.0)

    # ── 运动执行 ──────────────────────────────────────────────────────────────

    def _parse_joint_targets(self, arm, joints_left, joints_right):
        """校验并组装关节目标 {arm: [7 度]}。返回 (targets, errmsg)；errmsg 非空即非法。"""
        targets = {}
        if arm in ('left', 'both'):
            if len(joints_left) != 7:
                return None, "joints_left 需 7 个"
            targets['left'] = list(joints_left)
        if arm in ('right', 'both'):
            if len(joints_right) != 7:
                return None, "joints_right 需 7 个"
            targets['right'] = list(joints_right)
        if not targets:
            return None, f"arm 非法: {arm!r}（应为 left|right|both）"
        return targets, None

    def _precheck(self):
        if not self._connected:
            return "未连接（先调用 /arm/connect）"
        if self._estop:
            return "estop 已触发（先调用 /arm/connect 清除）"
        if self._streaming:
            return "流式模式开启中（先 /arm/enable_streaming data:false）"
        return None

    def _drive_to_joints(self, targets, timeout, goal_handle, FeedbackCls, send=True):
        """驱动到关节目标并轮询到位。返回 ('ok'|'fail'|'cancel', message)。

        每拍释放 SDK 锁 → 状态发布与 estop 可并发响应。
        """
        if send:
            with self._lock:
                try:
                    for arm, j in targets.items():
                        if not self._da.move_joints(arm, list(j), safe=True):
                            return 'fail', f"move_joints({arm}) 返回 False"
                except Exception as e:
                    return 'fail', f"move_joints 异常: {e}"
        deadline = time.time() + timeout
        while rclpy.ok():
            if self._estop:
                return 'fail', "estop 中止"
            if goal_handle.is_cancel_requested:
                return 'cancel', "已取消"
            try:
                with self._lock:
                    cur = self._da.read_joints()
            except Exception as e:
                return 'fail', f"read_joints 异常: {e}"
            max_err = 0.0
            for arm, j in targets.items():
                for c, t in zip(cur[arm], j):
                    max_err = max(max_err, abs(float(c) - float(t)))
            fb = FeedbackCls()
            fb.max_error_deg = float(max_err)
            goal_handle.publish_feedback(fb)
            if max_err <= self._tol:
                return 'ok', f"到位 max_err={max_err:.2f}°"
            if time.time() > deadline:
                return 'fail', f"超时 max_err={max_err:.2f}°"
            time.sleep(0.05)
        return 'fail', "节点已关闭"

    def _finish(self, goal_handle, ResultCls, status, msg):
        r = ResultCls()
        r.message = msg
        if status == 'ok':
            r.success = True
            goal_handle.succeed()
        elif status == 'cancel':
            r.success = False
            goal_handle.canceled()
        else:
            r.success = False
            goal_handle.abort()
        self.get_logger().info(f"[{ResultCls.__module__.split('.')[0]}] {status}: {msg}")
        return r

    def _exec_joints(self, goal_handle):
        self._busy = True
        try:
            g = goal_handle.request
            err = self._precheck()
            if err:
                return self._finish(goal_handle, MoveToJoints.Result, 'fail', err)
            targets, perr = self._parse_joint_targets(g.arm, g.joints_left, g.joints_right)
            if perr:
                return self._finish(goal_handle, MoveToJoints.Result, 'fail', perr)
            timeout = g.timeout if g.timeout > 0 else self._default_timeout
            status, msg = self._drive_to_joints(targets, timeout, goal_handle, MoveToJoints.Feedback)
            return self._finish(goal_handle, MoveToJoints.Result, status, msg)
        finally:
            self._busy = False

    def _exec_pose(self, goal_handle):
        self._busy = True
        try:
            g = goal_handle.request
            err = self._precheck()
            if err:
                return self._finish(goal_handle, MoveToPose.Result, 'fail', err)
            arm = g.arm
            if arm not in ('left', 'right'):
                return self._finish(goal_handle, MoveToPose.Result, 'fail', f"arm 须 left/right: {arm!r}")
            p, o = g.pose.position, g.pose.orientation
            pos = [p.x, p.y, p.z]
            rpy = list(quat_to_rpy(o.x, o.y, o.z, o.w))
            with self._lock:
                try:
                    ok = bool(self._da.move_to_ee_base(arm, pos, rpy, bool(g.safe)))
                except Exception as e:
                    return self._finish(goal_handle, MoveToPose.Result, 'fail', f"move_to_ee_base 异常: {e}")
                target = None
                lij = getattr(self._da, 'last_ik_joints', None)
                if callable(lij):
                    target = lij(arm)
            if not ok:
                return self._finish(goal_handle, MoveToPose.Result, 'fail', "IK 无解/越界/禁区，已拦截")
            if not target:
                return self._finish(goal_handle, MoveToPose.Result, 'ok', "已下发（无关节反馈，未轮询到位）")
            timeout = g.timeout if g.timeout > 0 else self._default_timeout
            status, msg = self._drive_to_joints({arm: list(target)}, timeout, goal_handle,
                                                MoveToPose.Feedback, send=False)
            return self._finish(goal_handle, MoveToPose.Result, status, msg)
        finally:
            self._busy = False

    def _exec_home(self, goal_handle):
        self._busy = True
        try:
            g = goal_handle.request
            err = self._precheck()
            if err:
                return self._finish(goal_handle, GoHome.Result, 'fail', err)
            arm = g.arm
            targets = {}
            if arm in ('left', 'both'):
                targets['left'] = list(self._cfg.HOME_JOINTS_LEFT)
            if arm in ('right', 'both'):
                targets['right'] = list(self._cfg.HOME_JOINTS_RIGHT)
            if not targets:
                return self._finish(goal_handle, GoHome.Result, 'fail', f"arm 非法: {arm!r}")
            timeout = g.timeout if g.timeout > 0 else self._default_timeout
            status, msg = self._drive_to_joints(targets, timeout, goal_handle, GoHome.Feedback)
            return self._finish(goal_handle, GoHome.Result, status, msg)
        finally:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node._connected and not node._use_mock:
                node._da.release()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
