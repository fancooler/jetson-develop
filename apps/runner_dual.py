#!/usr/bin/env python3
"""
runner_dual.py — 双臂 GR00T N1.5 控制主循环

职责：
  硬件初始化 → 传感器读取 → infer_dual.get_action() → 执行动作 → 计时统计

运行模式（config.MOCK_ACTIONS）：
  True  → 真实推理 + 不下发机械臂运动指令（时序 / 功能调试）
  False → 正式运行

动作执行逻辑（双臂关节空间控制）：
  GR00T 推理输出 ACTION_HORIZON 步关节目标位置（弧度，绝对值）
  对 EXEC_STEPS 步：
    da.move_joints('right', rcmd.joints_deg)   # 非阻塞，控制器自行插值
    da.move_joints('left',  lcmd.joints_deg)
    grip_r/l.open/close(blocking=False)
"""

import os
import sys
import time
import csv
import logging
import threading
import contextlib

# ── 压制第三方库的冗余输出 ──────────────────────────────────────────────────────
os.environ.setdefault("TRANSFORMERS_VERBOSITY", "error")   # HF transformers
os.environ.setdefault("TOKENIZERS_PARALLELISM",  "false")  # tokenizers 警告

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import config_camera as cam_cfg
import camera_topics as topics

logging.basicConfig(
    format='%(asctime)s %(levelname)-5s %(name)s: %(message)s',
    datefmt='%H:%M:%S',
    level=logging.WARNING,          # 默认只显示 WARNING+
)
logger = logging.getLogger("runner_dual")
logger.setLevel(logging.INFO)       # runner_dual 本身保留 INFO
# arm_utils 的 IK 失败等重要警告也要看到
logging.getLogger("arm_utils").setLevel(logging.WARNING)
# 夹爪 SDK 内部的 DDS 节点 logger（名如 gripper_<mac>*）只显示 ERROR+
# gripper 模块本身的 ERROR 仍然可见
class _SdkNodeFilter(logging.Filter):
    """过滤掉 xensegripper SDK 自动创建的 DDS 节点 INFO 日志。"""
    def filter(self, record):
        # 节点 logger 名形如 "gripper_72a7da225db7*"，不是我们的 "gripper"
        if record.name.startswith("gripper_") and record.levelno < logging.ERROR:
            return False
        return True
logging.root.addFilter(_SdkNodeFilter())

# 压制 albumentations 版本检查 UserWarning
import warnings
warnings.filterwarnings("ignore", category=UserWarning,
                        module="albumentations")

import config_dual as config
import infer_dual
from arm_utils import DualArm
from gripper   import XenseGripper, MockGripper


# ── 抑制 C 库 stdout/stderr（天机 SDK 打印）──────────────────────────────────

@contextlib.contextmanager
def _suppress_c_stdout():
    """临时将 stdout 重定向到 /dev/null 屏蔽 C 库噪音；保留 stderr 给 Python logger 和 SDK ERROR。"""
    devnull = os.open(os.devnull, os.O_WRONLY)
    saved_out = os.dup(1)
    try:
        os.dup2(devnull, 1)
        yield
    finally:
        os.dup2(saved_out, 1)
        os.close(devnull)
        os.close(saved_out)


# ── 三路 RealSense 摄像头（通过 ROS2 订阅 camera_publisher 节点的图像）──────

class Ros2TripleCamera(Node):
    """
    通过 ROS2 订阅 camera_publisher 发布的三路图像。
    与旧 TripleCamera 接口一致：start() / read() / close()。
    需要先在另一个终端启动 camera_publisher.py。

    后台线程跑 rclpy.spin，主循环用 read() 拿最新帧（非阻塞）。
    """

    def __init__(self):
        super().__init__('runner_dual_cam_sub')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._bridge      = CvBridge()
        self._latest      = {r: None for r in topics.ROLES}
        self._locks       = {r: threading.Lock() for r in topics.ROLES}
        self._subs        = []
        self._spin_thread = None
        self._spinning    = False

        for role in topics.ROLES:
            sub = self.create_subscription(
                Image,
                topics.TOPICS_BY_ROLE[role],
                self._make_callback(role),
                qos,
            )
            self._subs.append(sub)

    def _make_callback(self, role: str):
        def cb(msg: Image):
            try:
                bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                logger.warning(f"[cam/{role}] cv_bridge 转换失败: {e}")
                return
            rgb = bgr[:, :, ::-1].copy()  # 推理代码消费 RGB
            with self._locks[role]:
                self._latest[role] = rgb
        return cb

    def start(self) -> bool:
        """启动订阅 + 等待三路首帧。所有路到位返回 True，超时返回 False。"""
        self._spinning = True
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self,), daemon=True
        )
        self._spin_thread.start()
        logger.info("[cam] ROS2 订阅已启动，等待 camera_publisher 首帧 ...")

        deadline = time.time() + 6.0
        while time.time() < deadline:
            with_all = all(
                self._latest.get(r) is not None for r in topics.ROLES
            )
            if with_all:
                logger.info("[cam] 三路首帧全部到位")
                return True
            time.sleep(0.05)

        missing = [r for r in topics.ROLES if self._latest.get(r) is None]
        logger.error(
            f"[cam] 等待首帧超时（6s），未收到: {missing}\n"
            f"  ┌─────────────────────────────────────────────────────────────┐\n"
            f"  │  camera_publisher 似乎没在运行。请在 Jetson 上启动：        │\n"
            f"  │    ros2 launch camera_driver camera.launch.py               │\n"
            f"  │  然后再重跑 runner_dual.py                                  │\n"
            f"  └─────────────────────────────────────────────────────────────┘"
        )
        return False

    def read(self) -> tuple:
        """返回 (front_rgb, left_wrist_rgb, right_wrist_rgb)，未到帧给黑帧。"""
        return tuple(self._get(r) for r in topics.ROLES)

    def close(self):
        self._spinning = False
        try:
            self.destroy_node()
        except Exception:
            pass
        logger.info("[cam] ROS2 订阅已关闭")

    # ── 内部 ─────────────────────────────────────────────────────────────────

    def _get(self, role: str) -> np.ndarray:
        with self._locks[role]:
            f = self._latest.get(role)
        if f is None:
            return np.zeros(
                (cam_cfg.CAM_HEIGHT, cam_cfg.CAM_WIDTH, 3), dtype=np.uint8
            )
        return f.copy()


# 旧名字别名，main() 中直接 TripleCamera() 即可
TripleCamera = Ros2TripleCamera


# ── 辅助 ──────────────────────────────────────────────────────────────────────

def _ts(label: str, t0: float) -> float:
    ms = (time.time() - t0) * 1000
    logger.info(f"  [init] {label}: {ms:.0f}ms")
    return ms


# ── 主循环 ────────────────────────────────────────────────────────────────────

def main():
    logger.info("=" * 62)
    logger.info(
        f"  GR00T N1.5 双臂控制  "
        f"模式={'MOCK（真实推理，不下发指令）' if config.MOCK_ACTIONS else '正式运行'}"
    )
    logger.info("=" * 62)

    # ── 初始化 ────────────────────────────────────────────────────────────────

    t0 = time.time()
    da = DualArm()
    with _suppress_c_stdout():
        ok = da.connect()
    if not ok:
        logger.error("双臂连接失败，退出")
        return
    _ts("双臂连接", t0)

    t0 = time.time()
    if config.GRIPPER_MOCK or getattr(config, 'GRIPPER_MOCK_RIGHT', False):
        grip_r = MockGripper(name='right')
    else:
        grip_r = XenseGripper(
            mac=config.GRIPPER_MAC_RIGHT, name='right',
            vmax=config.GRIPPER_VMAX, fmax=config.GRIPPER_FMAX,
            tol=config.GRIPPER_TOL,
        )
    if config.GRIPPER_MOCK:
        grip_l = MockGripper(name='left')
    else:
        grip_l = XenseGripper(
            mac=config.GRIPPER_MAC_LEFT,  name='left',
            vmax=config.GRIPPER_VMAX, fmax=config.GRIPPER_FMAX,
            tol=config.GRIPPER_TOL,
        )
    if not grip_r.connect():
        logger.error("右夹爪连接失败，退出（检查夹爪是否开机/IP可达）")
        da.release()
        return
    if not grip_l.connect():
        logger.warning("左夹爪连接失败，降级为 MockGripper（左臂夹爪命令将被忽略）")
        grip_l = MockGripper(name='left')
    _ts("双夹爪连接", t0)

    t0 = time.time()
    rclpy.init()    # 此处之前如有 return 不会留下未 shutdown 的 ROS2 context
    cams = TripleCamera()
    if not cams.start():
        cams.close()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        grip_r.disconnect()
        grip_l.disconnect()
        da.release()
        return
    _ts("三路摄像头", t0)

    t0 = time.time()
    logger.info("  [init] 加载 GR00T N1.5 双臂模型 ...")
    policy = infer_dual.load_policy()
    _ts("GR00T 模型加载", t0)

    # 回准备位 + 双爪张开
    t0 = time.time()
    grip_r.open(blocking=True)
    grip_l.open(blocking=True)
    with _suppress_c_stdout():
        da.go_home('right')
        da.go_home('left', left_variant=1)
    _ts("回准备位 + 夹爪张开", t0)

    # go_home 完成后 SDK 状态可能回退，进入控制循环前显式切位置模式（带 sleep）
    with _suppress_c_stdout():
        if not da.enter_position_mode('both'):
            logger.error("切位置模式失败，退出")
            cams.close()
            try:
                rclpy.shutdown()
            except Exception:
                pass
            grip_r.disconnect()
            grip_l.disconnect()
            da.release()
            return

    # ── 跳过 warmup ────────────────────────────────────────────────────────────
    # 2026-05-29：HOME 已改为训练分布内的"任务起点"姿态，go_home 直接到位，
    # 不再需要"先 home 后 warmup 慢速对齐"两段移动。
    # 如需重新启用 warmup，回滚 git 即可。

    logger.info("=" * 62)
    logger.info(
        f"  控制循环  EXEC_HZ={config.EXEC_HZ}  "
        f"ACTION_HORIZON={config.ACTION_HORIZON}  "
        f"EXEC_STEPS={config.EXEC_STEPS}"
    )
    logger.info("=" * 62)

    step_dt      = 1.0 / config.EXEC_HZ
    cycle        = 0
    t_obs_list   = []
    t_infer_list = []
    t_exec_list  = []

    # ── 运动数据日志（每步记录观测关节角 + 指令关节角，Ctrl-C 不丢数据）──────
    _motion_log        = None
    _motion_log_writer = None
    _motion_log_path   = None
    try:
        _log_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'log'
        )
        os.makedirs(_log_dir, exist_ok=True)
        _motion_log_path = os.path.join(
            _log_dir,
            f"motion_{time.strftime('%Y%m%d_%H%M%S')}.csv",
        )
        _motion_log        = open(_motion_log_path, 'w', newline='')
        _motion_log_writer = csv.writer(_motion_log)
        _motion_log_writer.writerow([
            'cycle', 'step',
            'r_obs_j1', 'r_obs_j2', 'r_obs_j3', 'r_obs_j4', 'r_obs_j5', 'r_obs_j6', 'r_obs_j7',
            'r_cmd_j1', 'r_cmd_j2', 'r_cmd_j3', 'r_cmd_j4', 'r_cmd_j5', 'r_cmd_j6', 'r_cmd_j7',
            'r_cmd_ok', 'r_gripper_open',
            'l_obs_j1', 'l_obs_j2', 'l_obs_j3', 'l_obs_j4', 'l_obs_j5', 'l_obs_j6', 'l_obs_j7',
            'l_cmd_j1', 'l_cmd_j2', 'l_cmd_j3', 'l_cmd_j4', 'l_cmd_j5', 'l_cmd_j6', 'l_cmd_j7',
            'l_cmd_ok', 'l_gripper_open',
        ])
        _motion_log.flush()
        logger.info(f"运动日志: {_motion_log_path}")
    except Exception as _e:
        logger.warning(f"运动日志创建失败: {_e}")

    try:
        while True:
            cycle += 1
            t_cycle_start = time.time()

            # ── 1. 读观测 ──────────────────────────────────────────────────────
            t0 = time.time()

            # 三路图像（后台线程实时更新，非阻塞）
            front_rgb, left_wrist_rgb, right_wrist_rgb = cams.read()

            # 双臂状态：单次 subscribe → 关节角(度) + EE 位姿(base_link系 m/rad)
            states   = da.read_all_states()
            joints_r = states['joints']['right']    # [j1..j7]，度
            joints_l = states['joints']['left']
            pos_r, rpy_r = states['ee']['right']    # (ndarray[3], tuple(3))
            pos_l, rpy_l = states['ee']['left']

            if pos_r is None or pos_l is None:
                logger.error(f"[{cycle:4d}] FK 失败，跳过本帧")
                time.sleep(step_dt)
                continue

            # 夹爪位置（Xense mm）
            gr_mm = grip_r.get_position()
            gl_mm = grip_l.get_position()

            t_obs = (time.time() - t0) * 1000

            # ── 2. GR00T 推理 ──────────────────────────────────────────────────
            t0 = time.time()

            # fk_right/fk_left = (pos_m, rpy_rad)，base_link 系
            # infer_dual 内部会做 base_link → 世界系转换，喂给 GR00T state
            actions = infer_dual.get_action(
                policy,
                front_rgb,
                left_wrist_rgb,
                right_wrist_rgb,
                joints_r,           # 右臂关节角，度
                joints_l,           # 左臂关节角，度
                (pos_r, rpy_r),     # fk_right：base_link 系
                (pos_l, rpy_l),     # fk_left：base_link 系
                gr_mm,
                gl_mm,
            )

            t_infer = (time.time() - t0) * 1000

            # ── 3. 执行动作序列 ────────────────────────────────────────────────
            t0 = time.time()

            for step in range(config.EXEC_STEPS):
                t_step_deadline = (
                    t_cycle_start
                    + t_obs   / 1000.0
                    + t_infer / 1000.0
                    + (step + 1) * step_dt
                )

                rcmd = infer_dual.extract_right_arm_cmd(actions, step)
                lcmd = infer_dual.extract_left_arm_cmd(actions, step)
                # cmd.joints_deg : [j1..j7]，度，直接下发
                # cmd.gripper_open: True=张开，False=闭合

                r_ok = False
                l_ok = False

                if not config.MOCK_ACTIONS:
                    r_ok = da.move_joints('right', rcmd.joints_deg.tolist())
                    if not r_ok:
                        logger.warning(
                            f"[{cycle:4d}/{step}] 右臂关节指令被拦截"
                            f"（joints=[{', '.join(f'{v:.1f}' for v in rcmd.joints_deg)}]°）"
                        )
                    l_ok = da.move_joints('left', lcmd.joints_deg.tolist())
                    if not l_ok:
                        logger.warning(
                            f"[{cycle:4d}/{step}] 左臂关节指令被拦截"
                            f"（joints=[{', '.join(f'{v:.1f}' for v in lcmd.joints_deg)}]°）"
                        )

                    if rcmd.gripper_open:
                        grip_r.open(blocking=False)
                    else:
                        grip_r.close(blocking=False)

                    if lcmd.gripper_open:
                        grip_l.open(blocking=False)
                    else:
                        grip_l.close(blocking=False)

                else:
                    logger.debug(
                        f"[MOCK {cycle:4d}/{step}] "
                        f"R_joints=[{', '.join(f'{v:.1f}' for v in rcmd.joints_deg)}]°  "
                        f"r_grip={'open' if rcmd.gripper_open else 'close'}  "
                        f"L_joints=[{', '.join(f'{v:.1f}' for v in lcmd.joints_deg)}]°  "
                        f"l_grip={'open' if lcmd.gripper_open else 'close'}"
                    )

                # 写运动日志（每步一行，flush 确保 Ctrl-C 不丢失）
                if _motion_log_writer is not None:
                    _motion_log_writer.writerow([
                        cycle, step,
                        *[f"{j:.2f}" for j in joints_r],
                        *[f"{j:.2f}" for j in rcmd.joints_deg],
                        1 if r_ok else 0, 1 if rcmd.gripper_open else 0,
                        *[f"{j:.2f}" for j in joints_l],
                        *[f"{j:.2f}" for j in lcmd.joints_deg],
                        1 if l_ok else 0, 1 if lcmd.gripper_open else 0,
                    ])
                    _motion_log.flush()

                # 频率控制（等待到本步 deadline）
                remaining = t_step_deadline - time.time()
                if remaining > 0:
                    time.sleep(remaining)
                elif remaining < -0.010:
                    logger.warning(
                        f"[{cycle:4d}/{step}] 超时 {-remaining*1000:.1f}ms"
                    )

            t_exec = (time.time() - t0) * 1000
            t_obs_list.append(t_obs)
            t_infer_list.append(t_infer)
            t_exec_list.append(t_exec)
            t_cycle = t_obs + t_infer + t_exec

            logger.info(
                f"[{cycle:4d}] "
                f"obs={t_obs:5.1f}ms  infer={t_infer:6.1f}ms  "
                f"exec={t_exec:6.1f}ms  "
                f"R_J2={joints_r[1]:.1f}°  L_J2={joints_l[1]:.1f}°  "
                f"grip_r={gr_mm:.1f}mm  "
                f"({1000/t_cycle:.1f}Hz)"
            )

    except KeyboardInterrupt:
        logger.info("\nCtrl+C，正在安全退出 ...")

    finally:
        if _motion_log is not None:
            try:
                _motion_log.flush()
                _motion_log.close()
                logger.info(f"运动日志已保存: {_motion_log_path}")
            except Exception:
                pass
        cams.close()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        grip_r.disconnect()
        grip_l.disconnect()
        da.release()

        if t_obs_list:
            print("\n" + "=" * 62)
            print(f"  统计汇总  {len(t_obs_list)} cycles")
            print("=" * 62)
            for name, lst in [
                ("读观测      ", t_obs_list),
                ("GR00T 推理  ", t_infer_list),
                (f"执行{config.EXEC_STEPS}步      ", t_exec_list),
            ]:
                arr = np.array(lst)
                print(
                    f"  {name}: avg={arr.mean():6.1f}ms  "
                    f"min={arr.min():6.1f}ms  max={arr.max():6.1f}ms  "
                    f"std={arr.std():5.1f}ms"
                )
            total = np.array([o + i + e
                               for o, i, e in zip(t_obs_list,
                                                   t_infer_list,
                                                   t_exec_list)])
            print(
                f"\n  总 cycle avg: {total.mean():.1f}ms  "
                f"≈ {1000/total.mean():.1f} Hz  "
                f"（每 cycle 执行 {config.EXEC_STEPS} 步）"
            )
            print("=" * 62)

        logger.info("程序结束")


if __name__ == "__main__":
    main()
