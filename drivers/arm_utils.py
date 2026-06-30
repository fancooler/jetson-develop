"""
arm_utils.py — 机械臂封装（单臂 + 双臂）

层次：
  ┌──────────────────────────────────────────────────────────┐
  │  公开接口（base_link 坐标系，米 / 弧度）                  │
  │  SingleArm('left'/'right')    DualArm()                  │
  └───────────────┬──────────────────────┬───────────────────┘
                  │                      │
           ┌──────▼──────┐       ┌───────▼──────┐
           │  _ArmCore   │       │  _ArmCore × 2│   内部，臂基坐标系
           └─────────────┘       └──────────────┘
                  │
           ┌──────▼──────────┐
           │ frame_transform │   臂基 ↔ base_link 坐标变换
           └─────────────────┘

坐标系约定：
  公开接口  — base_link 坐标系，位置单位米，姿态弧度（roll/pitch/yaw，XYZ 内旋）
  内部/SDK  — 各臂 FK/IK 基坐标系，位置 mm，姿态 ABC（ZYX 欧拉角，度）

臂标识：
  'left'  → SDK arm='A', outputs index=0, URDF joint_10
  'right' → SDK arm='B', outputs index=1, URDF joint_1

纯工具函数（无硬件依赖，可单独导入测试）：
  check_joints_in_limits / clamp_delta_pos / clamp_delta_rpy / apply_ee_delta
"""

import os
import sys
import math
import time
import logging
from typing import Optional

import numpy as np

from frame_transform import fk_to_base, base_to_fk
import joint_map   # HOME_JOINTS 是 URDF/模型约定，下发前换算到 SDK（见 joint_map.py）
from wrench_source import SDKWrenchSource, MockWrenchSource

logger = logging.getLogger(__name__)

# ── 臂参数映射 ─────────────────────────────────────────────────────────────────
_ARM_CMD = {'left': 'A', 'right': 'B'}   # SDK set_joint_position_cmd / set_position_state
_ARM_IDX = {'left':  0,  'right':  1 }   # subscribe()['outputs'][idx]


# ── 懒加载辅助 ─────────────────────────────────────────────────────────────────

def _load_sdk():
    """懒加载天机 SDK（仅在创建 SingleArm / DualArm 时触发）。"""
    import config_dual as _cfg
    sys.path.insert(0, _cfg.TJ_SDK)
    from SDK_PYTHON.fx_robot import Concise_Marvin_Robot, DCSS
    from SDK_PYTHON.fx_kine  import Marvin_Kine, FX_InvKineSolvePara
    return Concise_Marvin_Robot, DCSS, Marvin_Kine, FX_InvKineSolvePara


def _cfg():
    """懒加载 config 模块。"""
    import config_dual as config
    return config


# ═══════════════════════════════════════════════════════════════════════════════
# 纯工具函数（无硬件依赖）
# ═══════════════════════════════════════════════════════════════════════════════

# 天机 MaRVIN 关节软限位（度）—— 与 ccs_m6_40.MvKDCfg SDK 限位一致
_JOINT_LIMITS_DEG = [
    (-170.0, 170.0),   # J1  SDK: -170 ~ +170
    (-100.0, 120.0),   # J2  硬件 ±120°，下限收到 -100° 以覆盖训练分布（min ≈ -97°）；link3 撞柱靠 workspace 禁区另外管
    (-170.0, 170.0),   # J3  SDK: -170 ~ +170
    (-145.0,  60.0),   # J4  SDK: -145 ~ +60
    (-170.0, 170.0),   # J5  SDK: -170 ~ +170
    ( -60.0,  60.0),   # J6  SDK:  -60 ~ +60
    ( -90.0,  90.0),   # J7  SDK:  -90 ~ +90
]


def check_joints_in_limits(joints: list) -> bool:
    """关节角软限位检查（度）。全部在限位内返回 True。"""
    for i, (j, (lo, hi)) in enumerate(zip(joints, _JOINT_LIMITS_DEG)):
        if not (lo <= j <= hi):
            logger.debug(f"J{i+1}={j:.1f}° 超出软限位 [{lo}, {hi}]")
            return False
    return True


def check_workspace(pos_m) -> bool:
    """
    检查目标位置是否在允许工作空间内（不在任何禁区中）。

    Args:
        pos_m: [x, y, z]，base_link 坐标系，米

    Returns:
        True = 位置安全；False = 落入禁区
    """
    forbidden = getattr(_cfg(), 'WORKSPACE_FORBIDDEN', [])
    x, y, z = pos_m[0], pos_m[1], pos_m[2]
    for (x0, x1, y0, y1, z0, z1) in forbidden:
        if x0 <= x <= x1 and y0 <= y <= y1 and z0 <= z <= z1:
            logger.debug(f"目标 [{x:.3f},{y:.3f},{z:.3f}]m 落入禁区 "
                         f"X[{x0},{x1}] Y[{y0},{y1}] Z[{z0},{z1}]")
            return False
    return True


def clamp_delta_pos(delta_m: np.ndarray,
                    max_step_m: float = 0.05) -> np.ndarray:
    """
    限制单步位置增量幅度。超出时等比缩放，方向不变。

    Args:
        delta_m:    位置增量 [dx,dy,dz]，米
        max_step_m: 单步最大位移，默认 50 mm
    """
    delta_m = np.asarray(delta_m, dtype=np.float64)
    norm = np.linalg.norm(delta_m)
    if norm > max_step_m:
        return delta_m / norm * max_step_m
    return delta_m.copy()


def clamp_delta_rpy(delta_rpy: np.ndarray,
                    max_step_rad: float = 0.30) -> np.ndarray:
    """
    限制单步姿态增量幅度（各轴独立 clip，约 ±17°）。

    Args:
        delta_rpy:    姿态增量 [droll,dpitch,dyaw]，弧度
        max_step_rad: 各轴最大转角，默认 0.30 rad
    """
    return np.clip(delta_rpy, -max_step_rad, max_step_rad)


def apply_ee_delta(current_pos_m:   np.ndarray,
                   current_rpy_rad: np.ndarray,
                   delta_pos_m:     np.ndarray,
                   delta_rpy_rad:   np.ndarray,
                   safe:            bool = True) -> tuple:
    """
    将 EE 增量叠加到当前位姿，返回新目标位姿。

    Args:
        current_pos_m:   当前位置 [x,y,z]，米，base_link 坐标系
        current_rpy_rad: 当前姿态 [roll,pitch,yaw]，弧度
        delta_pos_m:     位置增量，米
        delta_rpy_rad:   姿态增量，弧度
        safe:            True = 叠加前对增量做安全 clamp

    Returns:
        (new_pos_m, new_rpy_rad): numpy arrays
    """
    p  = np.asarray(current_pos_m,   dtype=np.float64)
    r  = np.asarray(current_rpy_rad, dtype=np.float64)
    dp = np.asarray(delta_pos_m,     dtype=np.float64)
    dr = np.asarray(delta_rpy_rad,   dtype=np.float64)
    if safe:
        dp = clamp_delta_pos(dp)
        dr = clamp_delta_rpy(dr)
    return p + dp, r + dr


# ═══════════════════════════════════════════════════════════════════════════════
# _ArmCore — 单臂运动学核心（内部，不对外暴露）
# ═══════════════════════════════════════════════════════════════════════════════

class _ArmCore:
    """
    单臂运动学核心：FK / IK / 关节指令下发。
    不持有网络连接，连接由 SingleArm 或 DualArm 管理并注入。
    所有输入输出均为臂基坐标系（mm / 度）。
    """

    def __init__(self, arm: str, robot, dcss, ini: dict,
                 Marvin_Kine_cls, FX_IKPara_cls):
        assert arm in ('left', 'right'), \
            f"arm 须为 'left' 或 'right'，实为 {arm!r}"
        self.arm_str        = arm
        self.arm_cmd        = _ARM_CMD[arm]
        self.arm_idx        = _ARM_IDX[arm]
        self._robot         = robot
        self._dcss          = dcss
        self._joints: list  = [0.0] * 7
        self._FX_IKPara_cls = FX_IKPara_cls
        self._last_ik_joints: Optional[list] = None

        self.kk = Marvin_Kine_cls()
        self.kk.log_switch(0)
        self.kk.initial_kine(
            robot_type=ini['TYPE'][self.arm_idx],
            dh=ini['DH'][self.arm_idx],
            pnva=ini['PNVA'][self.arm_idx],
            j67=ini['BD'][self.arm_idx],
        )

    # ── 状态 ──────────────────────────────────────────────────────────────────

    def read_joints_from(self, data: dict) -> list:
        """从 subscribe 数据中提取本臂关节角，更新缓存并返回。"""
        fb = data['outputs'][self.arm_idx].get('fb_joint_pos')
        if fb:
            self._joints = list(fb)
        return self._joints

    def get_fk_xyzabc(self, joints: Optional[list] = None) -> Optional[list]:
        """正运动学 → [X_mm, Y_mm, Z_mm, A_deg, B_deg, C_deg]，失败返回 None。"""
        if joints is None:
            joints = self._joints
        fk_mat = self.kk.fk(joints)
        if not fk_mat:
            logger.warning(f"[{self.arm_str}] FK 失败")
            return None
        xyzabc = self.kk.mat4x4_to_xyzabc(fk_mat)
        if not xyzabc:
            logger.warning(f"[{self.arm_str}] mat4x4_to_xyzabc 失败")
            return None
        return xyzabc

    # ── IK ────────────────────────────────────────────────────────────────────

    def solve_ik(self, xyzabc_mm_deg: list) -> Optional[list]:
        """逆运动学 → 关节角（度），无解/超限返回 None。"""
        tcp_mat = self.kk.xyzabc_to_mat4x4(xyzabc_mm_deg)
        if not tcp_mat:
            logger.warning(f"[{self.arm_str}] XYZABC→mat4x4 失败")
            self._last_ik_joints = None
            return None

        tcp_flat = [tcp_mat[r][c] for r in range(4) for c in range(4)]
        sp = self._FX_IKPara_cls()
        sp.set_input_ik_target_tcp(tcp_flat)

        ref = list(self._joints)
        if abs(ref[3]) < 0.5:
            ref[3] = 1.0           # J4≈0 时给非零参考，避免万向节锁
        sp.set_input_ik_ref_joint(ref)
        sp.set_input_ik_zsp_type(0)

        res = self.kk.ik(sp)
        if not res:
            logger.warning(f"[{self.arm_str}] IK 无解")
            self._last_ik_joints = None
            return None
        if res.m_Output_IsOutRange:
            logger.warning(f"[{self.arm_str}] IK 超出工作空间")
            self._last_ik_joints = None
            return None
        if res.m_Output_IsJntExd:
            logger.warning(f"[{self.arm_str}] IK 超出关节软限位")
            self._last_ik_joints = None
            return None

        joints = res.m_Output_RetJoint.to_list()
        self._joints = joints
        self._last_ik_joints = joints
        return joints

    # ── 指令 ──────────────────────────────────────────────────────────────────

    def send_joints(self, joints: list) -> bool:
        """非阻塞下发关节位置指令。

        若首次失败（SDK 不在位置模式），自动重切位置模式后重试一次。
        """
        ok = self._robot.set_joint_position_cmd(arm=self.arm_cmd, joint=joints)
        if ok:
            return True
        # 重切位置模式 + 重试（go_home 完成后 SDK 状态可能回退）
        cfg = _cfg()
        if not self._robot.set_position_state(
            arm=self.arm_cmd,
            velRatio=cfg.VEL_RATIO,
            AccRatio=cfg.ACC_RATIO,
        ):
            return False
        time.sleep(0.3)
        return self._robot.set_joint_position_cmd(arm=self.arm_cmd, joint=joints)


# ═══════════════════════════════════════════════════════════════════════════════
# 共享连接辅助
# ═══════════════════════════════════════════════════════════════════════════════

def _wait_joints_reached(robot, dcss, arm_str: str,
                          target_joints: list,
                          tol: float, timeout: float) -> bool:
    """
    轮询直到指定臂的关节角与目标关节角之差均在 tol 内，超时返回 False。
    """
    idx = _ARM_IDX[arm_str]
    deadline = time.time() + timeout
    while time.time() < deadline:
        data = robot.subscribe(dcss)
        fb = data['outputs'][idx].get('fb_joint_pos')
        if fb and all(abs(c - t) < tol for c, t in zip(fb, target_joints)):
            return True
        time.sleep(0.05)
    logger.warning(f"[{arm_str}] 等待到位超时 ({timeout:.0f}s)")
    return False


def _do_connect(robot, dcss, cfg) -> Optional[dict]:
    """
    执行 TCP 连接、UDP 验证、运动学配置加载。
    成功返回 ini 字典；失败返回 None（robot 已被 release）。
    """
    logger.info(f"连接机械臂 {cfg.ROBOT_IP} ...")
    if not robot.connect(robot_ip=cfg.ROBOT_IP, log_switch=0):
        logger.error("连接失败，请检查网线和 IP")
        return None

    # UDP 数据帧验证
    prev, cnt = None, 0
    for _ in range(10):
        d = robot.subscribe(dcss)
        f = d['outputs'][0]['frame_serial']
        if f != 0 and f != prev:
            cnt += 1
            prev = f
        time.sleep(0.01)
    if cnt == 0:
        logger.error("UDP 数据帧未更新，请检查防火墙")
        robot.release_robot()
        return None

    robot.check_error_and_clear()

    if not os.path.exists(cfg.CFG_FILE):
        logger.error(f"运动学配置文件不存在: {cfg.CFG_FILE}")
        robot.release_robot()
        return None

    # 加载运动学配置（含双臂 DH/PNVA/BD 参数）
    _, _, MK, _ = _load_sdk()
    kk_loader = MK()
    kk_loader.log_switch(0)
    ini = kk_loader.load_config(arm_type=0, config_path=cfg.CFG_FILE)
    if not ini:
        logger.error("运动学配置加载失败")
        robot.release_robot()
        return None

    logger.info("连接成功，运动学配置已加载")
    return ini


def _go_home_arms(robot, dcss, arms: list, cfg) -> bool:
    """将指定臂列表运动到准备位（位置模式，阻塞等待，超时 60s）。"""
    home_map = {
        'left':  cfg.HOME_JOINTS_LEFT,
        'right': cfg.HOME_JOINTS_RIGHT,
    }
    # HOME_JOINTS 是 URDF/模型约定（= action mean）→ 换算到 SDK 约定再下发/比对。
    # 映射未标定/未启用时 urdf_to_sdk 恒等返回，等价改造前行为。
    if joint_map.is_active():
        for a in arms:
            sdk_home = list(joint_map.urdf_to_sdk(a, home_map[a]))
            logger.info(f"[{a}] HOME URDF={[round(v,1) for v in home_map[a]]}° "
                        f"→ SDK={[round(v,1) for v in sdk_home]}°")
    home_map = {a: list(joint_map.urdf_to_sdk(a, q)) for a, q in home_map.items()}
    for a in arms:
        cmd = _ARM_CMD[a]
        vel = getattr(cfg, 'HOME_VEL_RATIO', cfg.VEL_RATIO)
        if not robot.set_position_state(arm=cmd,
                                        velRatio=vel,
                                        AccRatio=vel):
            logger.error(f"[{a}] 切换位置模式失败")
            return False
        time.sleep(0.3)
        if not robot.set_joint_position_cmd(arm=cmd, joint=home_map[a]):
            logger.error(f"[{a}] 下发 HOME 关节角失败")
            return False
        logger.info(f"[{a}] 运动到准备位 ...")

    deadline = time.time() + 60.0
    while time.time() < deadline:
        data = robot.subscribe(dcss)
        arrived = True
        for a in arms:
            fb = data['outputs'][_ARM_IDX[a]].get('fb_joint_pos')
            if not fb or any(abs(c - t) >= cfg.REACH_TOL
                             for c, t in zip(fb, home_map[a])):
                arrived = False
                break
        if arrived:
            break
        time.sleep(0.05)
    else:
        logger.warning("go_home 超时（60s），继续执行")

    logger.info(f"已到准备位: {arms}")
    return True


# ═══════════════════════════════════════════════════════════════════════════════
# SingleArm — 单臂公开接口
# ═══════════════════════════════════════════════════════════════════════════════

class SingleArm:
    """
    单臂封装，自持 SDK 连接，对外使用 base_link 坐标系（米/弧度）。

    用法：
        arm = SingleArm('left')   # 或 'right'
        arm.connect()

        joints          = arm.read_joints()          # [j1..j7]，度
        pos_m, rpy_rad  = arm.get_ee_state_base()    # base_link 系
        arm.move_to_ee_base([x,y,z], [r,p,y])        # base_link 系目标

        arm.go_home()
        arm.release()
    """

    def __init__(self, arm: str):
        """
        Args:
            arm: 'left' 或 'right'
        """
        assert arm in ('left', 'right'), \
            f"arm 须为 'left' 或 'right'，实为 {arm!r}"
        self.arm_str   = arm
        self._arm_cmd  = _ARM_CMD[arm]
        self._robot    = None
        self._dcss     = None
        self._core: Optional[_ArmCore] = None
        self._connected = False

    # ── 连接 ──────────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """连接机械臂，初始化运动学。"""
        cfg = _cfg()
        CMR, DCSS_cls, MK, FIKP = _load_sdk()
        self._robot = CMR()
        self._dcss  = DCSS_cls()

        ini = _do_connect(self._robot, self._dcss, cfg)
        if ini is None:
            return False

        self._core = _ArmCore(self.arm_str, self._robot, self._dcss, ini, MK, FIKP)

        # 读取初始关节角，供 IK 参考
        data = self._robot.subscribe(self._dcss)
        self._core.read_joints_from(data)

        self._connected = True
        logger.info(f"[{self.arm_str}] 单臂就绪")
        return True

    # ── 运动控制 ──────────────────────────────────────────────────────────────

    def go_home(self) -> bool:
        """运动到准备位（位置模式，阻塞等待到位）。"""
        self._assert_connected()
        return _go_home_arms(self._robot, self._dcss, [self.arm_str], _cfg())

    def release(self):
        """关闭使能并释放连接。"""
        if self._connected:
            try:
                self._robot.disable(arm=self._arm_cmd)
            except Exception:
                pass
            time.sleep(0.3)
            self._robot.release_robot()
            self._connected = False
            logger.info(f"[{self.arm_str}] 连接已释放")

    # ── 状态读取（base_link 坐标系）──────────────────────────────────────────

    def read_joints(self) -> list:
        """当前关节角，度，[j1..j7]。"""
        self._assert_connected()
        data = self._robot.subscribe(self._dcss)
        return self._core.read_joints_from(data)

    def get_fk_raw(self) -> Optional[list]:
        """FK 输出，臂基坐标系：[X_mm,Y_mm,Z_mm,A_deg,B_deg,C_deg]，失败返回 None。"""
        self._assert_connected()
        data = self._robot.subscribe(self._dcss)
        joints = self._core.read_joints_from(data)
        return self._core.get_fk_xyzabc(joints)

    def get_ee_state_base(self) -> tuple:
        """
        末端位姿，base_link 坐标系。

        Returns:
            (pos_m: np.ndarray[3], rpy_rad: tuple[3])
            FK 失败时返回 (None, None)
        """
        fk = self.get_fk_raw()
        if fk is None:
            return (None, None)
        return fk_to_base(fk, self.arm_str)

    # ── 动作下发（base_link 坐标系）──────────────────────────────────────────

    def move_to_ee_base(self,
                         pos_m:   list,
                         rpy_rad: list,
                         safe:    bool = True) -> bool:
        """
        下发末端目标位姿（base_link 坐标系，非阻塞）。

        流程：base_link → frame_transform → 臂基 xyzabc → IK → 关节角指令

        Args:
            pos_m:   目标位置 [x,y,z]，米
            rpy_rad: 目标姿态 [roll,pitch,yaw]，弧度
            safe:    True = IK 结果须通过软限位检查

        Returns:
            True = 指令已下发；False = IK 无解或安全检查失败
        """
        self._assert_connected()
        xyzabc = base_to_fk(pos_m, rpy_rad, self.arm_str)
        return self._send_xyzabc(xyzabc, safe)

    # ── 验证工具 ──────────────────────────────────────────────────────────────

    def verify_transform(self):
        """打印当前 EE 状态（臂基坐标系 + base_link 坐标系），供上机验证。"""
        fk = self.get_fk_raw()
        if fk is None:
            print(f"[{self.arm_str}] FK 失败")
            return
        pos_m, rpy_rad = fk_to_base(fk, self.arm_str)
        print(f"\n[{self.arm_str}臂] FK 输出 (臂坐标系):")
        print(f"  XYZ=[{fk[0]:.1f}, {fk[1]:.1f}, {fk[2]:.1f}]mm  "
              f"ABC=[{fk[3]:.1f}, {fk[4]:.1f}, {fk[5]:.1f}]°")
        print(f"[{self.arm_str}臂] 变换后 (base_link 系):")
        if pos_m is not None:
            print(f"  pos=[{pos_m[0]:.4f}, {pos_m[1]:.4f}, {pos_m[2]:.4f}]m  "
                  f"rpy=[{math.degrees(rpy_rad[0]):.1f}, "
                  f"{math.degrees(rpy_rad[1]):.1f}, "
                  f"{math.degrees(rpy_rad[2]):.1f}]°")

    # ── 阻塞式接口 ────────────────────────────────────────────────────────────

    def move_joints_sync(self, joints: list, timeout: float = 30.0) -> bool:
        """
        切换位置模式 → 下发关节角 → 阻塞等待到位。

        Args:
            joints:  目标关节角，度，长度 7
            timeout: 等待超时，秒
        """
        self._assert_connected()
        cfg = _cfg()
        if not self._robot.set_position_state(
            arm=self._arm_cmd,
            velRatio=cfg.VEL_RATIO,
            AccRatio=cfg.ACC_RATIO,
        ):
            logger.error(f"[{self.arm_str}] 切换位置模式失败")
            return False
        time.sleep(0.3)
        if not self._robot.set_joint_position_cmd(arm=self._arm_cmd, joint=joints):
            logger.error(f"[{self.arm_str}] 下发关节角失败")
            return False
        return _wait_joints_reached(
            self._robot, self._dcss, self.arm_str, joints, cfg.REACH_TOL, timeout
        )

    def move_to_ee_base_sync(self,
                              pos_m:   list,
                              rpy_rad: list,
                              timeout: float = 30.0) -> bool:
        """
        base_link 目标位姿 → IK → 阻塞等待到位。

        Args:
            pos_m:   目标位置 [x,y,z]，米
            rpy_rad: 目标姿态 [roll,pitch,yaw]，弧度
            timeout: 等待超时，秒
        """
        self._assert_connected()
        xyzabc = base_to_fk(pos_m, rpy_rad, self.arm_str)
        joints = self._core.solve_ik(xyzabc)
        if joints is None:
            return False
        if not check_joints_in_limits(joints):
            logger.warning(f"[{self.arm_str}] IK 结果超出软限位")
            return False
        return self.move_joints_sync(joints, timeout)

    # ── 内部辅助 ──────────────────────────────────────────────────────────────

    def _send_xyzabc(self, xyzabc_mm_deg: list, safe: bool = True) -> bool:
        """臂基坐标系 xyzabc → IK → 下发（内部接口，供 arm.py TJArm 使用）。"""
        joints = self._core.solve_ik(xyzabc_mm_deg)
        if joints is None:
            return False
        if safe and not check_joints_in_limits(joints):
            logger.warning(f"[{self.arm_str}] IK 结果超出软限位，指令已拦截")
            return False
        return self._core.send_joints(joints)

    def _assert_connected(self):
        if not self._connected:
            raise RuntimeError(f"SingleArm[{self.arm_str}] 尚未连接，请先调用 connect()")


# ═══════════════════════════════════════════════════════════════════════════════
# DualArm — 双臂公开接口
# ═══════════════════════════════════════════════════════════════════════════════

class DualArm:
    """
    双臂封装，共享一个 SDK 连接，对外使用 base_link 坐标系（米/弧度）。

    用法：
        da = DualArm()
        da.connect()

        joints = da.read_joints()
        # joints['left'] = [j1..j7]，度

        states = da.get_ee_states_base()
        # states['left'] = (pos_m, rpy_rad)

        da.move_to_ee_base('left', [x,y,z], [r,p,y])
        da.move_both_ee_base(l_pos, l_rpy, r_pos, r_rpy)

        da.go_home()          # 双臂同时
        da.go_home('right')   # 仅右臂
        da.release()
    """

    def __init__(self):
        self._robot  = None
        self._dcss   = None
        self._cores: dict[str, _ArmCore] = {}   # {'left': _ArmCore, 'right': _ArmCore}
        self._connected = False
        self._wrench_source = None

    # ── 连接 ──────────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """连接机械臂（共享连接），初始化双臂运动学。"""
        cfg = _cfg()
        CMR, DCSS_cls, MK, FIKP = _load_sdk()
        self._robot = CMR()
        self._dcss  = DCSS_cls()

        ini = _do_connect(self._robot, self._dcss, cfg)
        if ini is None:
            return False

        self._cores = {
            'left':  _ArmCore('left',  self._robot, self._dcss, ini, MK, FIKP),
            'right': _ArmCore('right', self._robot, self._dcss, ini, MK, FIKP),
        }

        # 读取初始关节角
        data = self._robot.subscribe(self._dcss)
        for core in self._cores.values():
            core.read_joints_from(data)

        # ── 控制模式预热 ──────────────────────────────────────────────────────
        # 实测：connect() 后首次模式设置返回 True，但控制器切换是异步的，
        # 若立即下发运动指令会被静默丢弃（现象：咔哒一声，不动）。
        # 预热后 sleep 1s 确保控制器完成模式切换。
        for arm_str in ('left', 'right'):
            cmd = _ARM_CMD[arm_str]
            if cfg.CTRL_MODE == 'impedance':
                ok = self._robot.set_imp_cart_state(
                    arm=cmd,
                    velRatio=cfg.VEL_RATIO,
                    AccRatio=cfg.ACC_RATIO,
                    K=cfg.IMP_K,
                    D=cfg.IMP_D,
                    rot_type=0,          # 不定义末端旋转方向
                    cart_ctrl_para=[0.0] * 7,
                )
                logger.info(f"[{arm_str}] 笛卡尔阻抗模式  K={cfg.IMP_K}  D={cfg.IMP_D}")
            else:
                ok = self._robot.set_position_state(
                    arm=cmd,
                    velRatio=cfg.VEL_RATIO,
                    AccRatio=cfg.ACC_RATIO,
                )
                logger.info(f"[{arm_str}] 位置跟随模式")
            if not ok:
                logger.warning(f"[{arm_str}] 控制模式设置失败（返回 False）")

        time.sleep(1.0)   # 等待双臂控制器完成模式切换
        logger.info(f"控制模式预热完成 ({cfg.CTRL_MODE})")

        self._connected = True
        self._wrench_source = SDKWrenchSource(self._robot, self._dcss)
        logger.info("双臂就绪 (left='A'/0, right='B'/1)")
        return True

    # ── 运动控制 ──────────────────────────────────────────────────────────────

    def go_home(self, arm: str = 'both') -> bool:
        """运动到准备位（位置模式，阻塞等待到位）。arm: 'left'|'right'|'both'"""
        self._assert_connected()
        arms = ['left', 'right'] if arm == 'both' else [arm]
        return _go_home_arms(self._robot, self._dcss, arms, _cfg())

    def enter_position_mode(self, arm: str = 'both') -> bool:
        """
        显式切到位置跟随模式（带 sleep 等切换）。

        关键步骤（与 SDK 文档对齐）：
          1. clear_error(arm)        清除之前实验残留的错误状态
          2. set_state(arm, 1)       arm 状态机切到"位置跟随"（state=1）
          3. set_position_state(...) 设置 vel/acc 比例
          4. sleep(1.0)              等异步切换完成

        go_home 完成后 SDK 的 arm_state 可能回到 0（下伺服），此时
        set_joint_position_cmd 会被静默拒（返回 False），机械臂全程不动。
        本函数确保进入 20Hz 控制循环前 arm_state 处于位置跟随，避免该问题。
        """
        self._assert_connected()
        arms = ['left', 'right'] if arm == 'both' else [arm]
        cfg = _cfg()

        def _set_pos_all() -> bool:
            for a in arms:
                ok = self._robot.set_position_state(
                    arm=_ARM_CMD[a],
                    velRatio=cfg.VEL_RATIO,
                    AccRatio=cfg.ACC_RATIO,
                )
                if not ok:
                    logger.warning(f"[{a}] set_position_state 返回 False")
                    return False
            return True

        # 先不做 servo_reset 直接尝试（已在位置模式 / go_home 后均可直接成功）
        if _set_pos_all():
            time.sleep(1.0)
            logger.info(f"已切到位置跟随模式: {arms}")
            return True

        # SDK 检查双臂状态：任一臂有问题都会导致失败。
        # 对所有目标臂统一 servo_reset 后再重试，避免只 reset 一臂、另一臂仍报错。
        logger.info("首次 set_position_state 失败，servo_reset 全部目标臂后重试")
        for a in arms:
            cmd = _ARM_CMD[a]
            for axis in range(7):
                try:
                    self._robot.servo_reset(arm=cmd, axis=axis)
                except Exception as e:
                    logger.warning(f"[{a}] servo_reset axis={axis} 异常（继续）: {e}")

        if not _set_pos_all():
            logger.error(f"servo_reset 后 set_position_state 仍失败，检查臂硬件状态")
            return False

        time.sleep(1.0)
        logger.info(f"已切到位置跟随模式: {arms}")
        return True

    def release(self):
        """关闭使能并释放连接。"""
        if self._connected:
            for cmd in _ARM_CMD.values():
                try:
                    self._robot.disable(arm=cmd)
                except Exception:
                    pass
            time.sleep(0.3)
            self._robot.release_robot()
            if self._wrench_source is not None:
                self._wrench_source.close()
                self._wrench_source = None
            self._connected = False
            logger.info("双臂连接已释放")

    # ── 状态读取（base_link 坐标系）──────────────────────────────────────────

    def read_joints(self) -> dict:
        """
        双臂关节角（单次 subscribe，两臂共享同一帧）。

        Returns:
            {'left': [j1..j7], 'right': [j1..j7]}，度
        """
        self._assert_connected()
        data = self._robot.subscribe(self._dcss)
        return {a: c.read_joints_from(data) for a, c in self._cores.items()}

    def get_fk_raw(self) -> dict:
        """
        双臂 FK 输出，臂基坐标系（调试用）。

        Returns:
            {'left': [X_mm,Y_mm,Z_mm,A_deg,B_deg,C_deg] 或 None, 'right': ...}
        """
        self._assert_connected()
        data = self._robot.subscribe(self._dcss)
        return {
            a: c.get_fk_xyzabc(c.read_joints_from(data))
            for a, c in self._cores.items()
        }

    def get_ee_states_base(self) -> dict:
        """
        双臂末端位姿，base_link 坐标系（单次 subscribe）。

        Returns:
            {
              'left':  (pos_m: np.ndarray[3], rpy_rad: tuple[3]),
              'right': (pos_m: np.ndarray[3], rpy_rad: tuple[3]),
            }
            FK 失败时对应臂为 (None, None)。
        """
        self._assert_connected()
        data = self._robot.subscribe(self._dcss)
        result = {}
        for arm_str, core in self._cores.items():
            joints = core.read_joints_from(data)
            fk = core.get_fk_xyzabc(joints)
            if fk is None:
                logger.warning(f"[{arm_str}] FK 失败")
                result[arm_str] = (None, None)
            else:
                result[arm_str] = fk_to_base(fk, arm_str)
        return result

    def read_all_states(self) -> dict:
        """
        单次 subscribe 同时返回双臂关节角 + base_link EE 状态。

        单次网络往返，避免 read_joints() + get_ee_states_base() 的双次 subscribe。

        Returns:
            {
              'joints': {'left': [j1..j7（度）], 'right': [j1..j7（度）]},
              'ee': {
                'left':  (pos_m: np.ndarray[3], rpy_rad: tuple[3]),
                'right': (pos_m: np.ndarray[3], rpy_rad: tuple[3]),
              },
            }
            FK 失败时对应臂 ee 为 (None, None)。
        """
        self._assert_connected()
        data = self._robot.subscribe(self._dcss)
        joints_dict = {}
        ee_dict = {}
        for arm_str, core in self._cores.items():
            j  = core.read_joints_from(data)
            fk = core.get_fk_xyzabc(j)
            joints_dict[arm_str] = j
            if fk is None:
                logger.warning(f"[{arm_str}] FK 失败")
                ee_dict[arm_str] = (None, None)
            else:
                ee_dict[arm_str] = fk_to_base(fk, arm_str)
        wrench_dict = {}
        if self._wrench_source is not None:
            for arm_str in ('left', 'right'):
                wrench_dict[arm_str] = self._wrench_source.read_from_data(data, arm_str)
        return {'joints': joints_dict, 'ee': ee_dict, 'wrench': wrench_dict}

    # ── 动作下发（base_link 坐标系）──────────────────────────────────────────

    def move_to_ee_base(self,
                         arm:     str,
                         pos_m:   list,
                         rpy_rad: list,
                         safe:    bool = True) -> bool:
        """
        单臂末端目标位姿（base_link 坐标系，非阻塞）。

        Args:
            arm:     'left' 或 'right'
            pos_m:   目标位置 [x,y,z]，米
            rpy_rad: 目标姿态 [roll,pitch,yaw]，弧度
            safe:    True = IK 结果须通过软限位检查

        Returns:
            True = 指令已下发；False = IK 无解或安全检查失败
        """
        self._assert_connected()
        if not check_workspace(pos_m):
            logger.warning(f"[{arm}] 目标位于禁区，指令已拦截 "
                           f"pos=[{pos_m[0]:.3f},{pos_m[1]:.3f},{pos_m[2]:.3f}]m")
            return False
        core = self._cores[arm]
        xyzabc = base_to_fk(pos_m, rpy_rad, arm)
        joints = core.solve_ik(xyzabc)
        if joints is None:
            return False
        if safe and not check_joints_in_limits(joints):
            logger.warning(f"[{arm}] IK 结果超出软限位，指令已拦截")
            return False
        return core.send_joints(joints)

    def move_both_ee_base(self,
                           left_pos:  list,
                           left_rpy:  list,
                           right_pos: list,
                           right_rpy: list,
                           safe:      bool = True) -> tuple:
        """
        同时下发双臂目标位姿（base_link 坐标系，非阻塞）。

        Returns:
            (left_ok: bool, right_ok: bool)
        """
        l_ok = self.move_to_ee_base('left',  left_pos,  left_rpy,  safe)
        r_ok = self.move_to_ee_base('right', right_pos, right_rpy, safe)
        return l_ok, r_ok

    def last_ik_joints(self, arm: str) -> Optional[list]:
        """返回指定臂上一次 IK 解算的关节角（度），失败则为 None。"""
        return self._cores[arm]._last_ik_joints

    # ── 验证工具 ──────────────────────────────────────────────────────────────

    def verify_transforms(self):
        """打印双臂 EE 状态（臂基 + base_link 坐标系），供上机后验证。"""
        self._assert_connected()
        raw    = self.get_fk_raw()
        states = self.get_ee_states_base()
        for arm_str in ('left', 'right'):
            fk = raw[arm_str]
            pos_m, rpy_rad = states[arm_str]
            print(f"\n[{arm_str}臂] FK 输出 (臂坐标系):")
            if fk:
                print(f"  XYZ=[{fk[0]:.1f},{fk[1]:.1f},{fk[2]:.1f}]mm  "
                      f"ABC=[{fk[3]:.1f},{fk[4]:.1f},{fk[5]:.1f}]°")
            else:
                print("  FK 失败")
            print(f"[{arm_str}臂] 变换后 (base_link 系):")
            if pos_m is not None:
                print(f"  pos=[{pos_m[0]:.4f},{pos_m[1]:.4f},{pos_m[2]:.4f}]m  "
                      f"rpy=[{math.degrees(rpy_rad[0]):.1f},"
                      f"{math.degrees(rpy_rad[1]):.1f},"
                      f"{math.degrees(rpy_rad[2]):.1f}]°")
            else:
                print("  变换失败")

    def move_joints(self, arm: str, joints: list, safe: bool = True) -> bool:
        """
        单臂关节角指令（非阻塞）。指令下发后立即返回，控制器自行插值到目标。

        Args:
            arm:    'left' 或 'right'
            joints: 目标关节角，度，长度 7
            safe:   True = 先做软限位检查，超限则拦截

        Returns:
            True = 指令已下发；False = 软限位拦截或 SDK 调用失败
        """
        self._assert_connected()
        if safe and not check_joints_in_limits(joints):
            logger.warning(
                f"[{arm}] 关节角超软限位，指令已拦截 "
                f"joints=[{', '.join(f'{v:.1f}' for v in joints)}]°"
            )
            return False
        return self._cores[arm].send_joints(joints)

    def move_joints_both(self,
                          joints_left:  list,
                          joints_right: list,
                          safe: bool = True) -> tuple:
        """
        双臂同时下发关节角指令（非阻塞）。

        Returns:
            (left_ok: bool, right_ok: bool)
        """
        l_ok = self.move_joints('left',  joints_left,  safe)
        r_ok = self.move_joints('right', joints_right, safe)
        return l_ok, r_ok

    # ── 阻塞式接口 ────────────────────────────────────────────────────────────

    def move_joints_sync(self, arm: str, joints: list,
                          timeout: float = 30.0) -> bool:
        """
        切换位置模式 → 下发关节角 → 阻塞等待到位（单臂）。

        Args:
            arm:     'left' 或 'right'
            joints:  目标关节角，度，长度 7
            timeout: 等待超时，秒
        """
        self._assert_connected()
        cfg = _cfg()
        cmd = _ARM_CMD[arm]
        logger.info(f"[{arm}] set_position_state arm={cmd!r} "
                    f"vel={cfg.VEL_RATIO} acc={cfg.ACC_RATIO}")
        ok_mode = self._robot.set_position_state(
            arm=cmd,
            velRatio=cfg.VEL_RATIO,
            AccRatio=cfg.ACC_RATIO,
        )
        logger.info(f"[{arm}] set_position_state → {ok_mode}")
        if not ok_mode:
            logger.error(f"[{arm}] 切换位置模式失败")
            return False
        time.sleep(0.3)
        logger.info(f"[{arm}] set_joint_position_cmd arm={cmd!r} "
                    f"joints=[{', '.join(f'{v:.1f}' for v in joints)}]°")
        ok_cmd = self._robot.set_joint_position_cmd(arm=cmd, joint=joints)
        logger.info(f"[{arm}] set_joint_position_cmd → {ok_cmd}")
        if not ok_cmd:
            logger.error(f"[{arm}] 下发关节角失败")
            return False
        return _wait_joints_reached(
            self._robot, self._dcss, arm, joints, cfg.REACH_TOL, timeout
        )

    def move_joints_both_sync(self,
                               joints_left:  list,
                               joints_right: list,
                               timeout: float = 30.0) -> tuple:
        """
        双臂同时运动到指定关节角，阻塞等待双臂均到位。

        先同时切换位置模式并下发指令，再同时等待。

        Returns:
            (left_ok: bool, right_ok: bool)
        """
        self._assert_connected()
        cfg = _cfg()

        # 切换两臂位置模式
        for arm in ('left', 'right'):
            if not self._robot.set_position_state(
                arm=_ARM_CMD[arm],
                velRatio=cfg.VEL_RATIO,
                AccRatio=cfg.ACC_RATIO,
            ):
                logger.error(f"[{arm}] 切换位置模式失败")
                return False, False
        time.sleep(0.3)

        # 同时下发
        ok_l = self._robot.set_joint_position_cmd(arm='A', joint=joints_left)
        ok_r = self._robot.set_joint_position_cmd(arm='B', joint=joints_right)
        if not ok_l:
            logger.error("[left] 下发关节角失败")
        if not ok_r:
            logger.error("[right] 下发关节角失败")
        if not (ok_l and ok_r):
            return ok_l, ok_r

        # 同时等待
        tol      = cfg.REACH_TOL
        deadline = time.time() + timeout
        done_l = done_r = False
        while time.time() < deadline:
            data = self._robot.subscribe(self._dcss)
            if not done_l:
                fb = data['outputs'][0].get('fb_joint_pos')
                done_l = bool(fb and all(
                    abs(c - t) < tol for c, t in zip(fb, joints_left)
                ))
            if not done_r:
                fb = data['outputs'][1].get('fb_joint_pos')
                done_r = bool(fb and all(
                    abs(c - t) < tol for c, t in zip(fb, joints_right)
                ))
            if done_l and done_r:
                break
            time.sleep(0.05)

        if not (done_l and done_r):
            logger.warning(
                f"move_joints_both_sync 超时: "
                f"left={'✓' if done_l else '✗'}  right={'✓' if done_r else '✗'}"
            )
        return done_l, done_r

    def move_to_ee_base_sync(self,
                              arm:     str,
                              pos_m:   list,
                              rpy_rad: list,
                              timeout: float = 30.0) -> bool:
        """
        base_link 目标位姿 → IK → 阻塞等待到位（单臂）。

        Args:
            arm:     'left' 或 'right'
            pos_m:   目标位置 [x,y,z]，米
            rpy_rad: 目标姿态 [roll,pitch,yaw]，弧度
            timeout: 等待超时，秒
        """
        self._assert_connected()
        core = self._cores[arm]
        xyzabc = base_to_fk(pos_m, rpy_rad, arm)
        joints = core.solve_ik(xyzabc)
        if joints is None:
            return False
        if not check_joints_in_limits(joints):
            logger.warning(f"[{arm}] IK 结果超出软限位")
            return False
        logger.info(f"[{arm}] IK解: [{', '.join(f'{v:.1f}' for v in joints)}]°")
        return self.move_joints_sync(arm, joints, timeout)

    def read_wrench(self, arm: str) -> list:
        """六维力数据：[Fx, Fy, Fz, Mx, My, Mz]，单位 N / N·m。"""
        self._assert_connected()
        if self._wrench_source is None:
            return [0.0] * 6
        return self._wrench_source.read(arm)

    # ── 内部辅助 ──────────────────────────────────────────────────────────────

    def _assert_connected(self):
        if not self._connected:
            raise RuntimeError("DualArm 尚未连接，请先调用 connect()")


# ═══════════════════════════════════════════════════════════════════════════════
# MockDualArm — 无硬件调试（接口与 DualArm 完全一致）
# ═══════════════════════════════════════════════════════════════════════════════

class MockDualArm:
    """
    模拟双臂，无需任何硬件连接。
    接口与 DualArm 完全一致，所有运动指令直接返回 True。
    用于在无机械臂连接时测试 demo / runner 的控制逻辑。
    """

    def __init__(self):
        self._joints = {
            'left':  [0.0] * 7,
            'right': [0.0] * 7,
        }
        self._wrench_source = MockWrenchSource()
        logger.info('[MockDualArm] 模拟双臂已启动（无硬件）')

    # ── 连接管理 ──────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        logger.info('[MockDualArm] connect() → True（无硬件）')
        return True

    def release(self):
        logger.info('[MockDualArm] release() → 无操作')

    def go_home(self, arm: str = 'both'):
        logger.info(f'[MockDualArm] go_home({arm!r}) → 无操作')

    def enter_position_mode(self, arm: str = 'both') -> bool:
        logger.info(f'[MockDualArm] enter_position_mode({arm!r}) → True（无操作）')
        return True

    # ── 状态读取 ──────────────────────────────────────────────────────────────

    def read_joints(self) -> dict:
        """返回当前模拟关节角（度），{'left': [...], 'right': [...]}。"""
        return {k: list(v) for k, v in self._joints.items()}

    def get_fk_raw(self) -> dict:
        """返回 {'left': None, 'right': None}（无 FK 计算）。"""
        return {'left': None, 'right': None}

    def get_ee_states_base(self) -> dict:
        """
        返回 {'left': (None, None), 'right': (None, None)}。
        pos/rpy 均为 None，调用方须做 None 检查。
        """
        return {'left': (None, None), 'right': (None, None)}

    def read_all_states(self) -> dict:
        """MockDualArm 版：关节角返回当前模拟值，EE 均为 None。"""
        return {'joints': self.read_joints(),
                'ee': {'left': (None, None), 'right': (None, None)},
                'wrench': {'left': [0.0]*6, 'right': [0.0]*6}}

    def read_wrench(self, arm: str) -> list:
        return [0.0] * 6

    # ── 运动控制 ──────────────────────────────────────────────────────────────

    def move_to_ee_base(self, arm: str,
                        pos_m:   list,
                        rpy_rad: list,
                        safe:    bool = True) -> bool:
        logger.debug(f'[MockDualArm] move_to_ee_base({arm}) pos={pos_m}')
        time.sleep(0.05)
        return True

    def move_both_ee_base(self,
                          l_pos_m:   list, l_rpy_rad: list,
                          r_pos_m:   list, r_rpy_rad: list,
                          safe:      bool = True) -> tuple:
        logger.debug('[MockDualArm] move_both_ee_base()')
        time.sleep(0.05)
        return True, True

    def move_joints(self, arm: str, joints: list, safe: bool = True) -> bool:
        """非阻塞关节指令（Mock）：直接更新模拟关节角并返回 True。"""
        self._joints[arm] = list(joints)
        return True

    def move_joints_both(self,
                         joints_left:  list,
                         joints_right: list,
                         safe: bool = True) -> tuple:
        self._joints['left']  = list(joints_left)
        self._joints['right'] = list(joints_right)
        return True, True

    def move_joints_sync(self,
                         arm:     str,
                         joints:  list,
                         timeout: float = 30.0) -> bool:
        logger.info(f'[MockDualArm] move_joints_sync({arm}) '
                    f'joints=[{", ".join(f"{v:.1f}" for v in joints)}]°')
        self._joints[arm] = list(joints)
        time.sleep(0.2)   # 模拟运动延迟
        return True

    def move_joints_both_sync(self,
                               joints_left:  list,
                               joints_right: list,
                               timeout:      float = 30.0) -> tuple:
        logger.info('[MockDualArm] move_joints_both_sync()')
        self._joints['left']  = list(joints_left)
        self._joints['right'] = list(joints_right)
        time.sleep(0.2)
        return True, True

    def move_to_ee_base_sync(self,
                              arm:     str,
                              pos_m:   list,
                              rpy_rad: list,
                              timeout: float = 30.0) -> bool:
        logger.info(f'[MockDualArm] move_to_ee_base_sync({arm}) '
                    f'pos={[f"{v:.3f}" for v in pos_m]}')
        time.sleep(0.2)
        return True

    # ── 调试辅助 ──────────────────────────────────────────────────────────────

    def verify_transforms(self):
        logger.info('[MockDualArm] verify_transforms() → 无操作（Mock 模式）')
