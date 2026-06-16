"""
gripper.py — Xense 夹爪封装

公开接口：
  XenseGripper(mac, name, vmax, fmax, tol)   真实硬件
  MockGripper(name)                           无硬件调试

量程：
  Xense TCP 位置：0 mm（完全闭合）~ 85 mm（完全张开）
  机械止点约 1.7 mm，POS_CLOSE 设为 2 mm 防止过流保护触发
  URDF 关节量程换算见 gripper_utils.py

注意：
  xensegripper 库依赖 Qt，需设置环境变量 QT_API=pyside6。
  本模块在 import 时自动设置，也可在启动脚本中预先设置。
"""

import os
import time
import logging

# 必须在 import xensegripper 之前设置
os.environ.setdefault('QT_API', 'pyside6')

logger = logging.getLogger(__name__)

# ── 量程常数（与 gripper_utils.py 保持一致）───────────────────────────────────
POS_OPEN  = 85.0   # mm，完全张开
POS_CLOSE  = 2.0   # mm，完全闭合（留 2mm 避免顶死过流）
VMAX_MIN  = 40.0   # mm/s，SDK 允许的最小速度


# ═══════════════════════════════════════════════════════════════════════════════
# XenseGripper — 真实硬件
# ═══════════════════════════════════════════════════════════════════════════════

class XenseGripper:
    """
    Xense TCP 夹爪封装。

    用法：
        g = XenseGripper(mac="3ad820773a85", name='left')
        g.connect()
        g.open()
        g.close()
        print(g.get_position())   # mm
        g.disconnect()
    """

    def __init__(self, mac: str,
                 name:  str   = '',
                 vmax:  float = 80.0,
                 fmax:  float = 27.0,
                 tol:   float = 2.0):
        """
        Args:
            mac:  夹爪 MAC 地址（无冒号小写，如 "3ad820773a85"）
            name: 标识符，用于日志（如 'left' / 'right'）
            vmax: 最大速度，mm/s（最小 40）
            fmax: 最大力，N
            tol:  到位容差，mm
        """
        self._mac   = mac
        self._name  = name or mac[-6:]
        self._vmax  = max(vmax, VMAX_MIN)
        self._fmax  = fmax
        self._tol   = tol
        self._sdk   = None   # xensegripper 实例（connect 后创建）
        self._last_cmd = -1.0  # 上次下发目标(mm)：仅用于"目标变化才打日志"，避免 20Hz 循环刷屏

    # ── 连接管理 ──────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """初始化并连接夹爪。"""
        try:
            from xensegripper import XenseGripper as _SDK
            # SDK 推荐使用工厂方法 create() 而非直接构造
            self._sdk = _SDK.create(mac_addr=self._mac)
            if self._sdk is None:
                logger.error(
                    f"[gripper/{self._name}] 连接失败：_SDK.create() 返回 None"
                    f"（MAC={self._mac}，检查夹爪是否开机/IP可达）"
                )
                return False
            logger.info(f"[gripper/{self._name}] 已连接 MAC={self._mac}，"
                        f"初始位置 {self.get_position():.1f}mm")
            return True
        except Exception as e:
            logger.error(f"[gripper/{self._name}] 连接失败: {e}")
            return False

    def disconnect(self):
        """断开夹爪连接。"""
        self._sdk = None
        logger.info(f"[gripper/{self._name}] 已断开")

    # ── 运动控制 ──────────────────────────────────────────────────────────────

    def open(self, blocking: bool = True, timeout: float = 10.0) -> bool:
        """夹爪张开到最大位置（85 mm）。"""
        return self.set_position(POS_OPEN, blocking=blocking, timeout=timeout)

    def close(self, blocking: bool = True, timeout: float = 10.0) -> bool:
        """夹爪闭合到最小位置（2 mm）。"""
        return self.set_position(POS_CLOSE, blocking=blocking, timeout=timeout)

    def set_position(self, pos_mm: float,
                     blocking: bool = True,
                     timeout:  float = 10.0) -> bool:
        """
        运动到指定位置。

        Args:
            pos_mm:   目标位置，mm，[POS_CLOSE, POS_OPEN]
            blocking: True = 阻塞等待到位
            timeout:  阻塞超时，秒

        Returns:
            True = 成功（非阻塞时始终返回 True）
        """
        self._assert_connected()
        pos_mm = float(max(POS_CLOSE, min(POS_OPEN, pos_mm)))

        # 仅在目标变化(>0.5mm)或阻塞调用时打 INFO——避免 20Hz 控制循环里每拍刷屏，
        # 但夹爪每次"开↔合"切换都会留一行，便于真机排查"夹爪到底有没有收到指令/动没动"。
        if blocking or abs(pos_mm - self._last_cmd) > 0.5:
            logger.info(f"[gripper/{self._name}] 下发目标 {pos_mm:.1f}mm "
                        f"(当前 {self.get_position():.1f}mm, vmax={self._vmax:.0f} "
                        f"fmax={self._fmax:.0f}, blocking={blocking})")
        self._last_cmd = pos_mm

        try:
            # set_position 发送运动指令（非阻塞）
            # SDK 签名：set_position(position, vmax, fmax)，无 tol 参数
            # 到位判断由 _wait_position 轮询实现
            self._sdk.set_position(pos_mm, self._vmax, self._fmax)
        except Exception as e:
            logger.error(f"[gripper/{self._name}] set_position 失败: {e}")
            return False

        if blocking:
            ok = self._wait_position(pos_mm, timeout)
            st = self.get_status()
            logger.info(f"[gripper/{self._name}] 到位={ok} 实际 {st.get('position', -1.0):.1f}mm "
                        f"力={st.get('force', float('nan')):.1f}N "
                        f"温度={st.get('temperature', float('nan')):.0f}℃")
            return ok
        return True

    # ── 状态读取 ──────────────────────────────────────────────────────────────

    def get_status(self) -> dict:
        """
        读取完整状态字典：position(mm)/velocity(mm/s)/force(N)/temperature(℃) 等。
        失败返回 {}（调用方按需做缺键/空字典处理）。
        """
        self._assert_connected()
        try:
            status = self._sdk.get_gripper_status()
            return dict(status) if isinstance(status, dict) else {}
        except Exception as e:
            logger.error(f"[gripper/{self._name}] get_status 失败: {e}")
            return {}

    def get_position(self) -> float:
        """
        读取当前位置，mm。失败返回 -1.0。
        """
        return float(self.get_status().get('position', -1.0))

    # ── 内部辅助 ──────────────────────────────────────────────────────────────

    def _wait_position(self, target_mm: float, timeout: float) -> bool:
        """轮询直到位置在容差内，超时返回 False。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            pos = self.get_position()
            if pos >= 0 and abs(pos - target_mm) <= self._tol:
                return True
            time.sleep(0.05)
        pos = self.get_position()
        logger.warning(f"[gripper/{self._name}] 未到位：目标={target_mm:.1f}mm "
                        f"当前={pos:.1f}mm 超时={timeout:.0f}s")
        return False

    def _assert_connected(self):
        if self._sdk is None:
            raise RuntimeError(
                f"XenseGripper[{self._name}] 未连接，请先调用 connect()"
            )


# ═══════════════════════════════════════════════════════════════════════════════
# MockGripper — 无硬件调试
# ═══════════════════════════════════════════════════════════════════════════════

class MockGripper:
    """
    模拟夹爪，接口与 XenseGripper 完全一致，不需要任何硬件。
    用于在 Jetson 未连接夹爪时测试其余系统逻辑。
    """

    def __init__(self, name: str = '', **kwargs):
        self._name = name or 'mock'
        self._pos  = POS_OPEN   # 初始位置：张开
        self._last_cmd = -1.0
        logger.info(f"[gripper/{self._name}] MockGripper 已启动（无硬件）")

    def connect(self)    -> bool:  return True
    def disconnect(self) -> None:  pass

    def open(self, blocking: bool = True, timeout: float = 10.0) -> bool:
        return self.set_position(POS_OPEN, blocking=blocking, timeout=timeout)

    def close(self, blocking: bool = True, timeout: float = 10.0) -> bool:
        return self.set_position(POS_CLOSE, blocking=blocking, timeout=timeout)

    def set_position(self, pos_mm: float,
                     blocking: bool = True,
                     timeout:  float = 10.0) -> bool:
        self._pos = float(max(POS_CLOSE, min(POS_OPEN, pos_mm)))
        if blocking or abs(self._pos - self._last_cmd) > 0.5:
            logger.info(f"[gripper/{self._name}] (mock) 下发目标 {self._pos:.1f}mm "
                        f"blocking={blocking}")
        self._last_cmd = self._pos
        if blocking:
            time.sleep(0.1)   # 模拟运动延迟
        return True

    def get_position(self) -> float:
        return self._pos

    def get_status(self) -> dict:
        """与 XenseGripper.get_status 接口一致；mock 下给合成值。"""
        return {'position': self._pos, 'velocity': 0.0, 'force': 0.0, 'temperature': 25.0}
