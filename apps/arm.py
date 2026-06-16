"""
arm.py — 向后兼容封装

TJArm 是 SingleArm 的薄层包装，保留旧版接口供 runner.py 使用：
  - 不传 arm 参数，由 config.ARM 决定左/右臂
  - get_ee_state()  → [x,y,z,roll,pitch,yaw]，臂基坐标系（非 base_link）
  - move_to_ee()    → 臂基坐标系目标，IK → 关节角指令

新代码请直接使用 arm_utils.SingleArm / DualArm（base_link 坐标系接口）。
"""

import logging

import config_single as config
from arm_utils import SingleArm

logger = logging.getLogger(__name__)

_ARM_STR = {'A': 'left', 'B': 'right'}


class TJArm(SingleArm):
    """
    向后兼容单臂封装。

    臂由 config.ARM（'A'=左, 'B'=右）决定，无需手动指定。
    get_ee_state() / move_to_ee() 使用臂基坐标系（与旧 runner.py 一致）。
    """

    def __init__(self):
        arm = _ARM_STR.get(config.ARM, 'left')
        super().__init__(arm)

    # ── 旧接口（臂基坐标系）──────────────────────────────────────────────────

    def get_ee_state(self) -> list:
        """
        读取末端状态，返回臂基坐标系 [x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad]。

        注意：这是臂自身的 FK/IK 基坐标系，不是 base_link 坐标系。
              新代码请改用 get_ee_state_base()（base_link 系）。
        """
        fk = self.get_fk_raw()
        if fk is None:
            logger.warning(f"[{self.arm_str}] FK 失败，返回零向量")
            return [0.0] * 6
        return config.mm_deg_to_m_rad(fk)

    def move_to_ee(self, x_m: float, y_m: float, z_m: float,
                   roll_rad: float, pitch_rad: float, yaw_rad: float) -> bool:
        """
        臂基坐标系目标位姿 → IK → 关节角指令（非阻塞）。

        注意：输入为臂基坐标系，不是 base_link 坐标系。
              新代码请改用 move_to_ee_base()（base_link 系）。
        """
        xyzabc = config.m_rad_to_mm_deg([x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad])
        return self._send_xyzabc(xyzabc, safe=True)
