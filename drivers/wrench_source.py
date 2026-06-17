"""
wrench_source.py — 六维力传感器数据源抽象层

WrenchSource 基类定义统一接口，具体实现负责从各自驱动/协议读取。
换传感器时只需替换 WrenchSource 实现，上层 DualArm / arm_node 代码不变。

当前实现：
  SDKWrenchSource  — 从天机 SDK subscribe() 流读取 fb_joint_them[:6]
  MockWrenchSource — 返回全零（无硬件调试用）

将来扩展示例：
  RS485WrenchSource(port='/dev/ttyUSB0')
  ModbusWrenchSource(host='192.168.1.x', ...)
"""

import logging
from typing import List

logger = logging.getLogger(__name__)

# subscribe()['outputs'] 臂索引（与 arm_utils._ARM_IDX 一致）
_ARM_IDX = {'left': 0, 'right': 1}


class WrenchSource:
    """六维力传感器数据源基类。子类须实现 read / read_from_data。"""

    def read(self, arm: str) -> List[float]:
        """
        独立读取（内部自行获取最新数据帧）。

        Returns:
            [Fx, Fy, Fz, Mx, My, Mz]，单位 N / N·m；失败返回 [0.0]*6。
        """
        raise NotImplementedError

    def read_from_data(self, data: dict, arm: str) -> List[float]:
        """
        从已有 subscribe 数据帧读取（避免额外网络调用，供 read_all_states 使用）。

        Args:
            data: robot.subscribe(dcss) 返回的原始字典
            arm:  'left' | 'right'
        Returns:
            [Fx, Fy, Fz, Mx, My, Mz]；失败返回 [0.0]*6。
        """
        raise NotImplementedError

    def close(self):
        """释放资源（RS-485/Modbus 等需关闭连接的实现会用到）。"""
        pass


class SDKWrenchSource(WrenchSource):
    """从天机 SDK subscribe() 流读取六维力数据（fb_joint_them[:6]）。"""

    def __init__(self, robot, dcss):
        self._robot = robot
        self._dcss  = dcss

    def read(self, arm: str) -> List[float]:
        try:
            data = self._robot.subscribe(self._dcss)
            return self.read_from_data(data, arm)
        except Exception as e:
            logger.warning(f'[{arm}] SDKWrenchSource.read 异常: {e}')
            return [0.0] * 6

    def read_from_data(self, data: dict, arm: str) -> List[float]:
        idx = _ARM_IDX.get(arm, 0)
        try:
            raw = data['outputs'][idx].get('fb_joint_them', [])
            if len(raw) >= 6:
                return [float(v) for v in raw[:6]]
        except Exception as e:
            logger.warning(f'[{arm}] SDKWrenchSource.read_from_data 异常: {e}')
        return [0.0] * 6


class MockWrenchSource(WrenchSource):
    """零值六维力数据源（无硬件 / mock 模式）。"""

    def read(self, arm: str) -> List[float]:
        return [0.0] * 6

    def read_from_data(self, data: dict, arm: str) -> List[float]:
        return [0.0] * 6
