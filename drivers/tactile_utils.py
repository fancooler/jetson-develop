"""
tactile_utils.py  —  Xense 视触觉传感器驱动封装

职责：
  封装 xensesdk.Sensor 的生命周期（初始化、采集、校准、释放），
  屏蔽 SDK 细节，向上层（tactile_driver ROS2 节点）提供简洁接口。

连接方式：
  通过算力卡 MAC 地址远程连接（与 xensegripper 同一机制，走有线网卡）。
  mac_addr 从 robots.yaml 读取，格式为无冒号小写，如 "3ad820773a85"。

OutputType 说明：
  Rectify       BGR 校正图像，shape=(700, 400, 3)，uint8
  Depth         深度图，shape=(h, w)，float32，单位 mm
  Force         三维力分布，shape=(35, 20, 3)，float32
  ForceResultant 六维合力向量，shape=(6,)，float32，[fx,fy,fz,tx,ty,tz]
  Marker2D      切向位移，shape=(35, 20, 2)，float32

用法：
  sensor = TactileSensor(mac_addr="3ad820773a85")
  data = sensor.get_data([OutputType.Rectify, OutputType.ForceResultant])
  img   = data[OutputType.Rectify]    # np.ndarray (700,400,3)
  force = data[OutputType.ForceResultant]  # np.ndarray (6,)
  sensor.release()
"""

import logging
from typing import Optional, List, Dict
import numpy as np

log = logging.getLogger(__name__)


# 延迟 import，避免在无 xensesdk 环境（如 ThinkBook）下 import 本模块报错
def _import_sdk():
    try:
        import xensesdk
        from xensesdk import Sensor
        OutputType = Sensor.OutputType
        return Sensor, OutputType
    except ImportError as e:
        raise ImportError(f"xensesdk 未安装或无法加载: {e}") from e


class TactileSensor:
    """单个视触觉传感器封装（对应一只手指）"""

    def __init__(
        self,
        mac_addr: str,
        config_path: Optional[str] = None,
        use_gpu: bool = True,
    ):
        """
        Args:
            mac_addr:    算力卡 MAC 地址，无冒号小写，如 "3ad820773a85"
            config_path: 个性化标定文件路径或目录；None 时 SDK 使用内置通用配置
            use_gpu:     是否使用 GPU 推理，默认 True
        """
        self._sensor = None
        Sensor, self.OutputType = _import_sdk()
        from xensesdk import call_service

        master_service = f"master_{mac_addr}"
        log.info(f"[tactile] 扫描传感器序列号 {master_service} ...")
        ret = call_service(master_service, "scan_sensor_sn", timeout_sec=3)
        if not ret:
            raise RuntimeError(f"找不到传感器，mac_addr={mac_addr}（设备未上电或未接网线）")

        serial_number = list(ret.keys())[0]
        log.info(f"[tactile] 发现传感器 serial={serial_number}，正在连接...")
        self._sensor = Sensor.create(
            serial_number,
            mac_addr=mac_addr,
            config_path=config_path,
            use_gpu=use_gpu,
        )
        if not self._sensor:
            raise RuntimeError(f"TactileSensor 初始化失败，serial={serial_number}")

        self._mac_addr = mac_addr
        log.info(f"[tactile] 已连接 mac={mac_addr}")

    def get_data(self, outputs: List) -> Dict:
        """
        采集一帧传感器数据。

        Args:
            outputs: OutputType 列表，如 [OutputType.Rectify, OutputType.ForceResultant]

        Returns:
            dict: {OutputType -> np.ndarray}，某路数据缺失时对应值为 None
        """
        results = self._sensor.selectSensorInfo(*outputs)
        if not isinstance(results, (list, tuple)):
            results = [results]
        return dict(zip(outputs, results))

    def get_rectify_image(self) -> np.ndarray:
        """快捷方法：获取校正图像 (700, 400, 3) uint8"""
        return self._sensor.getRectifyImage()

    def calibrate(self) -> None:
        """重置参考图像（在无物理接触时调用，相当于重新调零）"""
        log.info(f"[tactile] 校准传感器 mac={self._mac_addr}")
        self._sensor.resetReferenceImage()

    def release(self) -> None:
        """释放传感器资源"""
        if self._sensor:
            self._sensor.release()
            self._sensor = None
            log.info(f"[tactile] 已释放 mac={self._mac_addr}")

    def __del__(self):
        self.release()
