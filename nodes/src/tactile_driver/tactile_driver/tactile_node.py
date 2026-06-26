"""
tactile_node.py — Xense 视触觉传感器 ROS2 发布节点

发布 topic（左右各一组）：
  /tactile/{side}/image_raw      sensor_msgs/Image  BGR 校正图像
  /tactile/{side}/depth          sensor_msgs/Image  深度图 float32 (mm)
  /tactile/{side}/force          geometry_msgs/WrenchStamped  六维合力
  /tactile/{side}/force_map      sensor_msgs/Image  力分布 35×20×3
  /tactile/{side}/marker         sensor_msgs/Image  切向位移 35×20×2

服务：
  /tactile/calibrate             std_srvs/srv/Trigger  重置参考图像（两爪同时校准）

launch 参数：
  app_dir       (str)  drivers 目录路径，默认 ~/develop/drivers
  publish_rate  (float) 发布频率 Hz，默认 30
  use_gpu       (bool)  是否使用 GPU，默认 True
  left_mac      (str)  左传感器算力卡 MAC，从 robots.yaml 读
  right_mac     (str)  右传感器算力卡 MAC，从 robots.yaml 读
"""

import os
import sys
import threading
import yaml

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

SIDES = ('left', 'right')

IMG_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _load_robot_config():
    path = os.environ.get('ROBOTS_YAML')
    robot_id = os.environ.get('ROBOT_ID', 'robot1')
    if not path or not os.path.exists(path):
        raise RuntimeError(f"$ROBOTS_YAML 未设或文件不存在: {path}")
    with open(path) as f:
        reg = yaml.safe_load(f) or {}
    cfg = reg.get('robots', {}).get(robot_id)
    if cfg is None:
        raise RuntimeError(f"robots.yaml 中找不到 ROBOT_ID={robot_id}")
    return cfg


class TactileNode(Node):

    def __init__(self):
        super().__init__('tactile_driver')

        gp = lambda n, d: self.get_parameter(n).get_parameter_value()

        self.declare_parameter('app_dir',      os.path.expanduser('~/develop/drivers'))
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('use_gpu',      True)
        self.declare_parameter('left_mac',     '')
        self.declare_parameter('right_mac',    '')

        app_dir      = self.get_parameter('app_dir').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        use_gpu      = self.get_parameter('use_gpu').get_parameter_value().bool_value
        left_mac     = self.get_parameter('left_mac').get_parameter_value().string_value
        right_mac    = self.get_parameter('right_mac').get_parameter_value().string_value

        # 若 launch 没传 MAC，从 robots.yaml 读
        if not left_mac or not right_mac:
            cfg = _load_robot_config()
            grippers = cfg.get('grippers', {})
            left_mac  = left_mac  or grippers.get('left_mac', '')
            right_mac = right_mac or grippers.get('right_mac', '')

        self.get_logger().info(
            f"[tactile] left_mac={left_mac} right_mac={right_mac} "
            f"rate={publish_rate}Hz use_gpu={use_gpu}")

        if app_dir not in sys.path:
            sys.path.insert(0, app_dir)

        from tactile_utils import TactileSensor
        OutputType = None  # 延迟取，连接后再拿

        self._bridge = CvBridge()
        self._sensors = {}
        self._output_types = {}

        macs = {'left': left_mac, 'right': right_mac}
        for side, mac in macs.items():
            if not mac:
                self.get_logger().warn(f"[tactile] {side} MAC 未配置，跳过")
                continue
            try:
                s = TactileSensor(mac_addr=mac, use_gpu=use_gpu)
                self._sensors[side] = s
                self._output_types[side] = s.OutputType
                self.get_logger().info(f"[tactile] {side} 已连接")
            except Exception as e:
                self.get_logger().error(f"[tactile] {side} 连接失败: {e}")

        # Publishers
        self._pubs = {}
        for side in self._sensors:
            self._pubs[side] = {
                'image_raw':  self.create_publisher(Image, f'/tactile/{side}/image_raw', IMG_QOS),
                'depth':      self.create_publisher(Image, f'/tactile/{side}/depth', IMG_QOS),
                'force':      self.create_publisher(WrenchStamped, f'/tactile/{side}/force', 10),
                'force_map':  self.create_publisher(Image, f'/tactile/{side}/force_map', IMG_QOS),
                'marker':     self.create_publisher(Image, f'/tactile/{side}/marker', IMG_QOS),
            }

        # Calibrate service
        self.create_service(Trigger, '/tactile/calibrate', self._calibrate_cb)

        # Timer
        self._lock = threading.Lock()
        period = 1.0 / publish_rate
        self.create_timer(period, self._timer_cb)

    def _timer_cb(self):
        now = self.get_clock().now().to_msg()
        for side, sensor in self._sensors.items():
            OT = self._output_types[side]
            try:
                data = sensor.get_data([
                    OT.Rectify,
                    OT.Depth,
                    OT.ForceResultant,
                    OT.Force,
                    OT.Marker2D,
                ])
            except Exception as e:
                self.get_logger().warn(f"[tactile] {side} 采集失败: {e}")
                continue

            pubs = self._pubs[side]

            rectify = data.get(OT.Rectify)
            if rectify is not None:
                msg = self._bridge.cv2_to_imgmsg(rectify, encoding='bgr8')
                msg.header.stamp = now
                pubs['image_raw'].publish(msg)

            depth = data.get(OT.Depth)
            if depth is not None:
                d = depth.astype(np.float32)
                msg = self._bridge.cv2_to_imgmsg(d, encoding='32FC1')
                msg.header.stamp = now
                pubs['depth'].publish(msg)

            force_r = data.get(OT.ForceResultant)
            if force_r is not None:
                msg = WrenchStamped()
                msg.header.stamp = now
                msg.wrench.force.x  = float(force_r[0])
                msg.wrench.force.y  = float(force_r[1])
                msg.wrench.force.z  = float(force_r[2])
                msg.wrench.torque.x = float(force_r[3])
                msg.wrench.torque.y = float(force_r[4])
                msg.wrench.torque.z = float(force_r[5])
                pubs['force'].publish(msg)

            force_map = data.get(OT.Force)
            if force_map is not None:
                fm = (force_map * 255).clip(0, 255).astype(np.uint8)
                msg = self._bridge.cv2_to_imgmsg(fm, encoding='bgr8')
                msg.header.stamp = now
                pubs['force_map'].publish(msg)

            marker = data.get(OT.Marker2D)
            if marker is not None:
                # (35,20,2) float32 → 打包为双通道 32FC2
                m = marker.astype(np.float32)
                msg = Image()
                msg.header.stamp = now
                msg.height = m.shape[0]
                msg.width  = m.shape[1]
                msg.encoding = '32FC2'
                msg.step = m.shape[1] * 4 * 2
                msg.data = m.tobytes()
                pubs['marker'].publish(msg)

    def _calibrate_cb(self, request, response):
        for side, sensor in self._sensors.items():
            try:
                sensor.calibrate()
                self.get_logger().info(f"[tactile] {side} 校准完成")
            except Exception as e:
                self.get_logger().error(f"[tactile] {side} 校准失败: {e}")
                response.success = False
                response.message = str(e)
                return response
        response.success = True
        response.message = "两侧传感器校准完成"
        return response

    def destroy_node(self):
        for side, sensor in self._sensors.items():
            sensor.release()
            self.get_logger().info(f"[tactile] {side} 已释放")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TactileNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
