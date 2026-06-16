import logging
import os
import queue

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .dispatcher import Dispatcher
from .event_forwarder import EventForwarder
from .mqtt_adapter import MqttAdapter


class TaskGatewayNode(Node):
    """
    MQTT ↔ ROS2 任务网关节点。

    收命令流程：
      MQTT broker → paho 回调（paho 线程）→ _cmd_queue
      → Timer 定时清队列（ROS2 executor 线程）
      → Dispatcher → Handler.call_async() + done_callback
      → mqtt_adapter.publish_response()

    推状态流程：
      /task_manager/task_status_events（ROS2 topic）
      → EventForwarder → mqtt_adapter.publish_event()
    """

    def __init__(self):
        super().__init__('mqtt_gateway')

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        self._config = self._load_config(config_path)

        # 从 robots.yaml（$ROBOTS_YAML + $ROBOT_ID）读取机器人专属字段
        sn = self._load_robot_sn()

        gw_cfg = self._config.get('gateway', {})
        self._cmd_queue = queue.Queue(maxsize=gw_cfg.get('cmd_queue_size', 100))

        self._dispatcher = Dispatcher(self)
        self._forwarder  = EventForwarder(self)

        self._mqtt = MqttAdapter(
            config=self._config['mqtt'],
            sn=sn,
            topics=self._config['topics'],
            on_command=self._on_mqtt_command,
            logger=self.get_logger(),
        )
        self._forwarder.set_publish_callback(self._mqtt.publish_event)

        # 50 ms 定时清队列（20 Hz）
        self.create_timer(0.05, self._drain_cmd_queue)

        self._mqtt.connect()
        broker = self._config['mqtt'].get('host', '')
        self.get_logger().info(f'TaskGateway started (robot_id={os.environ.get("ROBOT_ID")} sn={sn} broker={broker})')

    # ── 命令处理 ───────────────────────────────────────────────────────────────

    def _on_mqtt_command(self, msg_id: str, method: str, params: dict):
        # paho 线程：只入队，不调 ROS2
        try:
            self._cmd_queue.put_nowait((msg_id, method, params))
        except queue.Full:
            self.get_logger().warning(
                f'命令队列已满，丢弃: method={method} msg_id={msg_id}'
            )

    def _drain_cmd_queue(self):
        # ROS2 executor 线程（Timer 回调）
        while True:
            try:
                msg_id, method, params = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            self._dispatch(msg_id, method, params)

    def _dispatch(self, msg_id: str, method: str, params: dict):
        def done_cb(success: bool, error: str, data: dict):
            self._mqtt.publish_response(msg_id, method, success, error, data)

        self.get_logger().debug(f'Dispatching method={method} msg_id={msg_id}')
        self._dispatcher.dispatch(method, params, done_cb)

    # ── 配置加载 ───────────────────────────────────────────────────────────────

    def _load_config(self, path: str) -> dict:
        # 优先：节点参数 > 环境变量 > 包内默认
        if not path:
            path = os.environ.get('GATEWAY_CONFIG', '')
        if not path:
            path = os.path.join(
                get_package_share_directory('mqtt_gateway'),
                'config', 'gateway_config.yaml',
            )
        self.get_logger().info(f'Loading gateway config: {path}')
        with open(path, encoding='utf-8') as f:
            return yaml.safe_load(f)

    def _load_robot_sn(self) -> str:
        """从 $ROBOTS_YAML 按 $ROBOT_ID 读取 sn。"""
        robots_yaml = os.environ.get('ROBOTS_YAML', '')
        robot_id    = os.environ.get('ROBOT_ID', '')

        if not robots_yaml or not robot_id:
            raise RuntimeError(
                '$ROBOTS_YAML 或 $ROBOT_ID 未设置。'
                '请确认已 source <repo>/config/robot_env.sh。'
            )
        if not os.path.exists(robots_yaml):
            raise RuntimeError(f'$ROBOTS_YAML={robots_yaml} 文件不存在。')

        with open(robots_yaml, encoding='utf-8') as f:
            registry = yaml.safe_load(f)

        robot = (registry or {}).get('robots', {}).get(robot_id)
        if robot is None:
            raise RuntimeError(f'robots.yaml 中找不到 ROBOT_ID={robot_id!r}。')

        sn = robot.get('sn', '')
        if not sn or sn.startswith('TODO'):
            self.get_logger().warning(f'robots.yaml 中 {robot_id}.sn 未填写真实值：{sn!r}')

        self.get_logger().info(f'Loaded from robots.yaml: robot_id={robot_id} sn={sn}')
        return sn

    def shutdown(self):
        self._mqtt.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = TaskGatewayNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
