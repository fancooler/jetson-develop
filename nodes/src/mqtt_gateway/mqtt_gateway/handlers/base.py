from typing import Callable
from rclpy.node import Node


class BaseHandler:
    """
    每个 ROS2 service 对应一个子类，只需实现：
      _make_request(params)  → srv Request 对象
      _parse_response(resp)  → dict（序列化成 MQTT JSON 的 data 字段）

    done_cb 签名：(success: bool, error: str, data: dict) -> None
    """

    SERVICE_NAME: str = ''
    SERVICE_TYPE = None

    def __init__(self, node: Node):
        self._node = node
        self._client = node.create_client(
            self.SERVICE_TYPE,
            f'/task_manager/{self.SERVICE_NAME}',
        )

    def _make_request(self, params: dict):
        raise NotImplementedError

    def _parse_response(self, response) -> dict:
        raise NotImplementedError

    def handle(self, params: dict, done_cb: Callable[[bool, str, dict], None]):
        if not self._client.service_is_ready():
            done_cb(False, 'task_manager service not available', {})
            return
        try:
            req = self._make_request(params)
        except (KeyError, ValueError, TypeError) as e:
            done_cb(False, f'invalid params: {e}', {})
            return

        future = self._client.call_async(req)
        future.add_done_callback(lambda f: self._on_done(f, done_cb))

    def _on_done(self, future, done_cb: Callable):
        try:
            resp = future.result()
            data = self._parse_response(resp)
            done_cb(True, '', data)
        except Exception as e:
            done_cb(False, str(e), {})


def _time_to_sec(t) -> int:
    return t.sec if t else 0


def _sec_to_time(sec):
    from builtin_interfaces.msg import Time
    t = Time()
    t.sec = int(sec) if sec else 0
    t.nanosec = 0
    return t
