from typing import Callable, Optional
from rclpy.node import Node
from task_interfaces.msg import TaskStatusEvent

from .handlers.base import _time_to_sec


class EventForwarder:
    """
    订阅 ROS2 topic /task_manager/task_status_events，
    将每条状态变更推送序列化后通过 publish_cb 转发到 MQTT。
    """

    def __init__(self, node: Node):
        self._publish_cb: Optional[Callable[[dict], None]] = None
        self._sub = node.create_subscription(
            TaskStatusEvent,
            '/task_manager/task_status_events',
            self._on_event,
            10,
        )

    def set_publish_callback(self, cb: Callable[[dict], None]):
        self._publish_cb = cb

    def _on_event(self, msg: TaskStatusEvent):
        if self._publish_cb is None:
            return
        self._publish_cb({
            'subscription_ids': list(msg.subscription_ids),
            'instance_id':      msg.instance_id,
            'template_id':      msg.template_id,
            'task_type':        msg.task_type,
            'description':      msg.description,
            'status':           msg.status,
            'result':           msg.result,
            'progress':         msg.progress,
            'submitted_at':     _time_to_sec(msg.submitted_at),
            'started_at':       _time_to_sec(msg.started_at),
            'finished_at':      _time_to_sec(msg.finished_at),
            'failure_reason':   msg.failure_reason,
        })
