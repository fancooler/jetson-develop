from typing import Callable
from rclpy.node import Node

from .handlers.template_handlers import (
    ListTaskTemplatesHandler,
    GetTaskTemplateDetailHandler,
    ListTaskTypesHandler,
)
from .handlers.instance_handlers import (
    ListActiveTaskInstancesHandler,
    ListHistoryTaskInstancesHandler,
    GetTaskInstanceDetailHandler,
    ValidateTaskHandler,
    StartTaskHandler,
    TerminateTaskHandler,
    PauseResumeTaskHandler,
)
from .handlers.subscription_handlers import (
    SubscribeTaskStatusHandler,
    UnsubscribeTaskStatusHandler,
)


class Dispatcher:
    """
    命令注册表：method 名 → Handler 实例。

    新增命令只需：
      1. 在 handlers/ 下写一个 BaseHandler 子类
      2. 在此处 import 并在 _handlers 字典里加一行
    """

    def __init__(self, node: Node):
        self._handlers = {
            'list_task_templates':          ListTaskTemplatesHandler(node),
            'get_task_template_detail':     GetTaskTemplateDetailHandler(node),
            'list_task_types':              ListTaskTypesHandler(node),
            'list_active_task_instances':   ListActiveTaskInstancesHandler(node),
            'list_history_task_instances':  ListHistoryTaskInstancesHandler(node),
            'get_task_instance_detail':     GetTaskInstanceDetailHandler(node),
            'validate_task':                ValidateTaskHandler(node),
            'start_task':                   StartTaskHandler(node),
            'terminate_task':               TerminateTaskHandler(node),
            'pause_resume_task':            PauseResumeTaskHandler(node),
            'subscribe_task_status':        SubscribeTaskStatusHandler(node),
            'unsubscribe_task_status':      UnsubscribeTaskStatusHandler(node),
        }

    def dispatch(self, method: str, params: dict,
                 done_cb: Callable[[bool, str, dict], None]):
        handler = self._handlers.get(method)
        if handler is None:
            done_cb(False, f"unknown method: '{method}'", {})
            return
        handler.handle(params, done_cb)

    @property
    def known_methods(self):
        return list(self._handlers.keys())
