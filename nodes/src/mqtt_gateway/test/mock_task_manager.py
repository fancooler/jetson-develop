#!/usr/bin/env python3
"""
mock_task_manager.py — task_manager 桩节点，用于 mqtt_gateway 全链路测试

实现了全部 12 个 /task_manager/* 服务，返回硬编码的合理数据。
不需要真实业务逻辑，只验证 gateway 能正确调用并序列化结果。

用法：
    source ~/ros2_ws/install/setup.bash
    python3 mock_task_manager.py
"""

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

from task_interfaces.srv import (
    ListTaskTemplates, GetTaskTemplateDetail,
    ListActiveTaskInstances, ListHistoryTaskInstances,
    GetTaskInstanceDetail, ValidateTask,
    StartTask, TerminateTask, PauseResumeTask,
    ListTaskTypes, SubscribeTaskStatus, UnsubscribeTaskStatus,
)
from task_interfaces.msg import (
    TaskTemplateSummary, TaskTemplateDetail,
    TaskInstanceSummary, TaskInstanceDetail,
    TaskTypeInfo,
)


def _now() -> Time:
    import time
    t = Time()
    t.sec = int(time.time())
    return t


class MockTaskManager(Node):

    def __init__(self):
        super().__init__('task_manager')
        self._next_instance_id = 1000

        self.create_service(ListTaskTemplates,       '/task_manager/list_task_templates',          self._list_templates)
        self.create_service(GetTaskTemplateDetail,   '/task_manager/get_task_template_detail',     self._get_template_detail)
        self.create_service(ListActiveTaskInstances, '/task_manager/list_active_task_instances',   self._list_active)
        self.create_service(ListHistoryTaskInstances,'/task_manager/list_history_task_instances',  self._list_history)
        self.create_service(GetTaskInstanceDetail,   '/task_manager/get_task_instance_detail',     self._get_instance_detail)
        self.create_service(ValidateTask,            '/task_manager/validate_task',                self._validate)
        self.create_service(StartTask,               '/task_manager/start_task',                   self._start)
        self.create_service(TerminateTask,           '/task_manager/terminate_task',               self._terminate)
        self.create_service(PauseResumeTask,         '/task_manager/pause_resume_task',            self._pause_resume)
        self.create_service(ListTaskTypes,           '/task_manager/list_task_types',              self._list_types)
        self.create_service(SubscribeTaskStatus,     '/task_manager/subscribe_task_status',        self._subscribe)
        self.create_service(UnsubscribeTaskStatus,   '/task_manager/unsubscribe_task_status',      self._unsubscribe)

        self.get_logger().info('MockTaskManager ready — all 12 services advertised')

    def _list_templates(self, req, resp):
        t1 = TaskTemplateSummary()
        t1.template_id = 1; t1.task_type = 'navigation'; t1.description = '导航到目标点'
        t2 = TaskTemplateSummary()
        t2.template_id = 2; t2.task_type = 'manipulation'; t2.description = '抓取物体'
        resp.success = True
        resp.templates = [t] if (req.task_type and req.task_type == t1.task_type) else [t1, t2]
        return resp

    def _get_template_detail(self, req, resp):
        d = TaskTemplateDetail()
        d.template_id = req.template_id
        d.task_type = 'navigation'
        d.description = '导航到目标点'
        d.parameter_spec_json = '{"type":"object","required":["x","y"],"properties":{"x":{"type":"number"},"y":{"type":"number"},"yaw":{"type":"number","default":0}}}'
        d.detail = '机器人移动到指定坐标点，到达后停止。'
        d.supports_pause = False
        resp.success = True; resp.detail = d
        return resp

    def _list_active(self, req, resp):
        resp.success = True; resp.instances = []
        return resp

    def _list_history(self, req, resp):
        s = TaskInstanceSummary()
        s.instance_id = 999; s.template_id = 1; s.task_type = 'navigation'
        s.description = '导航到目标点'; s.status = 'success'; s.result = 'success'
        s.progress = '100'; s.submitted_at = _now(); s.started_at = _now(); s.finished_at = _now()
        resp.success = True; resp.total_count = 1; resp.has_more = False; resp.instances = [s]
        return resp

    def _get_instance_detail(self, req, resp):
        d = TaskInstanceDetail()
        d.instance_id = req.instance_id; d.template_id = 1; d.task_type = 'navigation'
        d.description = '导航到目标点'; d.parameters_json = '{"x":1.0,"y":2.0}'
        d.status = 'success'; d.result = 'success'; d.progress = '100'
        d.submitted_at = _now(); d.started_at = _now(); d.finished_at = _now()
        resp.success = True; resp.detail = d
        return resp

    def _validate(self, req, resp):
        resp.can_execute = True; resp.failure_reason = ''
        return resp

    def _start(self, req, resp):
        resp.started = True
        resp.instance_id = self._next_instance_id
        self._next_instance_id += 1
        self.get_logger().info(f'Task started: template_id={req.template_id} instance_id={resp.instance_id}')
        return resp

    def _terminate(self, req, resp):
        resp.terminated = True; resp.failure_reason = ''
        self.get_logger().info(f'Task terminated: instance_id={req.instance_id}')
        return resp

    def _pause_resume(self, req, resp):
        resp.success = True; resp.failure_reason = ''
        self.get_logger().info(f'Task {req.operation}: instance_id={req.instance_id}')
        return resp

    def _list_types(self, req, resp):
        t1 = TaskTypeInfo(); t1.type_name = 'navigation';   t1.type_description = '导航类任务'
        t2 = TaskTypeInfo(); t2.type_name = 'manipulation'; t2.type_description = '操作类任务'
        resp.success = True; resp.task_types = [t1, t2]
        return resp

    def _subscribe(self, req, resp):
        import uuid
        resp.success = True
        resp.subscription_id = f'sub_{uuid.uuid4().hex[:8]}'
        self.get_logger().info(f'Subscription created: {resp.subscription_id}')
        return resp

    def _unsubscribe(self, req, resp):
        resp.success = True; resp.failure_reason = ''
        return resp


def main():
    rclpy.init()
    node = MockTaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
