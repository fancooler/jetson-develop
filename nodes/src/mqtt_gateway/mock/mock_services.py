#!/usr/bin/env python3
"""
mock_services.py — 用于 broker 通信调试的 task_manager 桩节点

实现全部 12 个 /task_manager/* 服务，收到调用立即返回合理的假数据，
使 mqtt_gateway 能走完 MQTT 命令→ROS2 service→MQTT 回包的完整链路。

可选：--emit-events  每 5 秒向 /task_manager/task_status_events 发一条假事件，
      用于验证 gateway 的事件推送路径（robot/{sn}/event）。

用法：
    source ~/ros2_ws/install/setup.bash
    python3 mock/mock_services.py
    python3 mock/mock_services.py --emit-events
"""

import argparse
import time
import uuid

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node

from task_interfaces.msg import (
    TaskInstanceDetail,
    TaskInstanceSummary,
    TaskStatusEvent,
    TaskTemplateDetail,
    TaskTemplateSummary,
    TaskTypeInfo,
)
from task_interfaces.srv import (
    GetTaskInstanceDetail,
    GetTaskTemplateDetail,
    ListActiveTaskInstances,
    ListHistoryTaskInstances,
    ListTaskTemplates,
    ListTaskTypes,
    PauseResumeTask,
    StartTask,
    SubscribeTaskStatus,
    TerminateTask,
    UnsubscribeTaskStatus,
    ValidateTask,
)


def _now() -> Time:
    t = Time()
    t.sec = int(time.time())
    return t


class MockServices(Node):

    def __init__(self, emit_events: bool):
        super().__init__('task_manager')
        self._next_instance_id = 1000
        self._emit_events = emit_events

        self.create_service(ListTaskTemplates,        '/task_manager/list_task_templates',         self._list_templates)
        self.create_service(GetTaskTemplateDetail,    '/task_manager/get_task_template_detail',    self._get_template_detail)
        self.create_service(ListActiveTaskInstances,  '/task_manager/list_active_task_instances',  self._list_active)
        self.create_service(ListHistoryTaskInstances, '/task_manager/list_history_task_instances', self._list_history)
        self.create_service(GetTaskInstanceDetail,    '/task_manager/get_task_instance_detail',    self._get_instance_detail)
        self.create_service(ValidateTask,             '/task_manager/validate_task',               self._validate)
        self.create_service(StartTask,                '/task_manager/start_task',                  self._start)
        self.create_service(TerminateTask,            '/task_manager/terminate_task',              self._terminate)
        self.create_service(PauseResumeTask,          '/task_manager/pause_resume_task',           self._pause_resume)
        self.create_service(ListTaskTypes,            '/task_manager/list_task_types',             self._list_types)
        self.create_service(SubscribeTaskStatus,      '/task_manager/subscribe_task_status',       self._subscribe)
        self.create_service(UnsubscribeTaskStatus,    '/task_manager/unsubscribe_task_status',     self._unsubscribe)

        if emit_events:
            self._event_pub = self.create_publisher(TaskStatusEvent, '/task_manager/task_status_events', 10)
            self.create_timer(5.0, self._publish_fake_event)
            self.get_logger().info('事件推送已启用，每 5 秒发一条假 TaskStatusEvent')

        self.get_logger().info('MockServices ready — all 12 services advertised')

    # ── services ────────────────────────────────────────────────────────────

    def _list_templates(self, req, resp):
        t1 = TaskTemplateSummary()
        t1.template_id = 1; t1.task_type = 'navigation'; t1.description = '导航到目标点'
        t2 = TaskTemplateSummary()
        t2.template_id = 2; t2.task_type = 'manipulation'; t2.description = '抓取物体'
        resp.success = True
        resp.templates = [t1] if (req.task_type and req.task_type == t1.task_type) else [t1, t2]
        self.get_logger().info(f'list_task_templates → {len(resp.templates)} items')
        return resp

    def _get_template_detail(self, req, resp):
        d = TaskTemplateDetail()
        d.template_id = req.template_id; d.task_type = 'navigation'
        d.description = '导航到目标点'
        d.parameter_spec_json = '{"type":"object","required":["x","y"]}'
        d.detail = '移动到指定坐标。'; d.supports_pause = False
        resp.success = True; resp.detail = d
        self.get_logger().info(f'get_task_template_detail template_id={req.template_id}')
        return resp

    def _list_active(self, req, resp):
        resp.success = True; resp.instances = []
        self.get_logger().info('list_active_task_instances → 0 items')
        return resp

    def _list_history(self, req, resp):
        s = TaskInstanceSummary()
        s.instance_id = 999; s.template_id = 1; s.task_type = 'navigation'
        s.description = '导航到目标点'; s.status = 'success'; s.result = 'success'
        s.progress = '100'; s.submitted_at = _now(); s.started_at = _now(); s.finished_at = _now()
        resp.success = True; resp.total_count = 1; resp.has_more = False; resp.instances = [s]
        self.get_logger().info('list_history_task_instances → 1 item')
        return resp

    def _get_instance_detail(self, req, resp):
        d = TaskInstanceDetail()
        d.instance_id = req.instance_id; d.template_id = 1; d.task_type = 'navigation'
        d.description = '导航到目标点'; d.parameters_json = '{"x":1.0,"y":2.0}'
        d.status = 'success'; d.result = 'success'; d.progress = '100'
        d.submitted_at = _now(); d.started_at = _now(); d.finished_at = _now()
        resp.success = True; resp.detail = d
        self.get_logger().info(f'get_task_instance_detail instance_id={req.instance_id}')
        return resp

    def _validate(self, req, resp):
        resp.can_execute = True; resp.failure_reason = ''
        self.get_logger().info(f'validate_task template_id={req.template_id} → can_execute=True')
        return resp

    def _start(self, req, resp):
        resp.started = True
        resp.instance_id = self._next_instance_id
        self._next_instance_id += 1
        self.get_logger().info(f'start_task template_id={req.template_id} → instance_id={resp.instance_id}')
        return resp

    def _terminate(self, req, resp):
        resp.terminated = True; resp.failure_reason = ''
        self.get_logger().info(f'terminate_task instance_id={req.instance_id}')
        return resp

    def _pause_resume(self, req, resp):
        resp.success = True; resp.failure_reason = ''
        self.get_logger().info(f'pause_resume_task operation={req.operation} instance_id={req.instance_id} reason={req.reason!r}')
        return resp

    def _list_types(self, req, resp):
        t1 = TaskTypeInfo(); t1.type_name = 'navigation';   t1.type_description = '导航类任务'
        t2 = TaskTypeInfo(); t2.type_name = 'manipulation'; t2.type_description = '操作类任务'
        resp.success = True; resp.task_types = [t1, t2]
        self.get_logger().info('list_task_types → 2 items')
        return resp

    def _subscribe(self, req, resp):
        resp.success = True
        resp.subscription_id = f'sub_{uuid.uuid4().hex[:8]}'
        self.get_logger().info(f'subscribe_task_status → subscription_id={resp.subscription_id}')
        return resp

    def _unsubscribe(self, req, resp):
        resp.success = True; resp.failure_reason = ''
        self.get_logger().info(f'unsubscribe_task_status subscription_id={req.subscription_id}')
        return resp

    # ── event injection ─────────────────────────────────────────────────────

    def _publish_fake_event(self):
        statuses = ['running', 'paused', 'success', 'failed']
        msg = TaskStatusEvent()
        msg.instance_id = self._next_instance_id - 1 if self._next_instance_id > 1000 else 999
        msg.template_id = 1
        msg.task_type = 'navigation'
        msg.description = '导航到目标点'
        msg.status = statuses[int(time.time()) % len(statuses)]
        msg.result = 'success' if msg.status == 'success' else ''
        msg.progress = '50'
        msg.submitted_at = _now()
        msg.started_at = _now()
        msg.finished_at = _now() if msg.status in ('success', 'failed') else Time()
        self._event_pub.publish(msg)
        self.get_logger().info(f'[event] published fake TaskStatusEvent status={msg.status} instance_id={msg.instance_id}')


def main():
    parser = argparse.ArgumentParser(description='task_manager mock for broker debugging')
    parser.add_argument('--emit-events', action='store_true',
                        help='每 5 秒发一条假 TaskStatusEvent，验证 gateway 事件推送路径')
    args = parser.parse_args()

    rclpy.init()
    node = MockServices(emit_events=args.emit_events)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
