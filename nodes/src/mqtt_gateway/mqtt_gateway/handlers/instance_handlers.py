from task_interfaces.srv import (
    ListActiveTaskInstances,
    ListHistoryTaskInstances,
    GetTaskInstanceDetail,
    ValidateTask,
    StartTask,
    TerminateTask,
    PauseResumeTask,
)
from .base import BaseHandler, _time_to_sec, _sec_to_time


def _instance_summary(i) -> dict:
    return {
        'instance_id': i.instance_id,
        'template_id': i.template_id,
        'task_type': i.task_type,
        'description': i.description,
        'status': i.status,
        'progress': i.progress,
        'submitted_at': _time_to_sec(i.submitted_at),
        'started_at': _time_to_sec(i.started_at),
        'finished_at': _time_to_sec(i.finished_at),
        'result': i.result,
        'failure_reason': i.failure_reason,
    }


class ListActiveTaskInstancesHandler(BaseHandler):
    SERVICE_NAME = 'list_active_task_instances'
    SERVICE_TYPE = ListActiveTaskInstances

    def _make_request(self, params: dict):
        req = ListActiveTaskInstances.Request()
        req.task_type  = params.get('task_type', '')
        req.template_id = int(params.get('template_id', 0))
        req.status     = params.get('status', '')
        return req

    def _parse_response(self, resp) -> dict:
        return {
            'success': resp.success,
            'failure_reason': resp.failure_reason,
            'instances': [_instance_summary(i) for i in resp.instances],
        }


class ListHistoryTaskInstancesHandler(BaseHandler):
    SERVICE_NAME = 'list_history_task_instances'
    SERVICE_TYPE = ListHistoryTaskInstances

    def _make_request(self, params: dict):
        req = ListHistoryTaskInstances.Request()
        req.instance_id  = int(params.get('instance_id', 0))
        req.template_id  = int(params.get('template_id', 0))
        req.task_type    = params.get('task_type', '')
        req.time_from    = _sec_to_time(params.get('time_from', 0))
        req.time_to      = _sec_to_time(params.get('time_to', 0))
        req.result       = params.get('result', '')
        req.page_size    = int(params.get('page_size', 0))
        cursor           = params.get('cursor', {})
        req.cursor_finished_at_sec    = int(cursor.get('finished_at_sec', 0))
        req.cursor_finished_at_nanosec = int(cursor.get('finished_at_nanosec', 0))
        req.cursor_instance_id        = int(cursor.get('instance_id', 0))
        return req

    def _parse_response(self, resp) -> dict:
        return {
            'success': resp.success,
            'failure_reason': resp.failure_reason,
            'total_count': resp.total_count,
            'has_more': resp.has_more,
            'instances': [_instance_summary(i) for i in resp.instances],
        }


class GetTaskInstanceDetailHandler(BaseHandler):
    SERVICE_NAME = 'get_task_instance_detail'
    SERVICE_TYPE = GetTaskInstanceDetail

    def _make_request(self, params: dict):
        req = GetTaskInstanceDetail.Request()
        req.instance_id = int(params['instance_id'])
        return req

    def _parse_response(self, resp) -> dict:
        d = resp.detail
        return {
            'success': resp.success,
            'failure_reason': resp.failure_reason,
            'detail': {
                'instance_id': d.instance_id,
                'template_id': d.template_id,
                'task_type': d.task_type,
                'description': d.description,
                'parameters_json': d.parameters_json,
                'status': d.status,
                'result': d.result,
                'progress': d.progress,
                'submitted_at': _time_to_sec(d.submitted_at),
                'started_at': _time_to_sec(d.started_at),
                'finished_at': _time_to_sec(d.finished_at),
                'failure_reason': d.failure_reason,
                'intermediate_log': d.intermediate_log,
            },
        }


class ValidateTaskHandler(BaseHandler):
    SERVICE_NAME = 'validate_task'
    SERVICE_TYPE = ValidateTask

    def _make_request(self, params: dict):
        req = ValidateTask.Request()
        req.template_id     = int(params['template_id'])
        req.parameters_json = params.get('parameters_json', '{}')
        return req

    def _parse_response(self, resp) -> dict:
        return {
            'can_execute': resp.can_execute,
            'failure_reason': resp.failure_reason,
        }


class StartTaskHandler(BaseHandler):
    SERVICE_NAME = 'start_task'
    SERVICE_TYPE = StartTask

    def _make_request(self, params: dict):
        req = StartTask.Request()
        req.template_id      = int(params['template_id'])
        req.parameters_json  = params.get('parameters_json', '{}')
        req.priority         = int(params.get('priority', 0))
        req.idempotency_key  = params.get('idempotency_key', '')
        return req

    def _parse_response(self, resp) -> dict:
        return {
            'started': resp.started,
            'instance_id': resp.instance_id,
            'failure_reason': resp.failure_reason,
        }


class TerminateTaskHandler(BaseHandler):
    SERVICE_NAME = 'terminate_task'
    SERVICE_TYPE = TerminateTask

    def _make_request(self, params: dict):
        req = TerminateTask.Request()
        req.instance_id = int(params['instance_id'])
        req.reason      = params.get('reason', '')
        req.force       = bool(params.get('force', False))
        return req

    def _parse_response(self, resp) -> dict:
        return {
            'terminated': resp.terminated,
            'failure_reason': resp.failure_reason,
        }


class PauseResumeTaskHandler(BaseHandler):
    SERVICE_NAME = 'pause_resume_task'
    SERVICE_TYPE = PauseResumeTask

    def _make_request(self, params: dict):
        req = PauseResumeTask.Request()
        req.instance_id = int(params['instance_id'])
        req.operation   = params['operation']   # 'pause' or 'resume'
        if req.operation not in ('pause', 'resume'):
            raise ValueError(f"operation must be 'pause' or 'resume', got '{req.operation}'")
        req.reason = params.get('reason', '')
        return req

    def _parse_response(self, resp) -> dict:
        return {
            'success': resp.success,
            'failure_reason': resp.failure_reason,
        }
