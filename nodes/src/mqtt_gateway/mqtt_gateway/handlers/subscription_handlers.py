from task_interfaces.srv import SubscribeTaskStatus, UnsubscribeTaskStatus
from .base import BaseHandler


class SubscribeTaskStatusHandler(BaseHandler):
    SERVICE_NAME = 'subscribe_task_status'
    SERVICE_TYPE = SubscribeTaskStatus

    def _make_request(self, params: dict):
        req = SubscribeTaskStatus.Request()
        req.task_type   = params.get('task_type', '')
        req.template_id = int(params.get('template_id', 0))
        req.instance_id = int(params.get('instance_id', 0))
        return req

    def _parse_response(self, resp) -> dict:
        return {
            'success': resp.success,
            'subscription_id': resp.subscription_id,
            'failure_reason': resp.failure_reason,
        }


class UnsubscribeTaskStatusHandler(BaseHandler):
    SERVICE_NAME = 'unsubscribe_task_status'
    SERVICE_TYPE = UnsubscribeTaskStatus

    def _make_request(self, params: dict):
        req = UnsubscribeTaskStatus.Request()
        req.subscription_id = params['subscription_id']
        return req

    def _parse_response(self, resp) -> dict:
        return {
            'success': resp.success,
            'failure_reason': resp.failure_reason,
        }
