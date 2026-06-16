from task_interfaces.srv import ListTaskTemplates, GetTaskTemplateDetail, ListTaskTypes
from .base import BaseHandler


class ListTaskTemplatesHandler(BaseHandler):
    SERVICE_NAME = 'list_task_templates'
    SERVICE_TYPE = ListTaskTemplates

    def _make_request(self, params: dict):
        req = ListTaskTemplates.Request()
        req.task_type = params.get('task_type', '')
        return req

    def _parse_response(self, resp) -> dict:
        return {
            'success': resp.success,
            'failure_reason': resp.failure_reason,
            'templates': [
                {
                    'template_id': t.template_id,
                    'task_type': t.task_type,
                    'description': t.description,
                }
                for t in resp.templates
            ],
        }


class GetTaskTemplateDetailHandler(BaseHandler):
    SERVICE_NAME = 'get_task_template_detail'
    SERVICE_TYPE = GetTaskTemplateDetail

    def _make_request(self, params: dict):
        req = GetTaskTemplateDetail.Request()
        req.template_id = int(params['template_id'])
        return req

    def _parse_response(self, resp) -> dict:
        d = resp.detail
        return {
            'success': resp.success,
            'failure_reason': resp.failure_reason,
            'detail': {
                'template_id': d.template_id,
                'task_type': d.task_type,
                'description': d.description,
                'parameter_spec_json': d.parameter_spec_json,
                'detail': d.detail,
                'supports_pause': d.supports_pause,
            },
        }


class ListTaskTypesHandler(BaseHandler):
    SERVICE_NAME = 'list_task_types'
    SERVICE_TYPE = ListTaskTypes

    def _make_request(self, params: dict):
        return ListTaskTypes.Request()

    def _parse_response(self, resp) -> dict:
        return {
            'success': resp.success,
            'failure_reason': resp.failure_reason,
            'task_types': [
                {'type_name': t.type_name, 'type_description': t.type_description}
                for t in resp.task_types
            ],
        }
