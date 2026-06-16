#!/usr/bin/env python3
"""
test_client.py — mqtt_gateway MQTT 测试客户端

模拟服务器侧：发送命令并检查回包。每条测试用例打印 PASS / FAIL。
覆盖全部 12 个接口 + 参数校验 + 错误处理，共 ~30 个用例。

用法：
    python3 test_client.py --sn <robot_sn> [--broker 192.168.124.200] [--port 1883]

依赖：paho-mqtt（pip install paho-mqtt）
"""

import argparse
import json
import threading
import time
import uuid

import paho.mqtt.client as mqtt


class GatewayTestClient:

    def __init__(self, sn: str, broker: str, port: int):
        self._sn         = sn
        self._cmd_topic  = f'robot/{sn}/cmd'
        self._rsp_topic  = f'robot/{sn}/rsp'
        self._evt_topic  = f'robot/{sn}/event'
        self._pending    = {}
        self._responses  = {}
        self._lock       = threading.Lock()

        cid = f'test_client_{uuid.uuid4().hex[:6]}'
        try:
            self._client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1, client_id=cid)
        except AttributeError:
            self._client = mqtt.Client(cid)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.connect(broker, port)
        self._client.loop_start()
        time.sleep(0.5)

    def _on_connect(self, client, userdata, flags, rc):
        client.subscribe(self._rsp_topic)
        client.subscribe(self._evt_topic)
        print(f'[client] connected  broker={self._sn}  rsp={self._rsp_topic}')

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
        except Exception:
            return
        if msg.topic == self._rsp_topic:
            mid = data.get('msg_id', '')
            with self._lock:
                self._responses[mid] = data
                ev = self._pending.get(mid)
            if ev:
                ev.set()
        elif msg.topic == self._evt_topic:
            print(f'  [event] {json.dumps(data, ensure_ascii=False)}')

    def call(self, method: str, params: dict, timeout: float = 5.0) -> dict:
        msg_id = uuid.uuid4().hex[:8]
        ev = threading.Event()
        with self._lock:
            self._pending[msg_id] = ev
        self._client.publish(
            self._cmd_topic,
            json.dumps({'msg_id': msg_id, 'method': method, 'params': params}),
            qos=1,
        )
        ok = ev.wait(timeout)
        with self._lock:
            self._pending.pop(msg_id, None)
            resp = self._responses.pop(msg_id, None)
        if not ok or resp is None:
            return {'success': False, 'error': f'timeout after {timeout}s', 'data': {}}
        return resp

    def stop(self):
        self._client.loop_stop()
        self._client.disconnect()


def run_tests(client: GatewayTestClient):
    passed = failed = 0

    def check(name: str, resp: dict, expect_key: str = None, expect_val=None,
              expect_fail: bool = False):
        """
        expect_fail=True：期望 success=False（测错误路径）
        expect_key/val：在 data 中额外校验某字段值
        """
        nonlocal passed, failed
        success = resp.get('success', False)
        if expect_fail:
            ok = not success
        else:
            ok = success
            if expect_key is not None and ok:
                ok = resp.get('data', {}).get(expect_key) == expect_val
        tag = 'PASS' if ok else 'FAIL'
        print(f'  [{tag}] {name}')
        if not ok:
            print(f'         resp={json.dumps(resp, ensure_ascii=False)}')
            failed += 1
        else:
            passed += 1
        return resp

    # ── 1. 模板查询 ──────────────────────────────────────────────────────────

    print('\n── 1. 模板查询 ─────────────────────────────')

    check('list_task_types',
          client.call('list_task_types', {}))

    check('list_task_templates（全部）',
          client.call('list_task_templates', {}))

    check('list_task_templates（按 task_type 过滤）',
          client.call('list_task_templates', {'task_type': 'navigation'}))

    check('list_task_templates（不存在的类型 → 成功但空列表）',
          client.call('list_task_templates', {'task_type': 'nonexistent'}))

    check('get_task_template_detail（template_id=1）',
          client.call('get_task_template_detail', {'template_id': 1}))

    check('get_task_template_detail（缺少 template_id → 失败）',
          client.call('get_task_template_detail', {}),
          expect_fail=True)

    # ── 2. 参数校验 ──────────────────────────────────────────────────────────

    print('\n── 2. 参数校验 ─────────────────────────────')

    check('validate_task（合法参数）',
          client.call('validate_task', {
              'template_id': 1,
              'parameters_json': '{"x":1.0,"y":2.0}',
          }),
          expect_key='can_execute', expect_val=True)

    check('validate_task（缺少 template_id → 失败）',
          client.call('validate_task', {'parameters_json': '{}'}),
          expect_fail=True)

    # ── 3. 任务生命周期 ───────────────────────────────────────────────────────

    print('\n── 3. 任务生命周期 ──────────────────────────')

    check('start_task（基础）',
          client.call('start_task', {
              'template_id': 1,
              'parameters_json': '{}',
          }),
          expect_key='started', expect_val=True)

    check('start_task（带 priority + idempotency_key）',
          client.call('start_task', {
              'template_id': 1,
              'parameters_json': '{}',
              'priority': 5,
              'idempotency_key': f'idem_{uuid.uuid4().hex[:6]}',
          }),
          expect_key='started', expect_val=True)

    check('start_task（缺少 template_id → 失败）',
          client.call('start_task', {'parameters_json': '{}'}),
          expect_fail=True)

    # 生命周期测试用 pausable_test（template_id=2），运行时间足够长
    resp_lc = check('start_task（pausable_test，用于生命周期测试）',
                    client.call('start_task', {'template_id': 2, 'parameters_json': '{}'}),
                    expect_key='started', expect_val=True)
    lc_id = resp_lc.get('data', {}).get('instance_id', 0)
    print(f'         → instance_id={lc_id}')

    if lc_id:
        check('pause_resume_task（pause）',
              client.call('pause_resume_task', {
                  'instance_id': lc_id,
                  'operation': 'pause',
                  'reason': '等待资源',
              }),
              expect_key='success', expect_val=True)

        check('pause_resume_task（resume）',
              client.call('pause_resume_task', {
                  'instance_id': lc_id,
                  'operation': 'resume',
                  'reason': '恢复执行',
              }),
              expect_key='success', expect_val=True)

        check('pause_resume_task（非法 operation → 失败）',
              client.call('pause_resume_task', {
                  'instance_id': lc_id,
                  'operation': 'invalid_op',
              }),
              expect_fail=True)

        check('terminate_task（正常）',
              client.call('terminate_task', {
                  'instance_id': lc_id,
                  'reason': '测试终止',
              }),
              expect_key='terminated', expect_val=True)

    resp_lc2 = check('start_task（第二个 pausable_test，用于 force terminate）',
                     client.call('start_task', {'template_id': 2, 'parameters_json': '{}'}),
                     expect_key='started', expect_val=True)
    lc_id2 = resp_lc2.get('data', {}).get('instance_id', 0)
    if lc_id2:
        check('terminate_task（force=True）',
              client.call('terminate_task', {
                  'instance_id': lc_id2,
                  'reason': '强制终止测试',
                  'force': True,
              }),
              expect_key='terminated', expect_val=True)

    check('terminate_task（缺少 instance_id → 失败）',
          client.call('terminate_task', {}),
          expect_fail=True)

    # ── 4. 实例查询 ──────────────────────────────────────────────────────────

    print('\n── 4. 实例查询 ─────────────────────────────')

    check('list_active_task_instances（无过滤）',
          client.call('list_active_task_instances', {}))

    check('list_active_task_instances（按 task_type 过滤）',
          client.call('list_active_task_instances', {'task_type': 'navigation'}))

    check('list_active_task_instances（按 status 过滤）',
          client.call('list_active_task_instances', {'status': 'running'}))

    check('list_active_task_instances（按 template_id 过滤）',
          client.call('list_active_task_instances', {'template_id': 1}))

    check('list_history_task_instances（无过滤）',
          client.call('list_history_task_instances', {}))

    check('list_history_task_instances（按 task_type 过滤）',
          client.call('list_history_task_instances', {'task_type': 'navigation'}))

    check('list_history_task_instances（按 result 过滤）',
          client.call('list_history_task_instances', {'result': 'success'}))

    check('list_history_task_instances（分页 page_size=1）',
          client.call('list_history_task_instances', {'page_size': 1}))

    if lc_id:
        check('get_task_instance_detail（已知 instance_id）',
              client.call('get_task_instance_detail', {'instance_id': lc_id}))

    check('get_task_instance_detail（缺少 instance_id → 失败）',
          client.call('get_task_instance_detail', {}),
          expect_fail=True)

    # ── 5. 订阅管理 ──────────────────────────────────────────────────────────

    print('\n── 5. 订阅管理 ─────────────────────────────')

    resp_sub1 = check('subscribe_task_status（指定 task_type）',
                      client.call('subscribe_task_status', {'task_type': 'navigation'}))
    sub_id1 = resp_sub1.get('data', {}).get('subscription_id', '')
    print(f'         → subscription_id={sub_id1}')

    resp_sub2 = check('subscribe_task_status（不指定 task_type，全部）',
                      client.call('subscribe_task_status', {}))
    sub_id2 = resp_sub2.get('data', {}).get('subscription_id', '')
    print(f'         → subscription_id={sub_id2}')

    if sub_id1:
        check('unsubscribe_task_status（有效 sub_id）',
              client.call('unsubscribe_task_status', {'subscription_id': sub_id1}))

    if sub_id2:
        check('unsubscribe_task_status（第二个 sub_id）',
              client.call('unsubscribe_task_status', {'subscription_id': sub_id2}))

    check('unsubscribe_task_status（缺少 subscription_id → 失败）',
          client.call('unsubscribe_task_status', {}),
          expect_fail=True)

    # ── 6. 错误处理 ──────────────────────────────────────────────────────────

    print('\n── 6. 错误处理 ─────────────────────────────')

    check('未知 method → 失败',
          client.call('unknown_method', {}),
          expect_fail=True)

    check('method 为空字符串 → 失败',
          client.call('', {}),
          expect_fail=True)

    # ── 汇总 ─────────────────────────────────────────────────────────────────

    total = passed + failed
    print(f'\n结果：{passed}/{total} passed，{failed} failed\n')
    return failed == 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sn',     required=True,  help='机器人序列号（robots.yaml 中的 sn 字段）')
    parser.add_argument('--broker', default='192.168.124.200')
    parser.add_argument('--port',   type=int, default=1883)
    args = parser.parse_args()

    print(f'Connecting to broker {args.broker}:{args.port}  sn={args.sn}')
    client = GatewayTestClient(args.sn, args.broker, args.port)

    try:
        success = run_tests(client)
    finally:
        client.stop()

    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
