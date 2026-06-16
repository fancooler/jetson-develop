#!/usr/bin/env python3
"""
send_cmd.py — 向 mqtt_gateway 发送单条 MQTT 命令并打印回包

用法：
    python3 send_cmd.py --sn z700/Z700-001 --method start_task \
        --params '{"template_id":1,"parameters_json":"{}"}'

    python3 send_cmd.py --sn z700/Z700-001 --method list_task_templates

依赖：pip install paho-mqtt
"""

import argparse
import json
import threading
import time
import uuid

import paho.mqtt.client as mqtt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sn',     required=True,  help='robots.yaml 中的 sn 字段')
    parser.add_argument('--method', required=True,  help='命令名，如 list_task_templates')
    parser.add_argument('--params', default='{}',   help='JSON 字符串，默认 {}')
    parser.add_argument('--broker', default='192.168.124.200')
    parser.add_argument('--port',   type=int, default=1883)
    parser.add_argument('--timeout', type=float, default=10.0, help='等待回包超时秒数')
    args = parser.parse_args()

    try:
        params = json.loads(args.params)
    except json.JSONDecodeError as e:
        print(f'[错误] --params 不是合法 JSON：{e}')
        raise SystemExit(1)

    msg_id    = uuid.uuid4().hex[:8]
    cmd_topic = f'robot/{args.sn}/cmd'
    rsp_topic = f'robot/{args.sn}/rsp'
    evt_topic = f'robot/{args.sn}/event'
    payload   = json.dumps({'msg_id': msg_id, 'method': args.method, 'params': params},
                           ensure_ascii=False)

    done  = threading.Event()
    result = {}

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(rsp_topic)
            client.subscribe(evt_topic)
            print(f'[已连接] broker={args.broker}:{args.port}')
            print(f'[发送]   topic={cmd_topic}')
            print(f'         {payload}')
            client.publish(cmd_topic, payload, qos=1)
        else:
            print(f'[错误] 连接失败 rc={rc}')
            done.set()

    def on_message(client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
        except Exception:
            return
        if msg.topic == rsp_topic and data.get('msg_id') == msg_id:
            result['data'] = data
            done.set()
        elif msg.topic == evt_topic:
            print(f'[event]  {json.dumps(data, ensure_ascii=False, indent=2)}')

    cid = f'send_cmd_{uuid.uuid4().hex[:6]}'
    try:
        client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1, client_id=cid)
    except AttributeError:
        client = mqtt.Client(cid)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(args.broker, args.port, keepalive=60)
    except Exception as e:
        print(f'[错误] 无法连接 broker：{e}')
        raise SystemExit(1)

    client.loop_start()
    timed_out = not done.wait(args.timeout)
    client.loop_stop()
    client.disconnect()

    if timed_out:
        print(f'\n[超时] {args.timeout}s 内未收到回包，检查 mqtt_gateway 是否在运行')
        raise SystemExit(1)

    resp = result.get('data', {})
    print(f'\n[回包]')
    print(json.dumps(resp, ensure_ascii=False, indent=2))

    if resp.get('success'):
        print('\n结果：PASS')
    else:
        print(f'\n结果：FAIL  error={resp.get("error", "")}')
        raise SystemExit(1)


if __name__ == '__main__':
    main()
