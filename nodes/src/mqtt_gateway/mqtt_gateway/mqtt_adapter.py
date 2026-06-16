import json
import threading
import uuid
from typing import Callable

import paho.mqtt.client as mqtt


class MqttAdapter:
    """
    paho-mqtt 封装。

    线程模型：
      paho 在后台线程（loop_start）处理网络 I/O 和回调。
      on_command 回调在 paho 线程触发，只应做入队操作（不调 ROS2）。
      publish_* 方法线程安全，可从 ROS2 executor 线程调用。
    """

    def __init__(self, config: dict, sn: str, topics: dict,
                 on_command: Callable[[str, str, dict], None], logger=None):
        self._sn           = sn
        self._cmd_topic    = topics['cmd_in'].format(sn=sn)
        self._rsp_topic    = topics['rsp_out'].format(sn=sn)
        self._event_topic  = topics['event_out'].format(sn=sn)
        self._on_command   = on_command
        self._logger       = logger

        client_id = config.get('client_id') or f'mqtt_gateway_{sn}_{uuid.uuid4().hex[:8]}'
        try:
            self._client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
                client_id=client_id, protocol=mqtt.MQTTv311)
        except AttributeError:
            self._client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)

        if config.get('username'):
            self._client.username_pw_set(config['username'], config.get('password', ''))

        self._client.on_connect    = self._handle_connect
        self._client.on_disconnect = self._handle_disconnect
        self._client.on_message    = self._handle_message
        self._client.reconnect_delay_set(min_delay=1, max_delay=30)

        self._host      = config['host']
        self._port      = int(config.get('port', 1883))
        self._keepalive = int(config.get('keepalive', 60))

    def connect(self):
        if self._logger:
            self._logger.info(f'Connecting to MQTT broker {self._host}:{self._port}')
        self._client.connect(self._host, self._port, self._keepalive)
        self._client.loop_start()

    def disconnect(self):
        self._client.loop_stop()
        self._client.disconnect()
        if self._logger:
            self._logger.info('MQTT disconnected')

    def publish_response(self, msg_id: str, method: str,
                         success: bool, error: str, data: dict):
        payload = {
            'msg_id':  msg_id,
            'method':  method,
            'success': success,
            'error':   error,
            'data':    data,
        }
        if self._logger:
            self._logger.info(f'RSP  method={method} msg_id={msg_id} success={success} error={error!r} data={json.dumps(data, ensure_ascii=False)}')
        threading.Timer(0.2, self._publish, args=[self._rsp_topic, payload]).start()

    def publish_event(self, data: dict):
        payload = {'event': 'task_status_changed', 'data': data}
        self._publish(self._event_topic, payload)

    # ── 内部 ──────────────────────────────────────────────────────────────────

    def _publish(self, topic: str, payload: dict):
        try:
            self._client.publish(topic, json.dumps(payload, ensure_ascii=False), qos=1)
        except Exception as e:
            if self._logger:
                self._logger.error(f'Publish failed: topic={topic} error={e}')

    def _handle_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(self._cmd_topic, qos=1)
            if self._logger:
                self._logger.info(f'MQTT connected — subscribed to {self._cmd_topic}')
        else:
            if self._logger:
                self._logger.error(f'MQTT connect failed: rc={rc} ({mqtt.connack_string(rc)})')

    def _handle_disconnect(self, client, userdata, rc):
        if rc != 0:
            if self._logger:
                self._logger.warning(f'MQTT disconnected unexpectedly (rc={rc}), will auto-reconnect')

    def _handle_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
        except Exception as e:
            if self._logger:
                self._logger.warning(f'Invalid JSON in command message: {e}')
            return

        msg_id = data.get('msg_id', '')
        method = data.get('method', '')
        params = data.get('params', {})

        if not method:
            if self._logger:
                self._logger.warning(f'Command missing "method" field, msg_id={msg_id!r}')
            return

        if self._logger:
            self._logger.info(f'CMD  method={method} msg_id={msg_id} params={json.dumps(params, ensure_ascii=False)}')
        self._on_command(msg_id, method, params)
