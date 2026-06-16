# Task Gateway MQTT 协议文档

## 1. 主题（Topic）

| 主题 | 方向 | 说明 |
|------|------|------|
| `robot/{sn}/cmd` | 服务器 → 机器人 | 下发命令 |
| `robot/{sn}/rsp` | 机器人 → 服务器 | 命令回包 |
| `robot/{sn}/event` | 机器人 → 服务器 | 主动推送事件 |

`{sn}` 为机器人序列号，在 `config/robots.yaml` 中配置。

---

## 2. 消息信封

### 命令（cmd）
```json
{
  "msg_id": "a1b2c3d4",
  "method": "start_task",
  "params": {}
}
```

### 回包（rsp）
```json
{
  "msg_id": "a1b2c3d4",
  "success": true,
  "error": "",
  "data": {}
}
```
失败时 `success` 为 `false`，`error` 填原因，`data` 为空对象。

> **时序说明**：rsp 在机器人侧处理完命令后**立即**发出，不等任务执行完成。对于 `start_task`，rsp 表示"任务已接受"；任务的执行结果通过 `event` 推送。

### 事件（event）
```json
{
  "event": "task_status_changed",
  "data": {}
}
```

---

## 3. 命令列表

### 3.1 list_task_types

查询系统支持的任务类型。

**params**
```json
{}
```

**data**
```json
{
  "success": true,
  "failure_reason": "",
  "task_types": [
    {
      "type_name": "navigation",
      "type_description": "导航类任务"
    }
  ]
}
```

---

### 3.2 list_task_templates

查询任务模板列表。

**params**
```json
{
  "task_type": "navigation"
}
```
> `task_type` 可选，不传则返回全部模板。

**data**
```json
{
  "success": true,
  "failure_reason": "",
  "templates": [
    {
      "template_id": 1,
      "task_type": "navigation",
      "description": "导航到目标点"
    }
  ]
}
```

---

### 3.3 get_task_template_detail

查询单个模板的详细信息。

**params**
```json
{
  "template_id": 1
}
```

**data**
```json
{
  "success": true,
  "failure_reason": "",
  "detail": {
    "template_id": 1,
    "task_type": "navigation",
    "description": "导航到目标点",
    "parameter_spec_json": "{\"type\":\"object\",\"required\":[\"x\",\"y\"]}",
    "detail": "机器人移动到指定坐标点，到达后停止。",
    "supports_pause": false
  }
}
```

---

### 3.4 validate_task

校验任务参数是否合法，不实际执行。

**params**
```json
{
  "template_id": 1,
  "parameters_json": "{\"x\": 1.0, \"y\": 2.0}"
}
```

**data**
```json
{
  "can_execute": true,
  "failure_reason": ""
}
```

---

### 3.5 start_task

启动一个任务实例。rsp 在 task_manager 接受任务后**立即**返回，包含 `instance_id`；任务执行结果通过 `task_status_changed` 事件推送。

**params**
```json
{
  "template_id": 1,
  "parameters_json": "{\"x\": 1.0, \"y\": 2.0}",
  "priority": 0,
  "idempotency_key": ""
}
```
> `priority`、`idempotency_key` 可选。

**data**
```json
{
  "started": true,
  "instance_id": 1000,
  "failure_reason": ""
}
```

> `instance_id` 是后续 `terminate_task`、`pause_resume_task`、`get_task_instance_detail` 等操作的必要参数，请在收到 rsp 后保存。

**典型时序：**
```
发送方          robot/{sn}/cmd|rsp|event
  |-- start_task -->|
  |<-- rsp(instance_id=1000, started=true) --|   ← 立即回
  |<-- event(status=running) ---------------|   ← 执行中
  |<-- event(status=success) ---------------|   ← 完成
```

---

### 3.6 terminate_task

终止一个任务实例。

**params**
```json
{
  "instance_id": 1000,
  "reason": "用户手动终止",
  "force": false
}
```
> `reason`、`force` 可选。

**data**
```json
{
  "terminated": true,
  "failure_reason": ""
}
```

---

### 3.7 pause_resume_task

暂停或恢复一个任务实例。

**params**
```json
{
  "instance_id": 1000,
  "operation": "pause",
  "reason": "等待资源"
}
```
> `operation` 取值：`"pause"` 或 `"resume"`。`reason` 可选。

**data**
```json
{
  "success": true,
  "failure_reason": ""
}
```

---

### 3.8 list_active_task_instances

查询当前活跃的任务实例。

**params**
```json
{
  "task_type": "",
  "template_id": 0,
  "status": ""
}
```
> 三个字段均可选，不传则返回全部活跃实例。

**data**
```json
{
  "success": true,
  "failure_reason": "",
  "instances": [
    {
      "instance_id": 1000,
      "template_id": 1,
      "task_type": "navigation",
      "description": "导航到目标点",
      "status": "running",
      "progress": "50",
      "submitted_at": 1718000000,
      "started_at": 1718000001,
      "finished_at": 0,
      "result": "",
      "failure_reason": ""
    }
  ]
}
```

---

### 3.9 list_history_task_instances

查询历史任务实例，支持分页。

**params**
```json
{
  "instance_id": 0,
  "template_id": 0,
  "task_type": "",
  "time_from": 0,
  "time_to": 0,
  "result": "",
  "page_size": 20,
  "cursor": {
    "finished_at_sec": 0,
    "finished_at_nanosec": 0,
    "instance_id": 0
  }
}
```
> 所有字段均可选。`time_from`/`time_to` 为 Unix 时间戳（秒）。

**data**
```json
{
  "success": true,
  "failure_reason": "",
  "total_count": 1,
  "has_more": false,
  "instances": [
    {
      "instance_id": 999,
      "template_id": 1,
      "task_type": "navigation",
      "description": "导航到目标点",
      "status": "success",
      "progress": "100",
      "submitted_at": 1718000000,
      "started_at": 1718000001,
      "finished_at": 1718000060,
      "result": "success",
      "failure_reason": ""
    }
  ]
}
```

---

### 3.10 get_task_instance_detail

查询单个任务实例的详细信息。

**params**
```json
{
  "instance_id": 1000
}
```

**data**
```json
{
  "success": true,
  "failure_reason": "",
  "detail": {
    "instance_id": 1000,
    "template_id": 1,
    "task_type": "navigation",
    "description": "导航到目标点",
    "parameters_json": "{\"x\": 1.0, \"y\": 2.0}",
    "status": "success",
    "result": "success",
    "progress": "100",
    "submitted_at": 1718000000,
    "started_at": 1718000001,
    "finished_at": 1718000060,
    "failure_reason": "",
    "intermediate_log": ""
  }
}
```

---

### 3.11 subscribe_task_status

向 task_manager 注册订阅，获取一个 `subscription_id`。

> **注意**：`task_status_changed` 事件**无论是否调用此接口都会推送**。task_manager 会在每条事件的 `subscription_ids` 字段中标注匹配的订阅 ID，发送方可据此过滤感兴趣的事件。此接口不是接收事件的前提条件。

**params**
```json
{
  "task_type": "navigation"
}
```
> `task_type` 可选，不传则匹配全部类型。

**data**
```json
{
  "success": true,
  "subscription_id": "sub_a1b2c3d4",
  "failure_reason": ""
}
```

---

### 3.12 unsubscribe_task_status

取消任务状态订阅。

**params**
```json
{
  "subscription_id": "sub_a1b2c3d4"
}
```

**data**
```json
{
  "success": true,
  "failure_reason": ""
}
```

---

## 4. 事件推送

task_manager 每次任务状态变更，机器人自动向 `robot/{sn}/event` 推送，**无需提前调用 `subscribe_task_status`**。

`subscription_ids` 字段列出了与本次事件匹配的订阅 ID（由 `subscribe_task_status` 注册），发送方可用于过滤；若未注册任何订阅，该字段为空数组 `[]`，事件仍然推送。

```json
{
  "event": "task_status_changed",
  "data": {
    "subscription_ids": ["sub_a1b2c3d4"],
    "instance_id": 1000,
    "template_id": 1,
    "task_type": "navigation",
    "description": "导航到目标点",
    "status": "running",
    "result": "",
    "progress": "80",
    "submitted_at": 1718000000,
    "started_at": 1718000001,
    "finished_at": 0,
    "failure_reason": ""
  }
}
```

**status 取值**

| 值 | 含义 |
|----|------|
| `running` | 执行中 |
| `paused` | 已暂停 |
| `success` | 成功完成 |
| `failed` | 失败 |

> 时间戳字段（`submitted_at`、`started_at`、`finished_at`）均为 Unix 时间戳（秒），`0` 表示尚未发生。
