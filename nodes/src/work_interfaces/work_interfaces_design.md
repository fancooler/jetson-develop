# work_interfaces — Long Task Chain ROS 接口设计

> **包名**：`work_interfaces`
> **用途**：Web App ↔ mqtt_gateway ↔ MQTT Backend 的 ROS 接口层，与 `task_interfaces` 完全独立，概念不同、不混用。

---

## 架构

```
Web App
  │  ROS Service Call / Topic Subscribe
  ▼
mqtt_gateway  ←→  MQTT  ←→  Backend Server
  │
  │  ROS Topic Publish（事件推送）
  ▼
Web App
```

mqtt_gateway 作为 ROS 服务服务端，接收 Web App 调用 → 转为 MQTT 命令发给后端 → 等待后端 MQTT 回包 → 返回 ROS 响应。

---

## 核心概念

```
作业模板 work（work_id: string）           ← 长任务链"定义"
   └── 任务模板 task（task_id: string）     ← 有序子任务

        │  start_work
        ▼

作业实例 work_instance（work_instance_id: string）  ← 一次具体执行
   └── 任务实例 task_instance（task_instance_id: string）
```

**ID 约定**
- 作业模板：`work_*`（如 `work_nav_001`）
- 任务模板：`task_*`（如 `task_nav_to_start`）
- 作业实例：`wi_*`（如 `wi_20260622_0001`）
- 任务实例：`ti_*`（如 `ti_20260622_0001_2`，后缀为任务序号）

**状态值（作业实例与任务实例通用）**
`queued` / `executing` / `paused` / `succeeded` / `failed` / `terminated`

**时间字段**：统一为 Unix 秒（`int64`），未赋值时为 `0`。

---

## 一、消息类型（msg，11 个）

### WorkTaskTypeInfo.msg
```
string type_name
string type_description
```

---

### WorkSummary.msg
> 用于 list_works 列表项

```
string work_id
string work_name
string work_description
bool supports_pause
```

---

### WorkTaskInfo.msg
> work 详情中的子任务模板

```
string task_id
string task_name
string task_description
uint32 sequence
string param_schema_json
string task_detail
bool supports_pause
string[] applicable_end_effectors
```

---

### WorkDetail.msg
> 作业模板完整定义，含子任务列表

```
string work_id
string work_name
string work_description
string param_schema_json
string work_detail
bool supports_pause
string[] applicable_end_effectors
uint32 task_count
WorkTaskInfo[] tasks
```

---

### WorkExecutorInfo.msg
> 执行机器人信息（实例排队未分配时不使用）

```
string robot_sn
string robot_name
int32 battery
```

---

### WorkInstanceSummary.msg
> 作业实例摘要，实时列表与历史列表通用

```
string work_instance_id
string work_id
string work_name
string order_no
string work_description
string status                        # queued/executing/paused/succeeded/failed/terminated
string priority                      # low/normal/high
int32 progress                       # 0-100
uint32 current_task_sequence         # 0 if not started
uint32 total_task_count
string current_task_name             # empty if not started
string current_task_instance_id      # empty if not started
bool has_executor                    # false when queued and not yet assigned
WorkExecutorInfo executor            # valid only when has_executor=true
int32 queue_position                 # -1 if not queued
int64 created_at
int64 started_at                     # 0 if not started
int64 finished_at                    # 0 if not finished
int64 estimated_duration             # seconds, 0 if unknown
int64 estimated_start_at             # 0 if not applicable
int64 estimated_finish_at            # 0 if unknown
int64 updated_at
string failure_reason
```

---

### WorkTaskInstanceSummary.msg
> 作业实例详情中嵌套的子任务实例摘要

```
string task_instance_id
string task_id
string task_name
uint32 sequence
string status
int32 progress
int64 started_at
int64 finished_at
string failure_reason
```

---

### WorkInstanceDetail.msg
> 作业实例完整详情，含子任务实例列表

```
string work_instance_id
string work_id
string work_name
string recipe
string station
string robot_sn
string order_no
string work_description
string params_json
string status
string priority
int32 progress
uint32 current_task_sequence
uint32 total_task_count
string current_task_name
string current_task_instance_id
int64 created_at
int64 started_at
int64 finished_at
string failure_reason
string log_summary
WorkTaskInstanceSummary[] task_instances
```

---

### WorkTaskInstanceDetail.msg
> 单个任务实例完整详情

```
string task_instance_id
string work_instance_id
string work_id
string task_id
string task_name
uint32 sequence
string params_json
string status
int32 progress
int64 created_at
int64 started_at
int64 finished_at
string failure_reason
string log_summary
```

---

### WorkPrecheckItem.msg
> 预校验结果中的单项

```
string item
bool passed
string message
```

---

### WorkStatusEvent.msg
> mqtt_gateway → Web App 的作业状态变更推送（与 WorkListRunningWorks 响应同构，全量快照）

```
int64 server_time
uint32 executing_count
uint32 queued_count
uint32 paused_count
uint32 total_count
WorkInstanceSummary[] works
```

---

## 二、服务类型（srv，12 个）

### WorkListTaskTypes.srv
> 对应 MQTT: 无（任务类型查询为本地能力）

```
# 无请求参数
---
bool success
string failure_reason
WorkTaskTypeInfo[] task_types
```

---

### WorkListWorks.srv
> 对应 MQTT: `list_works`

```
# 无请求参数
---
bool success
string failure_reason
WorkSummary[] works
```

---

### WorkGetWork.srv
> 对应 MQTT: `get_work`

```
string work_id
---
bool success
string failure_reason
WorkDetail work
```

---

### WorkListRunningWorks.srv
> 对应 MQTT: `list_running_works`

```
# 无请求参数
---
bool success
string failure_reason
int64 server_time
uint32 executing_count
uint32 queued_count
uint32 paused_count
uint32 total_count
WorkInstanceSummary[] works
```

---

### WorkListHistoryWorks.srv
> 对应 MQTT: `list_history_works`

```
string status               # 可选：succeeded/failed/terminated；空=不过滤
int64 start_time            # 可选，Unix 秒；0=不过滤
int64 end_time              # 可选，Unix 秒；0=不过滤
uint32 page                 # 页码，默认 1
uint32 page_size            # 每页条数，0=节点默认值（20）
---
bool success
string failure_reason
int64 server_time
uint32 page
uint32 page_size
uint32 total
WorkInstanceSummary[] works
```

---

### WorkGetWorkInstance.srv
> 对应 MQTT: `get_work_instance`

```
string work_instance_id
---
bool success
string failure_reason
WorkInstanceDetail work_instance
```

---

### WorkGetTaskInstance.srv
> 对应 MQTT: `get_task_instance`

```
string task_instance_id
---
bool success
string failure_reason
WorkTaskInstanceDetail task_instance
```

---

### WorkPrecheck.srv
> 对应 MQTT: `precheck_work`

```
string work_id
string params_json
---
bool success
string failure_reason
bool passed
WorkPrecheckItem[] checks
```

---

### WorkStartWork.srv
> 对应 MQTT: `start_work`

```
string work_id
string params_json
string priority             # low/normal/high；空=normal
string idempotency_key      # 可选，防重复下发
string button_id            # 可选，UI 按钮标识，原样回传
---
bool success
string failure_reason
bool started
string work_instance_id
string status               # executing 或 queued
string button_id
```

---

### WorkStopWork.srv
> 对应 MQTT: `stop_work`

```
string work_instance_id
string stop_reason          # 可选，审计用
bool force
string button_id            # 可选，UI 按钮标识，原样回传
---
bool success
string failure_reason
bool stopped
string button_id
```

---

### WorkPauseWork.srv
> 对应 MQTT: `pause_work`

```
string work_instance_id
string button_id
---
bool success
string failure_reason
string button_id
```

---

### WorkResumeWork.srv
> 对应 MQTT: `resume_work`

```
string work_instance_id
string button_id
---
bool success
string failure_reason
string button_id
```

---

## 三、事件推送 Topic

| 字段 | 值 |
|---|---|
| Topic 名 | `/mqtt_gateway/work_status_events` |
| 消息类型 | `work_interfaces/msg/WorkStatusEvent` |
| 发布方 | mqtt_gateway（收到 backend MQTT `work_status_changed` 后转发） |
| 订阅方 | Web App |

每次作业实例状态变更时推送全量实时列表快照（与 WorkListRunningWorks 同构）。

---

## 四、接口汇总

| MQTT method | ROS srv | 说明 |
|---|---|---|
| — | WorkListTaskTypes | 查询任务类型 |
| `list_works` | WorkListWorks | 查询可用作业列表 |
| `get_work` | WorkGetWork | 查询作业详情 |
| `list_running_works` | WorkListRunningWorks | 查询实时作业实例 |
| `list_history_works` | WorkListHistoryWorks | 查询历史作业实例 |
| `get_work_instance` | WorkGetWorkInstance | 查询作业实例详情 |
| `get_task_instance` | WorkGetTaskInstance | 查询任务实例详情 |
| `precheck_work` | WorkPrecheck | 作业执行预校验 |
| `start_work` | WorkStartWork | 开始执行作业 |
| `stop_work` | WorkStopWork | 终止作业 |
| `pause_work` | WorkPauseWork | 暂停作业 |
| `resume_work` | WorkResumeWork | 恢复作业 |
| `work_status_changed` (event) | Topic: `/mqtt_gateway/work_status_events` | 状态变更推送 |

---

## 五、待定接口

| 接口 | 说明 |
|---|---|
| 基础配置上报（SN/型号/末端设备 → Backend） | mqtt_gateway 启动时自动上报，不经 Web App 触发，接口待后续定义 |
