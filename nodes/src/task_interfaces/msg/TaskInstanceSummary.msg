int64 instance_id
int64 template_id
string task_type
string description
string status                        # waiting/running/paused/success/failed/terminated
string progress                      # 0-100 或阶段描述
builtin_interfaces/Time submitted_at
builtin_interfaces/Time started_at   # 未开始则 sec/nanosec 均为 0
builtin_interfaces/Time finished_at  # 仅历史实例有效，实时列表为 0
string result                        # 历史实例：success/failed/terminated；实时为空
string failure_reason
