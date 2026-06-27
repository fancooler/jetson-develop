string task_instance_id
string task_id
string task_name
uint32 sequence
string status                        # queued/executing/paused/succeeded/failed/terminated
int32 progress                       # 0-100
int64 started_at                     # 0 if not started
int64 finished_at                    # 0 if not finished
string failure_reason
