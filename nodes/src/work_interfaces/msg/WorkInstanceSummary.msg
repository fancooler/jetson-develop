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
int64 created_at                     # Unix seconds
int64 started_at                     # 0 if not started
int64 finished_at                    # 0 if not finished
int64 estimated_duration             # seconds, 0 if unknown
int64 estimated_start_at             # 0 if not applicable
int64 estimated_finish_at            # 0 if unknown
int64 updated_at
string failure_reason
