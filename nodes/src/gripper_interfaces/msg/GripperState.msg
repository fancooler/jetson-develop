# 单个夹爪状态
string name           # "left" | "right"
bool connected        # 是否已连接
float64 position      # 当前位置 mm（约 2=闭合 ~ 85=张开）
float64 velocity      # mm/s（不可用时为 NaN）
float64 force         # N（不可用时为 NaN）
float64 temperature   # ℃（不可用时为 NaN）
bool moving           # 是否在运动（距上次目标 > 容差）
