# arm_client

通用机械臂**客户端**封装（厂商无关）。对着通用 `/arm/*` 话题/动作/服务
（[`arm_interfaces`](../arm_interfaces)）封装出一组阻塞式同步 API + 命令行，
让上层（脚本、状态机、推理 runner）像调函数一样控制机械臂，不用手敲
`ros2 action send_goal` / `ros2 service call`。

> 只依赖 `arm_interfaces` + 标准消息，**不依赖任何厂商 SDK / jetson-work**。
> 在 ThinkBook 跑即可通过跨机 DDS 控制 Jetson 上的 `arm_node`（驱动见
> [`tj_marvin_driver`](../tj_marvin_driver)）。两端 `ROS_DOMAIN_ID` 须一致。

## 构建 / 环境

```bash
cd ~/work/jetson-ros2
colcon build --packages-select arm_interfaces arm_client
source install/setup.bash
export ROS_DOMAIN_ID=1          # 与 Jetson 一致（ROBOT_ID=robot1 → 1）
```

## 命令行 arm_cli

```bash
ros2 run arm_client arm_cli status                       # 状态 + 关节角(°)
ros2 run arm_client arm_cli home both --variant 0        # 回 HOME（双臂）
ros2 run arm_client arm_cli joints right --right 10 20 30 -40 50 -10 5
ros2 run arm_client arm_cli pose right --pos 0.3 -0.2 0.4 --rpy 0 0 0
ros2 run arm_client arm_cli connect                      # connect/release/estop/enter-pos
ros2 run arm_client arm_cli set-mode position

# 流式控制（Phase 2）：自动 开流式→线性插值到目标→关流式
ros2 run arm_client arm_cli stream right --right 8 -8 16 -50 24 2 -4 --duration 1.5 --rate 30
```

全局参数：`--ns`(默认 /arm) `--timeout`(动作超时秒,<=0 用节点默认)
`--server-timeout`(等节点就绪秒) `--feedback`(运动中打印关节误差)。
关节角单位 **度**；`pose` 姿态用 `--rpy r p y`(rad) 或 `--quat x y z w` 二选一。

## 作为库（Python）

```python
from arm_client import ArmClient

with ArmClient() as arm:                       # 自动 init/shutdown
    if not arm.wait_for_servers(timeout=10.0):
        raise RuntimeError('arm_node 未就绪')
    print(arm.connect())                       # (True, '已连接')
    print(arm.go_home('both'))
    print(arm.move_to_joints('right', right=[10, 20, 30, -40, 50, -10, 5]))
    print(arm.move_to_pose('right', position=[0.3, -0.2, 0.4], rpy=[0, 0, 0]))
    print(arm.status)                          # 最新 ArmStatus（缓存）
    print(arm.joints_dict())                   # {'left':[..7 rad..], 'right':[..]}

    # 流式控制（Phase 2，对接 20Hz 推理/遥操作）
    arm.enable_streaming(True)
    arm.stream_to('right', target_right=[10, 20, 30, -40, 50, -10, 5], duration=2.0, rate=20.0)
    # 或自己按节奏逐帧发：arm.stream_joints('right', right=[...])  # 度
    arm.enable_streaming(False)
```

> 流式与离散动作**互斥**：开流式时 `move_*`/`go_home` 会被拒；`estop` 会自动关流式。

所有动作/服务方法返回 `(success: bool, message: str)`。动作方法支持
`feedback_cb=lambda max_err_deg: ...`。内部自建后台 executor 线程，
方法可在任意线程调用（**勿在 ROS 回调内调用**，会自死锁）。

## 设计

- `ArmClient`（`client.py`）：订阅缓存 `/arm/status`、`/arm/joint_states`、
  `/arm/ee_pose_*`；服务/动作用 `*_async`+future，done_callback+Event 等待，
  不与后台 executor 抢 spin。
- `arm_cli`（`cli.py`）：argparse 子命令薄封装，console_script 入口。
