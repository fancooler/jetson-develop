# tj_marvin_driver

天机 MaRVIN 双臂的 ROS2 驱动（Phase 1：离散高层目标）。薄包装 jetson-work 仓
`app/arm_utils.py` 的 `DualArm`（真机）/ `MockDualArm`（无硬件），对外实现通用
[`arm_interfaces`](../arm_interfaces)。多型号机械臂将来各写一个 `*_driver`，
复用同一套 `arm_interfaces` 与 `/arm/...` 话题。

> **只能在 Jetson 跑真机**（天机 SDK 是 aarch64 `.so`，且需有线连控制器 192.168.1.190）。
> ThinkBook 作为客户端只需 `arm_interfaces`，无需 SDK / jetson-work。

## 运行

```bash
# 默认 mock，不碰真机
ros2 launch tj_marvin_driver tj_marvin.launch.py

# 接真机（需现场监督、急停在手、低速）—— 操作前务必看 REAL_HARDWARE.md
ros2 launch tj_marvin_driver tj_marvin.launch.py use_mock:=false
```

> 🔴 **接真机前必读 [`REAL_HARDWARE.md`](REAL_HARDWARE.md)**（检查清单、启动顺序、
> 真机特有坑）。尤其注意：控制器 `192.168.1.190` 与当前 WiFi 同网段会冲突；
> go_home 后需先 `/arm/enter_position_mode` 再运动。

参数：`use_mock`(true) `app_dir`(~/work/app) `auto_connect`(true) `publish_rate`(25.0)；
节点内另有 `reach_tol_deg`(1.0) `default_timeout`(30.0)。

## 接口

**发布**
| 话题 | 类型 | 说明 |
|------|------|------|
| `/arm/joint_states` | `sensor_msgs/JointState` | 14 关节（left_joint1..7, right_joint1..7），**rad** |
| `/arm/ee_pose_left` / `_right` | `geometry_msgs/PoseStamped` | base_link 系（mock 下无 EE，不发布） |
| `/arm/status` | `arm_interfaces/ArmStatus` | connected/estopped/busy/**streaming**/ctrl_mode |

**订阅**
| 话题 | 类型 | 说明 |
|------|------|------|
| `/arm/joint_command` | `arm_interfaces/JointCommand` | **流式**关节目标(°)，BestEffort/depth1；需先开 `enable_streaming` |

**动作**
| 名称 | 类型 | 说明 |
|------|------|------|
| `/arm/move_to_joints` | `MoveToJoints` | arm(left/right/both)+目标关节角(°) |
| `/arm/move_to_pose` | `MoveToPose` | arm(left/right)+`Pose`@base_link |
| `/arm/go_home` | `GoHome` | arm（HOME 来自 config_dual） |

**服务**：`/arm/connect` `/arm/release` `/arm/enter_position_mode` `/arm/estop`（`std_srvs/Trigger`）；`/arm/set_ctrl_mode`（`arm_interfaces/SetCtrlMode`）；`/arm/enable_streaming`（`std_srvs/SetBool`，开/关流式）

### 流式控制（Phase 2）

对接 20Hz 推理 / 遥操作的连续关节控制（非点到点）：

1. `/arm/enable_streaming data:true` 开启流式（要求已连接、非 estop、无动作在跑）；
2. 高频发布 `/arm/joint_command`（°），节点透传到非阻塞 `move_joints(safe=True)`；
3. `/arm/enable_streaming data:false` 关闭。

**与离散动作互斥**：流式开启时 `move_to_*`/`go_home` 会被拒；反之亦然。`/arm/estop` 立即关流式。软限位/禁区仍兜底。客户端封装见 [`arm_client`](../arm_client)（`stream_to` / `arm_cli stream`）。

## 客户端示例（mock 验证）

```bash
# 看状态
ros2 topic echo /arm/status
ros2 topic echo /arm/joint_states

# 回 home（双臂）
ros2 action send_goal /arm/go_home arm_interfaces/action/GoHome "{arm: both, timeout: 0}"

# 右臂移到一组关节角
ros2 action send_goal /arm/move_to_joints arm_interfaces/action/MoveToJoints \
  "{arm: right, joints_right: [42,4,15,-79,31,14,10], timeout: 0}" --feedback

# 软急停 / 清除
ros2 service call /arm/estop std_srvs/srv/Trigger
ros2 service call /arm/connect std_srvs/srv/Trigger

# 流式控制（Phase 2）：开启 → 高频发布关节目标 → 关闭
ros2 service call /arm/enable_streaming std_srvs/srv/SetBool "{data: true}"
ros2 topic pub -r 20 /arm/joint_command arm_interfaces/msg/JointCommand \
  "{arm: right, joints_right: [10,20,30,-40,50,-10,5]}"
ros2 service call /arm/enable_streaming std_srvs/srv/SetBool "{data: false}"
```

## 安全
- 软限位 / 工作空间禁区 / 单步钳位 / IK 越界检查全部沿用 `DualArm`，服务端兜底
- `/arm/estop` 立即停止下发新指令并中止当前动作（运动循环每拍检查；真机另有硬件急停）
- 与 `runner_dual` **互斥运行**（共享控制器单连接）
- 接真机：低 `VEL_RATIO`、现场监督、急停在手 —— 完整清单见 [`REAL_HARDWARE.md`](REAL_HARDWARE.md)

## 设计要点
- 运动动作串行（MutuallyExclusive 回调组），状态/服务并发（Reentrant），运动循环每拍释放 SDK 锁 → estop/状态在运动中可响应
- `MoveToJoints`/`GoHome`：下发非阻塞 `move_joints` + 轮询关节误差到位；`MoveToPose`：`move_to_ee_base`（含 Jetson 本地 IK）后按 `last_ik_joints` 轮询
- 流式：`/arm/joint_command`（独立 MutuallyExclusive 回调组）透传 `move_joints`；与离散动作的互斥靠 `_streaming` 门控（开流式拒动作、动作期间拒开流式），逻辑上不会同时下发
- mock 下 `move_joints` 即时更新关节、EE 为空：关节类动作秒到位，位姿动作下发即成功（无 IK 反馈）
