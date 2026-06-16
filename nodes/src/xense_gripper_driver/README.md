# xense_gripper_driver

Xense 双夹爪的 ROS2 驱动（**独立于机械臂节点**）。薄包装 jetson-work 仓
`app/gripper.py` 的 `XenseGripper`（真机）/ `MockGripper`（无硬件），对外实现通用
[`gripper_interfaces`](../gripper_interfaces)。

> ⚠️ 与本仓已有的 [`gripper_driver`](../gripper_driver) **不是同一个东西**：那个是
> 同事 lsj 的 **RS485 串口**夹爪驱动（`/dev/ttyUSB0`、`Float32MultiArray`）。本包是
> **Xense TCP/网络**夹爪（按 MAC 寻址），二者硬件与协议完全不同，互不干扰。

## 与机械臂节点的关系

**完全独立、可并行运行**。Xense 夹爪是网络设备（MAC→IP），与天机臂控制器无共享资源：
- 命名空间分开：机械臂 `/arm/*`，夹爪 `/gripper/*`。
- 两个节点各自启动、各自 estop。系统级急停应**同时**调 `/arm/estop` 和 `/gripper/estop`。
- 推理 runner 同时用 [`arm_client`](../arm_client) 和夹爪接口即可（见 jetson-work `runner_dual.py` 里 arm+gripper 的配合）。

## 运行

```bash
# 默认两爪都 mock（不碰硬件）
ros2 launch xense_gripper_driver xense_gripper.launch.py

# 接真机（按需单/双爪；app_dir 指向含 gripper.py 的目录）
ros2 launch xense_gripper_driver xense_gripper.launch.py mock_left:=false mock_right:=false
# ThinkBook 上跑 mock 联调（app 在 jetson-work 下）：
ros2 launch xense_gripper_driver xense_gripper.launch.py app_dir:=~/work/jetson-work/app
```

参数：`mock_left`/`mock_right`(true) `app_dir`(~/work/app) `auto_connect`(true)
`publish_rate`(25.0)；节点内另有 `mac_left`/`mac_right` `vmax`(80) `fmax`(27) `tol`(2.0)
`default_timeout`(10)。

## 接口

**发布**
| 话题 | 类型 | 说明 |
|------|------|------|
| `/gripper/status` | `gripper_interfaces/GripperStatus` | 两爪 connected/position(mm)/velocity/force/temperature/moving + estopped |

**订阅**
| 话题 | 类型 | 说明 |
|------|------|------|
| `/gripper/command` | `gripper_interfaces/GripperCommand` | **流式**目标位置(mm)，side=left/right/both，BestEffort/depth1 |

**动作**
| 名称 | 类型 | 说明 |
|------|------|------|
| `/gripper/grip` | `Grip` | side + 目标位置(mm)，阻塞到位、带 max_error_mm 反馈 |

**服务**：`/gripper/connect` `/gripper/estop` `/gripper/open` `/gripper/close`（`std_srvs/Trigger`）

位置量程：约 `2mm`（闭合，`POS_CLOSE`，防顶死过流）~ `85mm`（张开，`POS_OPEN`），来自 `gripper.py`。

## 客户端示例（mock 验证）

```bash
ros2 topic echo /gripper/status
ros2 service call /gripper/open  std_srvs/srv/Trigger
ros2 service call /gripper/close std_srvs/srv/Trigger
# 流式（20Hz 控制循环用）：
ros2 topic pub -r 20 /gripper/command gripper_interfaces/msg/GripperCommand "{side: both, position: 42.5, max_effort: 0}"
# 阻塞到位：
ros2 action send_goal /gripper/grip gripper_interfaces/action/Grip "{side: left, position: 10, max_effort: 0, timeout: 0}" --feedback
ros2 service call /gripper/estop std_srvs/srv/Trigger   # 软急停；connect 清除
```

## 设计要点

- 单节点管左右两爪，每爪可独立 `mock`（对应 config_dual 的 `GRIPPER_MOCK` / `GRIPPER_MOCK_RIGHT`，当前硬件故障默认都 mock）。
- **每个夹爪一把锁，串行化其所有 SDK 访问**（状态轮询 vs 下发不会并发调 SDK）——直接规避
  `xensegripper` 底层非线程安全时的偶发报错（`test_gripper.py` 后台状态线程 + 主线程下发并发就属此类）。
- 动作串行（MutuallyExclusive），状态/服务/流式并发（Reentrant/独立组）。
- `max_effort` 字段已预留，但当前用节点 `fmax` 参数（`gripper.py` 的 `set_position` 不收 per-call fmax）；
  如需 per-command 力控、以及在 `/gripper/status` 里上报 velocity/force/temperature，
  需给 `app/gripper.py` 加一个 `get_status()`（包 SDK `get_gripper_status()`）——本节点已用
  `getattr` 探测，有则自动上报，无则填 NaN。这对排查真机偶发报错（看力/温度）也有用。
