# 接真机操作与安全检查清单（tj_marvin_driver）

> ⚠️ **真机会动、会伤人/损设备。** 任何一次接真机：现场监督、**硬件急停在手**、
> 先低速、清空机械臂运动范围内的人和物。本清单基于 jetson-work `app/arm_utils.py`
> 的 `DualArm` 与 `config*.py` 实际行为整理。**只能在 Jetson 跑真机**（天机 SDK 是
> aarch64 `.so`，且需有线连控制器）。

---

## 0. ⚠️ 当前最大的坑：控制器 IP 与 WiFi 同网段冲突（2026-05-31 新增）

- 天机控制器 `ROBOT_IP = 192.168.1.190`（`config.py`），**有线**连接。
- 但现在 Jetson/ThinkBook 的 **WiFi 也在 `192.168.1.0/24`**（Jetson `192.168.1.34`）。
- 实测 `ip route get 192.168.1.190` → **走 WiFi `wlP5p1s0`**，不是有线口。
  接上有线控制器后会有两个接口抢同一 /24 → 路由走错网口 → **连不上控制器**
  （`_do_connect` 报 "连接失败，请检查网线和 IP" 或 "UDP 数据帧未更新"）。

  > 以前 WiFi 是 `10.163.x`、控制器 `192.168.1.x` 不冲突；是今天换网段才出现的。

**接控制器前必须先解决其一：**
1. （推荐）把 Jetson/ThinkBook 的 WiFi 换到非 `192.168.1.x` 网段；或
2. 给控制器加 **/32 主机路由**走有线口，优先级高于 WiFi 默认路由：
   ```bash
   # 先确认有线网口名（接上网线后 ip -br link 看，常见 eth0 / enP*）
   sudo ip addr add 192.168.1.100/24 dev <有线口>      # Jetson 有线口配同段地址
   sudo ip link set <有线口> up
   sudo ip route add 192.168.1.190/32 dev <有线口>     # 强制 .190 走有线
   ip route get 192.168.1.190                          # 应显示 dev <有线口>
   ```
   注意：Jetson 当前**没有**有线口在用（只有 `lo` + WiFi），接线后需手动配置并 up。

---

## 1. 接机前检查清单

- [ ] **互斥**：确认 `runner_dual` / `infer_dual` / 其它占用控制器的程序**没在运行**。
      控制器是**单连接**（`DualArm` 双臂共享一个 SDK 连接），两个程序同时连会冲突。
      `pgrep -af "runner_dual|infer_dual|arm_node"` 检查。
- [ ] **物理**：机械臂周围无人无障碍；硬件急停按钮在手且测试可按下。
- [ ] **网络**：解决第 0 节的网段冲突后，`ping -c2 192.168.1.190` 通。
- [ ] **防火墙**：控制器靠 UDP 回传状态帧，`_do_connect` 会校验帧更新；
      若 "UDP 数据帧未更新" 检查 Jetson 防火墙（`sudo ufw status`，必要时放行）。
- [ ] **SDK/配置**：`~/work/TJ_marvin/TJ_FX_ROBOT_CONTRL_SDK-master/ccs_m6_40.MvKDCfg` 存在（已确认在）。
- [ ] **速度**：`config.py` 的 `VEL_RATIO=10 ACC_RATIO=10 HOME_VEL_RATIO=10`（10%）。
      首次接机/到新 HOME 时建议**更低**（改 5）。改 `config.py` 会同时影响 runner_dual。
- [ ] **控制模式**：`config.CTRL_MODE`（`'position'` 默认硬跟随 / `'impedance'` 笛卡尔阻抗、
      碰阻力顺应更安全）。connect 时按此值预热，**运行时不可切**（Phase 1）。

---

## 2. 启动与首次运动顺序

```bash
# Jetson 上（已 source ROS + ros2_ws/install，ROBOT_ID=robot1）
ros2 launch tj_marvin_driver tj_marvin.launch.py use_mock:=false
```

- `use_mock:=false` → 用真机 `DualArm`。`auto_connect:=true`（默认）启动即 `connect()`：
  - connect **只设控制模式 + 预热，不运动**（相对安全）；但会 `sleep(1s)` 等控制器异步切模式。
  - 看到日志 `位置跟随模式` / `控制模式预热完成 (position)` / `双臂就绪` 才算连上。
- **首次运动务必从最小、低速开始**，全程手放急停：
  ```bash
  # ThinkBook 或 Jetson 客户端
  ros2 run arm_client arm_cli status                 # 确认 connected=true
  ros2 run arm_client arm_cli home both --variant 0 --feedback   # 先回 HOME
  ```
- 确认 HOME 动作正常、无异响后，再尝试 `joints` / `pose` 小幅运动。

---

## 3. 真机特有行为 / 坑（mock 下不会遇到）

| 现象 | 原因 | 处理 |
|------|------|------|
| connect 后立刻发指令"咔哒一声不动" | 控制器模式切换是异步的，指令被静默丢弃 | connect 内已 `sleep(1s)` 预热；勿在 connect 刚返回就猛发指令 |
| **go_home 后再 move 不动**（命令返回但臂不动） | go_home 完成后 SDK `arm_state` 可能回到 0（下伺服），`set_joint_position_cmd` 被静默拒 | **先调 `/arm/enter_position_mode`** 再发运动（servo_reset 7 轴 + 重设位置模式 + sleep1s）。`arm_cli enter-pos` |
| "连接失败，请检查网线和 IP" | 网线/IP/网段冲突（见第 0 节） | 查路由与网线 |
| "UDP 数据帧未更新，请检查防火墙" | TCP 连上但 UDP 状态帧收不到 | 查防火墙 |
| EE 位姿能读到（mock 下是 None） | 真机有 FK | `/arm/ee_pose_*` 才有有效数据 |

---

## 4. 安全层（服务端兜底，但**不能替代现场监督**）

`DualArm` 内部、arm_node 调用前都有这些拦截（`safe=True` 时）：
- **关节软限位** `_JOINT_LIMITS_DEG`（度）：J1±170 / J2[-100,120] / J3±170 / J4[-145,60] / J5±170 / J6±60 / J7±90。超限拒发。
- **工作空间禁区** `config_dual.WORKSPACE_FORBIDDEN`：当前含柱子区 `X[0.40,0.60] Y[-0.10,0.10] Z[0,1.0]` m（base_link）。EE 目标落入则拒。
- **IK 越界/无解检查** + 单步增量 **clamp**（`clamp_delta_pos/rpy`）。
- `MoveToPose` 的 `safe` 字段透传到这里；**`arm_cli pose --unsafe` 会跳过软限位检查，真机慎用**。

---

## 5. 急停

- **软急停**：`/arm/estop`（`arm_cli estop`）→ 立即停止下发新指令并中止当前动作
  （运动循环每拍检查）。清除：`/arm/connect`（`arm_cli connect`）。
- **硬件急停**：始终以物理急停按钮为最终保障，软急停不能替代。

---

## 6. 收尾

- 正常关闭：`Ctrl-C` 停 launch。节点 `main()` 在真机模式下会 `release()`（disable 双臂 + release_robot）。
- 或显式 `/arm/release`（`arm_cli release`）后再关。
- 关掉 arm_node 后才能再跑 `runner_dual`（单连接互斥）。

---

## 附：关键参数位置

| 参数 | 值 | 文件 |
|------|----|----|
| `ROBOT_IP` | `192.168.1.190` | `app/config.py` |
| `VEL_RATIO` / `ACC_RATIO` / `HOME_VEL_RATIO` | `10` / `10` / `10` | `app/config.py` |
| `REACH_TOL`（SDK 到位°） | `0.5` | `app/config.py` |
| `CTRL_MODE` / `IMP_K` / `IMP_D` | `position` / … | `app/config.py` |
| `HOME_JOINTS_*` | 见文件 | `app/config_dual.py` |
| `WORKSPACE_FORBIDDEN` | 柱子区 | `app/config_dual.py` |
| 节点 `reach_tol_deg`（动作轮询°）/ `default_timeout` | `1.0` / `30.0` | launch/节点参数 |
