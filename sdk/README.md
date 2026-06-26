# 天机双臂 ROS2 推理 SDK（算法侧）

给算法同事：在**你自己的算法服务器**（或本机）上，通过 ROS2 连接 Jetson 上的
机械臂（和摄像头）服务，跑你的推理控制循环。

本 SDK 是**客户端侧**——机械臂/摄像头的服务跑在机器人的 Jetson 上，你这边只需要
本 SDK + ROS2 Humble，**不需要任何天机机械臂 SDK**。

```
[机器人 Jetson]  tj_marvin_arm 节点(机械臂)  +  camera_driver(相机)
        │   ROS2 话题 / 动作 / 服务（同一个 DDS domain，走网络）
        ▼
[你的算法服务器]  你的推理程序（= 本 SDK 的 demo/ + 你的模型）
```

---

## 目录结构

```
sdk/
├── README.md                          本文档
├── src/
│   ├── arm_interfaces/                机械臂 ROS2 接口定义（消息/服务/动作）→ nodes/src/arm_interfaces
│   ├── gripper_interfaces/            夹爪 ROS2 接口定义 → nodes/src/gripper_interfaces
│   └── arm_client/                    ArmClient 客户端库 + arm_cli 命令行 → nodes/src/arm_client
├── demo/
│   ├── runner_dual_ros2.py            推理主循环示例（策略填进 MockPolicy.infer()）→ apps/runner_dual_ros2.py
│   └── camera_viewer.py               三路相机田字格查看器 → apps/camera/camera_viewer_ros2.py
└── fastdds_unicast_profile.xml.template  网络多播不通时的单播发现模板（一般用不到）
```

> `src/` 和 `demo/` 下均为符号链接，指向 monorepo 内的唯一源文件。源码只维护一份，SDK 自动跟随更新。

---

## 一、前置条件（算法服务器）

- **Ubuntu 22.04 + ROS2 Humble**（建议 `ros-humble-desktop`，自带 `cv_bridge`）
  安装见 https://docs.ros.org/en/humble/Installation.html
- Python 依赖：`numpy`、`opencv-python`
  ```bash
  pip3 install numpy opencv-python
  ```
- 与机器人 Jetson **在同一网络**（同子网或路由可达）。

---

## 二、构建（一次性）

把 `sdk/` 当成一个 colcon 工作区构建：

```bash
cd ~/develop/sdk
source /opt/ros/humble/setup.bash
colcon build                 # 构建 arm_interfaces + gripper_interfaces + arm_client
source install/setup.bash    # 每开新终端都要 source（放进 ~/.bashrc 更省事）
```

---

## 三、网络配置（关键）

ROS2 靠 DDS 自动发现，**两端 `ROS_DOMAIN_ID` 必须一致**：

```bash
export ROS_DOMAIN_ID=1       # 连哪台机器人就设哪个值，以机器人方告知为准
```

当前机器人与 domain 对应关系：

| 机器人 | ROS_DOMAIN_ID |
|--------|--------------|
| robot1 | 1 |
| robot2 | 2 |

验证能发现机械臂节点：

```bash
ros2 node list               # 应看到 /tj_marvin_arm
ros2 topic list | grep /arm  # 应看到 /arm/joint_states /arm/status 等
```

- 能看到 → 网络 OK，直接进第五步。
- **看不到**（常见于某些 WiFi 丢多播）→ 用单播发现兜底：
  1. 编辑 `fastdds_unicast_profile.xml.template`，把 `JETSON_IP_REPLACE_ME` 改成
     机器人 Jetson 的 IP，存成 `fastdds_unicast_profile.xml`；
  2. `export FASTRTPS_DEFAULT_PROFILES_FILE=~/develop/sdk/fastdds_unicast_profile.xml`
  3. 同时需要机器人方在 Jetson 端也配一份指向你服务器 IP 的 profile（否则只有单向）。
  4. 有线接同一交换机则多播必通，最省事。

> 提示：`ROS_DOMAIN_ID` 不一致是"连不上"的头号原因；其次才是多播。

---

## 四、机器人 Jetson 侧提供的服务（由机器人方启动，你确认在跑即可）

机械臂节点（必须）：
```bash
# mock（无硬件，安全联调）：
ros2 launch tj_marvin_driver tj_marvin.launch.py use_mock:=true
# 真机：
ros2 launch tj_marvin_driver tj_marvin.launch.py use_mock:=false
```
夹爪节点（可选；不在线时 demo 自动降级，或用 `--no-gripper` 跳过）：
```bash
# mock：
ros2 launch xense_gripper_driver xense_gripper.launch.py mock_left:=true mock_right:=true
# 真机：
ros2 launch xense_gripper_driver xense_gripper.launch.py mock_left:=false mock_right:=false
```
摄像头（可选）：
```bash
ros2 launch camera_driver camera.launch.py
```
视触觉传感器（可选）：
```bash
ros2 launch tactile_driver tactile.launch.py
```

**接口清单**（你的程序消费这些）：

| 类型 | 名称 | 说明 |
|------|------|------|
| 话题(订阅) | `/arm/joint_states` | `sensor_msgs/JointState`，14 关节(left_joint1..7,right_joint1..7)，**弧度** |
| 话题(订阅) | `/arm/ee_pose_left` `/_right` | `geometry_msgs/PoseStamped`，base_link 系 |
| 话题(订阅) | `/arm/status` | `arm_interfaces/ArmStatus`：connected/estopped/busy/streaming/ctrl_mode |
| 话题(发布) | `/arm/joint_command` | `arm_interfaces/JointCommand`，**流式**关节目标(**度**)，需先开 enable_streaming |
| 动作 | `/arm/move_to_joints` `/move_to_pose` `/go_home` | 离散点到点运动 |
| 服务 | `/arm/connect` `/release` `/estop` `/enter_position_mode` | `std_srvs/Trigger` |
| 服务 | `/arm/set_ctrl_mode` | `arm_interfaces/SetCtrlMode` |
| 服务 | `/arm/enable_streaming` | `std_srvs/SetBool`，开/关流式（与离散动作互斥） |
| 话题(订阅) | `/arm/wrench_left` `/arm/wrench_right` | `geometry_msgs/WrenchStamped`，末端腕力六维力矩 [fx,fy,fz,tx,ty,tz] |
| 相机 | `/camera_front\|left_wrist\|right_wrist/color/image_raw[/compressed]` | RGB 图像 |
| 夹爪(发布) | `/gripper/command` | `gripper_interfaces/GripperCommand`：**流式**目标位置(mm)，side=left/right/both |
| 夹爪(订阅) | `/gripper/status` | `gripper_interfaces/GripperStatus`：双爪 position(mm)/force/温度/connected |
| 夹爪(动作/服务) | `/gripper/grip`(Grip 动作)、`/gripper/open\|close\|estop\|connect`(Trigger) | 阻塞到位 / 便捷开合急停 |
| 视触觉(订阅) | `/tactile/{left,right}/image_raw` | `sensor_msgs/Image` bgr8，校正图像 350×200 |
| 视触觉(订阅) | `/tactile/{left,right}/depth` | `sensor_msgs/Image` 32FC1，深度图 float32，单位 mm |
| 视触觉(订阅) | `/tactile/{left,right}/force` | `geometry_msgs/WrenchStamped`，六维合力 [fx,fy,fz,tx,ty,tz] |
| 视触觉(订阅) | `/tactile/{left,right}/force_map` | `sensor_msgs/Image` bgr8，35×20 力分布图 |
| 视触觉(订阅) | `/tactile/{left,right}/marker` | `sensor_msgs/Image` 32FC2，35×20 切向位移 |
| 视触觉(服务) | `/tactile/calibrate` | `std_srvs/Trigger`，重置参考图像（无接触时调用）|

---

## 五、跑示例 demo

```bash
cd ~/develop/sdk
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=1   # robot1=1, robot2=2；连哪台设哪个

# 没有真实相机：本进程发合成图，链路照样跑通
python3 demo/runner_dual_ros2.py --mock-cameras

# 接真实 camera_driver；跨网络订阅图像建议用压缩流省带宽
python3 demo/runner_dual_ros2.py --compressed

# 其它选项
python3 demo/runner_dual_ros2.py --help
# --rate 20  --max-cycles 100  --arm both|left|right  --no-gripper
```

预期：打印 connect/go_home/enable_streaming 成功 → 进入控制循环 → 每 10 拍打印一次
耗时与关节角 → Ctrl-C 退出并给计时统计。

### 实时查看相机

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
python3 demo/camera_viewer.py                  # 原始流（同机/局域网）
python3 demo/camera_viewer.py --compressed     # 跨 WiFi 用压缩流（推荐）
python3 demo/camera_viewer.py --snapshot ./snaps   # headless 服务器：存图不开窗
```

田字格显示 front / left_wrist / right_wrist；窗口内按 `q`/`ESC` 退出。
**`camera_viewer.py` 只用标准 `sensor_msgs`，不需要 build SDK 工作区**，
`source /opt/ros/humble/setup.bash` + 设 `ROS_DOMAIN_ID` 即可。

---

## 六、接入你自己的算法（核心）

打开 `demo/runner_dual_ros2.py`，找到 **`MockPolicy.infer()`**，把它换成你的推理：

- **输入** `obs`（字典）：
  - `obs['images']`：`{'front','left_wrist','right_wrist'}` → 每个是 `HxWx3` RGB `uint8` numpy 数组
  - `obs['joints_deg']`：`{'left':[7], 'right':[7]}`，当前关节角，**度**
  - `obs['ee']`：`{'left','right'}` → `PoseStamped` 或 `None`（mock 无 FK）
  - `obs['gripper_mm']`：`{'left':mm|None, 'right':mm|None}`，双爪当前位置
- **输出**：`{'left':[7], 'right':[7]}` **绝对**关节角目标(**度**)，外加
  `'gripper':{'left':mm, 'right':mm}` 夹爪目标位置（约 2=闭合 ~ 85=张开）

其余（ROS2 订阅、流式下发、定频、统计）都不用改。把模型加载放在 `main()` 里，循环里调 `infer()` 即可。

> 控制范式：示例用**流式**（`enable_streaming` + 高频 `stream_joints`），对应
> 20Hz 连续控制。若你想点到点，用 `arm.move_to_joints(...)` 等离散动作（**流式开启时离散动作会被拒，二者互斥**）。

---

## 七、ArmClient API 速查

```python
from arm_client import ArmClient
arm = ArmClient()
arm.wait_for_servers(timeout=10.0)

# 连接 / 模式
arm.connect();  arm.release();  arm.estop()
arm.enter_position_mode()
arm.set_ctrl_mode('position')      # 'position'|'impedance'

# 离散运动（阻塞；joints 单位=度）
arm.go_home('both', left_variant=0)
arm.move_to_joints('right', right=[10,20,30,-40,50,-10,5])
arm.move_to_pose('right', position=[0.3,-0.2,0.4], rpy=[0,0,0])   # base_link, m+rad

# 流式（20Hz 连续控制）
arm.enable_streaming(True)
arm.stream_joints('both', left=[...7 度...], right=[...7 度...])
arm.stream_to('right', target_right=[...], duration=2.0, rate=20)  # 插值
arm.enable_streaming(False)

# 状态
arm.status            # ArmStatus
arm.joints_dict()     # {'left':[7 rad],'right':[7 rad]}
arm.ee_pose('left')   # PoseStamped 或 None
arm.shutdown()
```

命令行：`ros2 run arm_client arm_cli --help`

---

## 八、单位与约定（容易踩）

- **关节命令（move_to_joints / joint_command / stream）：度**；**joint_states 反馈：弧度**。
- 位姿：`base_link` 坐标系，位置米、姿态四元数（`move_to_pose` 也可传 `rpy` 弧度）。
- 关节顺序：`left_joint1..7` 然后 `right_joint1..7`（共 14）。
- 流式与离散动作**互斥**；`/arm/estop` 立即停指令并关流式，`connect` 清除急停。
- 服务端有软限位/工作空间禁区/IK 越界兜底，但**真机仍需现场监督、急停在手**。

---

## 九、故障排查

| 现象 | 排查 |
|------|------|
| `ros2 node list` 空 / 看不到 `/tj_marvin_arm` | ① `ROS_DOMAIN_ID` 两端是否一致 ② 是否同网络 ③ WiFi 丢多播 → 用单播 profile（第三步） |
| `ModuleNotFoundError: arm_client` | 没 `source ~/develop/sdk/install/setup.bash` |
| `wait_for_servers` 超时 | 机械臂节点没在 Jetson 跑，或 domain/网络问题 |
| 相机一直黑帧 | camera_driver 没跑；无相机就加 `--mock-cameras`；跨网络建议 `--compressed` |
| 动作返回"流式模式开启中" | 先 `arm.enable_streaming(False)` 再发离散动作 |
| `cv_bridge` / `cv2` 导入失败 | 装 `ros-humble-desktop` + `pip3 install opencv-python` |
| `/tactile/*/image_raw` 无数据 | ① tactile_driver 是否在跑（`ros2 node list`） ② 视触觉传感器网线/上电 ③ `ros2 service call /tactile/calibrate std_srvs/srv/Trigger {}` 校准 |
