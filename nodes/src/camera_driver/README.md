# camera_driver

三路 RealSense 相机的 ROS2 (Humble) 驱动包：把**头部 D435 + 左/右腕 D405** 的彩色图像
发布成标准 ROS2 topic，并附带一个田字格查看器。

本包从 `jetson-work` 仓的 `app/camera_publisher.py` + `app/camera_viewer.py`
迁移而来（独立脚本 → 正规 ROS2 包）。

## 两个节点

| 节点 | 可执行名 | 作用 | 运行位置 |
|------|----------|------|----------|
| `camera_publisher` | `camera_node`   | 独占三路 RealSense，发布 `image_raw` + `compressed` | **接相机的 Jetson** |
| `camera_viewer`    | `camera_viewer` | 订阅压缩流，田字格 `cv2.imshow` 显示 | 任意带显示器 + ROS2 的机器（Jetson 本机 / ThinkBook 远端） |

## 发布的 topic 合约（见 `camera_driver/topics.py`）

| role | `image_raw`（`sensor_msgs/Image`, bgr8） | `compressed`（`sensor_msgs/CompressedImage`, jpeg） |
|------|------------------------------------------|------------------------------------------------------|
| front       | `/camera_front/color/image_raw`        | `/camera_front/color/image_raw/compressed`        |
| left_wrist  | `/camera_left_wrist/color/image_raw`   | `/camera_left_wrist/color/image_raw/compressed`   |
| right_wrist | `/camera_right_wrist/color/image_raw`  | `/camera_right_wrist/color/image_raw/compressed`  |

- 命名对齐官方 `realsense2_camera`，未来若换官方驱动，订阅端无需改 topic 名。
- QoS：`BestEffort` + `KEEP_LAST` depth=1（只关心最新帧，丢帧不重传）。
- `frame_id`：`camera_<role>_color_optical_frame`。

## 编译

```bash
# Jetson 上是 ~/ros2_ws，ThinkBook 上是 ~/work/jetson-ros2
cd ~/ros2_ws
colcon build --packages-select camera_driver
source install/setup.bash
```

## 使用

### 1. 启动发布端（在接相机的 Jetson 上）

```bash
ros2 launch camera_driver camera.launch.py
```

覆盖示例：

```bash
# 以其它机器人的配置启动（序列号从 robots.yaml 按 ROBOT_ID 取）
ROBOT_ID=robot2 ros2 launch camera_driver camera.launch.py

# 直接 run 并临时改帧率 / 关掉压缩流（绕过 launch / 注册表）
ros2 run camera_driver camera_node --ros-args -p fps:=15 -p publish_compressed:=false
```

验证：

```bash
ros2 topic list | grep camera
ros2 topic hz /camera_front/color/image_raw      # 应 ~30Hz
```

### 2. 启动查看器（本机 Jetson 带屏，或 ThinkBook 远端）

```bash
ros2 run camera_driver camera_viewer             # 按 q / ESC 退出
```

## 参数

**`camera_node`（发布端）** — 序列号默认值仅作兜底；正常由 launch 从 `config/robots.yaml` 按 `ROBOT_ID` 注入，见[多机器人部署](#多机器人部署)。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `head_serial`        | `254622074992` | 头部 D435 序列号 |
| `left_wrist_serial`  | `260322273418` | 左腕 D405 序列号 |
| `right_wrist_serial` | `260322272642` | 右腕 D405 序列号 |
| `width` / `height`   | `640` / `480`  | 彩色流分辨率 |
| `fps`                | `30`           | 帧率 |
| `jpeg_quality`       | `80`           | 压缩流 JPEG 质量 0-100 |
| `publish_compressed` | `true`         | 是否额外发 `.../compressed` |

**`camera_viewer`（查看端）**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `width` / `height` | `640` / `480` | 每个窗格尺寸（占位帧/网格） |

## 多机器人部署

每台机器人的相机序列号、`ROS_DOMAIN_ID` 等差异，集中在 `config/robots.yaml`，按 **`ROBOT_ID`** 选择。共享代码与 topic 名对所有机器人保持不变。

### 加一台新机器人
1. 在 `config/robots.yaml` 的 `robots:` 下追加一块，填 3 个相机序列号 + **唯一**的 `domain_id`：
   ```yaml
   robot2:
     domain_id: 2
     cameras:
       head_serial:        "..."
       left_wrist_serial:  "..."
       right_wrist_serial: "..."
   ```
   （查序列号：`rs-enumerate-devices -s`，或看启动日志 `[cam/<role>] 已启动 serial=...`）
2. 提交、push，在该机 `colcon build` 同步。
3. 在该机 Jetson 的 `~/.bashrc` 末尾加（放在 source ROS 之后）：
   ```bash
   export ROBOT_ID=robot2
   source ~/ros2_ws/install/camera_driver/share/camera_driver/config/robot_env.sh
   ```
   `robot_env.sh` 会按 `ROBOT_ID` 从注册表导出对应 `ROS_DOMAIN_ID`（domain 单一来源，不会两处写串）。

### domain 隔离
每台机器人用**唯一** `ROS_DOMAIN_ID`，DDS 流量天然隔离——即使多台同网络、topic 名都叫 `/camera_front/...` 也不串台，`runner_dual` 等订阅端无需改动。

### 看某台机器人（开发机 / viewer 端）
开发机想看哪台，就把 `ROBOT_ID` 设成那台再 source，使 `ROS_DOMAIN_ID` 与目标机一致：
```bash
export ROBOT_ID=robot2
source ~/work/jetson-ros2/install/camera_driver/share/camera_driver/config/robot_env.sh
ros2 run camera_driver camera_viewer
```

> launch 启动会打印 `ROBOT_ID / domain_id / 各路序列号`，并在 `ROS_DOMAIN_ID` 与注册表不一致时醒目告警。

## 远端查看的前提

- viewer 端需安装 ROS2 Humble + `cv2`（不需要 cv_bridge，viewer 用 `cv2.imdecode`）。
- viewer 与发布端 **`ROS_DOMAIN_ID` 一致**（由 `ROBOT_ID` + `robot_env.sh` 决定，见上）。
- 两台机器在同一局域网、DDS 能互相发现（WiFi 多播常被 AP 挡，需单播 profile）。远端只订压缩流，带宽友好。

## 与 `runner_dual` 的关系

`jetson-work` 仓的 `runner_dual.py` 通过订阅同名 `image_raw` topic 取帧推理。
本包只负责发布，二者**仅通过 topic 解耦、互不 import**。运行顺序：先启动本包发布端，
再跑 `runner_dual.py`。

## 排错

| 现象 | 处理 |
|------|------|
| 相机启动失败 / 设备被占用 | `pkill -f camera_node`，并确认旧的 `camera_publisher.py` 没在抢占 RealSense |
| viewer 一直 `waiting ...` | 检查发布端在跑、`ROS_DOMAIN_ID` 一致、`ros2 topic list` 能看到 topic |
| 无窗口弹出 | 确认有图形界面（`echo $DISPLAY`）；无头 SSH 无法用 viewer，改用 `ros2 topic hz` 验证 |
