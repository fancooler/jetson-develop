"""topics.py — ROS2 摄像头 topic 名称 + frame_id 合约

这是发布端（本包 camera_publisher_node）与订阅端
（jetson-work 仓的 runner_dual.py / camera_viewer.py）之间的唯一合约。
两边必须保持一致；改 topic 名时两处都要改。

命名约定对齐官方 realsense2_camera ROS 包：
  /<camera_namespace>/color/image_raw
未来若改用官方包驱动，订阅端代码无需改 topic 名。
"""

# ── Topic 名称（彩色图像）────────────────────────────────────────────────────
TOPIC_FRONT       = "/camera_front/color/image_raw"
TOPIC_LEFT_WRIST  = "/camera_left_wrist/color/image_raw"
TOPIC_RIGHT_WRIST = "/camera_right_wrist/color/image_raw"

# ── ROS 坐标系 frame_id（消息 header 用）──────────────────────────────────────
# 仿 realsense2_camera 的 *_color_optical_frame 命名
FRAME_ID_FRONT       = "camera_front_color_optical_frame"
FRAME_ID_LEFT_WRIST  = "camera_left_wrist_color_optical_frame"
FRAME_ID_RIGHT_WRIST = "camera_right_wrist_color_optical_frame"

# ── 角色 → topic / frame_id 映射 ──────────────────────────────────────────────
ROLES = ('front', 'left_wrist', 'right_wrist')

TOPICS_BY_ROLE = {
    'front':       TOPIC_FRONT,
    'left_wrist':  TOPIC_LEFT_WRIST,
    'right_wrist': TOPIC_RIGHT_WRIST,
}

# ── 压缩图像 topic（JPEG，远端 viewer 用，省带宽）─────────────────────────────
# ROS image_transport 标准约定：<base_topic>/compressed
TOPICS_COMPRESSED_BY_ROLE = {
    role: f"{TOPICS_BY_ROLE[role]}/compressed" for role in ROLES
}

FRAME_IDS_BY_ROLE = {
    'front':       FRAME_ID_FRONT,
    'left_wrist':  FRAME_ID_LEFT_WRIST,
    'right_wrist': FRAME_ID_RIGHT_WRIST,
}
