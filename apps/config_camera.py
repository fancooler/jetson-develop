"""相机硬件配置（RealSense 三路：头部 D435 + 左/右腕 D405）

被以下文件直接 import：
  camera_publisher.py / camera_viewer.py / camera.py / runner_dual.py / test_camera.py

config.py 和 config_dual.py / config_single.py 通过 `from config_camera import ...`
re-export，保持 `config.CAM_WIDTH` 等旧引用不破。
"""

# ── 图像分辨率 / 帧率 ────────────────────────────────────────────────────────
# 训练数据 metadata.json: 640×480 @ 30fps
CAM_WIDTH  = 640
CAM_HEIGHT = 480
CAM_FPS    = 30

# ── RealSense 设备序列号 ─────────────────────────────────────────────────────
# 上机实测后写死，避免 enable_device(None) 自动枚举时角色混淆
# 2026-05-29 又换回旧硬件
# 2026-05-31 纠正左右腕接反（L/R 序列号对调）
HEAD_CAM_SERIAL    = "254622074992"  # D435 头部（前向）
WRIST_L_CAM_SERIAL = "260322273418"  # D405 左腕
WRIST_R_CAM_SERIAL = "260322272642"  # D405 右腕
