"""
camera_utils.py  —  RealSense 图像预处理工具

职责：
  将 RealSense 采集的彩色帧裁剪、缩放为模型输入尺寸（256×256 RGB）。

两种裁剪模式：
  1. center_crop（默认）：取中心正方形后 resize，不变形，简单可靠。
  2. fov_crop：按训练时 Isaac Sim 相机 FOV 倒推裁剪窗口，与训练保持一致。
     需要确认算法同事提供的相机 FOV 后使用。

TODO: 与算法同事确认 Isaac Sim 相机 FOV，更新 ISAAC_CAM_FOV_DEG。
"""

import cv2
import numpy as np

# ── 模型输入尺寸 ───────────────────────────────────────────────────────────────
MODEL_INPUT_SIZE = 256      # GR00T N1.5 输入 256×256

# ── RealSense 相机水平 FOV（单位：度）──────────────────────────────────────────
# 参考 Intel 规格书，实际值以标定结果为准
REALSENSE_FOV_DEG = {
    "D405": 87.0,   # D405 水平 FOV
    "D435": 69.0,   # D435 水平 FOV（彩色传感器）
}

# TODO: 确认 Isaac Sim 训练时使用的相机 FOV
ISAAC_CAM_FOV_DEG = 69.0   # 暂时与 D435 对齐，待算法同事确认


# ── 核心预处理函数 ─────────────────────────────────────────────────────────────

def center_crop_resize(frame_bgr: np.ndarray,
                       output_size: int = MODEL_INPUT_SIZE) -> np.ndarray:
    """
    中心正方形裁剪 → resize → RGB

    适用场景：快速验证，无需知道训练时的 FOV。

    Args:
        frame_bgr: RealSense 采集的 BGR 图像，任意分辨率
        output_size: 输出边长（像素），默认 256

    Returns:
        (output_size, output_size, 3) uint8 RGB 图像
    """
    h, w = frame_bgr.shape[:2]
    side = min(h, w)

    # 中心裁剪为正方形
    y0 = (h - side) // 2
    x0 = (w - side) // 2
    square = frame_bgr[y0:y0 + side, x0:x0 + side]

    # resize
    resized = cv2.resize(square, (output_size, output_size),
                         interpolation=cv2.INTER_LINEAR)
    # BGR → RGB
    return cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)


def fov_crop_resize(frame_bgr: np.ndarray,
                    cam_fov_deg: float,
                    target_fov_deg: float = ISAAC_CAM_FOV_DEG,
                    output_size: int = MODEL_INPUT_SIZE) -> np.ndarray:
    """
    按 FOV 比例裁剪 → resize → RGB

    使裁剪后的视野与 Isaac Sim 训练相机一致，减少 sim-to-real gap。

    Args:
        frame_bgr:      RealSense 采集的 BGR 图像
        cam_fov_deg:    当前相机水平 FOV（度），如 87.0（D405）/ 69.0（D435）
        target_fov_deg: 训练相机水平 FOV（度），默认 ISAAC_CAM_FOV_DEG
        output_size:    输出边长（像素），默认 256

    Returns:
        (output_size, output_size, 3) uint8 RGB 图像

    注意：
        若 target_fov >= cam_fov，无法通过裁剪获得更宽视野，退化为 center_crop。
    """
    if target_fov_deg >= cam_fov_deg:
        # 目标 FOV 比相机 FOV 还宽，无法裁剪，退化为中心裁剪
        return center_crop_resize(frame_bgr, output_size)

    h, w = frame_bgr.shape[:2]

    # 由水平 FOV 比例推算裁剪宽度
    # tan(fov/2) 与像素宽度成正比（针孔模型）
    ratio = (np.tan(np.radians(target_fov_deg / 2)) /
             np.tan(np.radians(cam_fov_deg / 2)))
    crop_w = int(w * ratio)
    crop_h = crop_w      # 正方形

    if crop_h > h:
        crop_h = h
        crop_w = h

    # 中心裁剪
    y0 = (h - crop_h) // 2
    x0 = (w - crop_w) // 2
    cropped = frame_bgr[y0:y0 + crop_h, x0:x0 + crop_w]

    resized = cv2.resize(cropped, (output_size, output_size),
                         interpolation=cv2.INTER_LINEAR)
    return cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)


def preprocess(frame_bgr: np.ndarray,
               cam_type: str = "D435",
               mode: str = "center_crop",
               output_size: int = MODEL_INPUT_SIZE) -> np.ndarray:
    """
    统一预处理入口

    Args:
        frame_bgr:   RealSense 采集的 BGR 图像
        cam_type:    相机型号字符串，"D405" 或 "D435"
        mode:        "center_crop"（默认）或 "fov_crop"
        output_size: 输出边长，默认 256

    Returns:
        (output_size, output_size, 3) uint8 RGB
    """
    if mode == "fov_crop":
        cam_fov = REALSENSE_FOV_DEG.get(cam_type, 69.0)
        return fov_crop_resize(frame_bgr, cam_fov,
                               target_fov_deg=ISAAC_CAM_FOV_DEG,
                               output_size=output_size)
    else:
        return center_crop_resize(frame_bgr, output_size)


# ── 验证工具 ───────────────────────────────────────────────────────────────────

def save_preview(frame_bgr: np.ndarray,
                 path: str,
                 cam_type: str = "D435",
                 mode: str = "center_crop"):
    """保存预处理前后对比图，用于验证裁剪效果"""
    processed = preprocess(frame_bgr, cam_type=cam_type, mode=mode)
    # 转回 BGR 保存
    processed_bgr = cv2.cvtColor(processed, cv2.COLOR_RGB2BGR)
    # 将原图 resize 到同等高度拼接对比
    h = processed_bgr.shape[0]
    scale = h / frame_bgr.shape[0]
    orig_resized = cv2.resize(frame_bgr,
                              (int(frame_bgr.shape[1] * scale), h))
    comparison = np.hstack([orig_resized, processed_bgr])
    cv2.imwrite(path, comparison)
    print(f"[camera_utils] 对比图已保存: {path}")
    print(f"  原图: {frame_bgr.shape[1]}×{frame_bgr.shape[0]}"
          f"  →  处理后: {processed_bgr.shape[1]}×{processed_bgr.shape[0]}"
          f"  模式: {mode}")


if __name__ == "__main__":
    # 快速自测：生成随机图像验证输出尺寸
    import sys
    print("=== camera_utils 自测 ===")
    for shape in [(480, 640, 3), (240, 424, 3)]:
        dummy = np.random.randint(0, 255, shape, dtype=np.uint8)
        for mode in ["center_crop", "fov_crop"]:
            for cam in ["D405", "D435"]:
                out = preprocess(dummy, cam_type=cam, mode=mode)
                assert out.shape == (256, 256, 3), f"输出尺寸错误: {out.shape}"
                assert out.dtype == np.uint8
        print(f"  输入 {shape[1]}×{shape[0]}  ✓")
    print("全部通过")
