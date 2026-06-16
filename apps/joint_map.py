"""joint_map.py — 训练(URDF/模型) 关节空间 ↔ 天机 SDK 关节空间 的逐关节线性映射

背景（见 memory: sim-real-coord-mismatch）：
  GR00T 模型的 action / state 关节角是 URDF/Isaac 约定；真机天机 SDK 是另一套
  约定（部分关节符号翻转 + 零偏）。两者直接互喂会导致真机物理姿态 ≠ 仿真，
  这是「sim 行真机不行」的主因(B)。

逐关节线性映射（per-arm, per-joint，由 test/calib_joints.py 离线标定）：
    q_sdk[i]  = sign[i] * q_urdf[i] + offset[i]        # urdf_to_sdk
    q_urdf[i] = (q_sdk[i] - offset[i]) / sign[i]       # sdk_to_urdf   (sign ∈ {+1,-1})

只映射 7 个手臂关节，夹爪/IK 解/SDK FK 不经过本模块（它们本就是物理 SDK 空间）。

在管线中的三个边界（全部经本模块）：
  1. action 下发前： urdf_to_sdk  —— infer_dual.extract_*_arm_cmd
  2. state 喂模型前：sdk_to_urdf  —— infer_dual._build_joint_pos_18
  3. go_home 前：    urdf_to_sdk  —— arm_utils._go_home_arms（HOME_JOINTS 是 URDF）

开关（两道，任一不满足即恒等映射，保留改造前行为，便于 A/B 对照）：
  - config_dual.USE_JOINT_MAP = False  → 全程恒等 + 一次性 WARNING
  - test/joint_map.json 不存在          → 全程恒等 + 一次性 WARNING（标定前的状态）

映射表来源：test/joint_map.json（`python3 test/calib_joints.py --save` 生成）：
  {"right": {"sign": [..7], "offset": [..7]}, "left": {"sign": [..7], "offset": [..7]}}

也可用环境变量 JOINT_MAP_FILE 覆盖映射表路径（调试/对比不同标定结果时方便）。
"""
import os
import json
import logging

import numpy as np

logger = logging.getLogger(__name__)

ARMS = ('right', 'left')
_NUM_JOINTS = 7

_DEFAULT_JSON = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'test', 'joint_map.json')

# 惰性单例：进程内首次使用时加载一次（runner/replay 每次是独立进程）。
_MAP = None        # {'right': {'sign': ndarray[7], 'offset': ndarray[7]}, 'left': {...}} 或 None
_LOADED = False    # 是否已尝试加载


def _map_path() -> str:
    return os.environ.get('JOINT_MAP_FILE', _DEFAULT_JSON)


def _config_enabled() -> bool:
    """读 config_dual.USE_JOINT_MAP（缺省视为启用）。延迟导入避免循环依赖。"""
    try:
        import config_dual as cfg
        return bool(getattr(cfg, 'USE_JOINT_MAP', True))
    except Exception as e:   # 配置不可用时按启用处理，但留痕
        logger.debug(f"读取 USE_JOINT_MAP 失败（按启用处理）: {e}")
        return True


def _load() -> None:
    """加载映射表到 _MAP；任一开关不满足则 _MAP=None（恒等）。"""
    global _MAP, _LOADED
    _LOADED = True
    _MAP = None

    if not _config_enabled():
        logger.warning("USE_JOINT_MAP=False → 关节映射禁用（恒等），URDF/SDK 直通")
        return

    path = _map_path()
    if not os.path.exists(path):
        logger.warning(
            f"未找到关节映射表 {path} → 暂用恒等映射。"
            f"请先在 Jetson 跑 `python3 test/calib_joints.py --save` 生成。")
        return

    try:
        with open(path) as f:
            raw = json.load(f)
        m = {}
        for arm in ARMS:
            sign = np.asarray(raw[arm]['sign'], dtype=np.float64)
            off = np.asarray(raw[arm]['offset'], dtype=np.float64)
            if sign.shape != (_NUM_JOINTS,) or off.shape != (_NUM_JOINTS,):
                raise ValueError(f"{arm} sign/offset 维度应为 {_NUM_JOINTS}")
            if not np.all(np.isin(sign, (1.0, -1.0))):
                raise ValueError(f"{arm} sign 须全为 ±1，实为 {sign.tolist()}")
            m[arm] = {'sign': sign, 'offset': off}
    except Exception as e:
        logger.error(f"解析关节映射表 {path} 失败 → 退回恒等映射: {e}")
        return

    _MAP = m
    logger.info(f"已加载关节映射表 {path}")
    for arm in ARMS:
        logger.info(f"  {arm:5}: sign={m[arm]['sign'].astype(int).tolist()} "
                    f"offset={np.round(m[arm]['offset'], 2).tolist()}")


def _wrap180(a):
    """把角度(度)绕回 (-180, 180]，避免 offset=±180/±90 把关节推出 ±180 表示范围。
    同一物理关节角 q 与 q±360 等价；SDK 期望主值表示，限位检查也在 ±180 内。"""
    return ((np.asarray(a, dtype=np.float64) + 180.0) % 360.0) - 180.0


def _ensure() -> None:
    if not _LOADED:
        _load()


def reload() -> None:
    """重新加载映射表 / 重新读开关（标定生成 json 后、或运行时改了 config 时调用）。"""
    global _LOADED
    _LOADED = False
    _load()


def is_active() -> bool:
    """当前是否真正在做非恒等映射（True = 已加载有效映射表且开关开）。"""
    _ensure()
    return _MAP is not None


def urdf_to_sdk(arm: str, q_deg) -> np.ndarray:
    """URDF/模型关节角(度) → 天机 SDK 关节角(度)。映射未就绪时恒等返回。

    q_sdk = sign * q_urdf + offset
    """
    _ensure()
    q = np.asarray(q_deg, dtype=np.float64)
    if _MAP is None:
        return q.copy()
    p = _MAP[arm]
    return _wrap180(p['sign'] * q + p['offset'])


def sdk_to_urdf(arm: str, q_deg) -> np.ndarray:
    """天机 SDK 关节角(度) → URDF/模型关节角(度)。映射未就绪时恒等返回。

    q_urdf = (q_sdk - offset) / sign   （sign ∈ {+1,-1}，等价于乘 sign）
    """
    _ensure()
    q = np.asarray(q_deg, dtype=np.float64)
    if _MAP is None:
        return q.copy()
    p = _MAP[arm]
    return _wrap180((q - p['offset']) / p['sign'])


if __name__ == '__main__':
    # 快速自检：打印当前映射状态 + 一组往返一致性
    logging.basicConfig(level=logging.INFO,
                        format='%(levelname)-5s %(name)s: %(message)s')
    print(f"映射表路径: {_map_path()}")
    print(f"is_active : {is_active()}")
    for arm in ARMS:
        q = np.array([10., -20., 30., -40., 50., -15., 25.])
        s = urdf_to_sdk(arm, q)
        back = sdk_to_urdf(arm, s)
        print(f"[{arm}] urdf={q}\n      sdk ={np.round(s, 2)}\n      back={np.round(back, 2)}  "
              f"round-trip max err={np.max(np.abs(back - q)):.2e}")
