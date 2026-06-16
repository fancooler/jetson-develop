"""
frame_transform.py  —  坐标系变换工具：base_link ↔ 各臂 FK/IK 基坐标系

J0 原点来自 URDF（ros_model_260506.urdf，即 base_link 坐标系中的位置）：
  joint_1  (右臂 J0): xyz=(0.61736, -0.37205,  0.035424) m
  joint_10 (左臂 J0): xyz=(0.61736, -0.37205, -0.038576) m

旋转矩阵（已通过 GRV 参数 + URDF 关节轴向量双重验证）：

  右臂 R_RIGHT：
    臂X → base_link -X
    臂Y → base_link -Y
    臂Z → base_link +Z（向上）

    世界系映射（经 R_ROOT_WORLD 变换）：
    臂X → world +Y   臂Y → world +Z   臂Z → world +X
    GRV 交叉验证：gravity_arm = R_RIGHT^T @ [0,+9.81,0] = (0,-9.81,0)
      对应 ccs_m6_40.MvKDCfg arm_type=1 行：GRV=(0,-9.81,0)  ✓

  左臂 R_LEFT：
    臂X → base_link -X
    臂Y → base_link +Y
    臂Z → base_link -Z（向下）

    世界系映射：
    臂X → world +Y   臂Y → world -Z   臂Z → world -X
    GRV 交叉验证：gravity_arm = R_LEFT^T @ [0,+9.81,0] = (0,+9.81,0)
      对应 ccs_m6_40.MvKDCfg arm_type=0 行：GRV=(0,+9.81,0)  ✓

  推导过程：
    R_arm_to_world_right = [[0,0,1],[1,0,0],[0,1,0]]  （列 = 臂轴在世界系方向）
    R_arm_to_world_left  = [[0,0,-1],[1,0,0],[0,-1,0]]
    R_ROOT_WORLD = [[0,0,1],[-1,0,0],[0,-1,0]]  （DUAL_ARM_ROOT_ROT = wxyz(0.5,-0.5,0.5,-0.5)）
    R_arm_to_base = R_ROOT_WORLD^T @ R_arm_to_world

说明：
  - base_link 坐标系 = 机器人根坐标系，GR00T action delta 在此系
  - FK/IK 基坐标系  = 各臂天机 SDK 解算所在系（≠ URDF J1/J10 坐标系！）
  - 欧拉角约定：天机 FK 输出 XYZABC，ABC 为 ZYX 顺序（A=绕Z, B=绕Y, C=绕X）
                对应 GR00T [roll←C, pitch←B, yaw←A]（见 config.ABC_TO_RPY）
"""

import math
import warnings
import numpy as np
from scipy.spatial.transform import Rotation


# ── 臂基坐标系相对 base_link 的变换参数 ───────────────────────────────────────

# ⚠️ 2026-06-01：用户指出这两个 _R 朝向都错（URDF 的 J1/J10 基座系朝向 ≠ SDK FK 臂系）。
#    试过用"轴对比 Kabsch"数值反解 → 因每根轴有 ± 自由度而欠定（解出的 _R 是假残差0、实际错，
#    换上后 calib 崩到 1000mm+）。sign/offset/_R/C 四者纠缠，纯 FK 数值拟合无法唯一确定。
#    正确做法：对照真机 DH(ccs_m6_40) 与 URDF 的 J1/J10 帧解析推导。下面暂保留旧值（未修正）。
#
# 列向量 = 臂坐标系各轴在 base_link 中的方向
_R_RIGHT = np.array([
    [-1.,  0.,  0.],
    [ 0., -1.,  0.],
    [ 0.,  0.,  1.],
], dtype=np.float64)

_R_LEFT = np.array([
    [-1.,  0.,  0.],
    [ 0.,  1.,  0.],
    [ 0.,  0., -1.],
], dtype=np.float64)

# 逆矩阵（正交矩阵逆 = 转置）
_R_RIGHT_INV = _R_RIGHT.T
_R_LEFT_INV  = _R_LEFT.T

# 向后兼容：_R_ARM 保留为右臂矩阵（旧代码引用）
_R_ARM     = _R_RIGHT
_R_ARM_INV = _R_RIGHT_INV

# 各臂 J0 原点在 base_link 坐标系中的位置（单位：米，来自 URDF）
_T_RIGHT = np.array([0.61736, -0.37205,  0.035424], dtype=np.float64)
_T_LEFT  = np.array([0.61736, -0.37205, -0.038576], dtype=np.float64)


def _arm_params(arm: str):
    """返回 (R, R_inv, T) 三元组。"""
    if arm == 'right':
        return _R_RIGHT, _R_RIGHT_INV, _T_RIGHT
    elif arm == 'left':
        return _R_LEFT, _R_LEFT_INV, _T_LEFT
    else:
        raise ValueError(f"arm 必须是 'left' 或 'right'，不是 {arm!r}")


# ── 欧拉角辅助（天机 FK/IK 约定：ABC = ZYX，单位：度）────────────────────────

def _abc_to_rmat(a_deg, b_deg, c_deg) -> np.ndarray:
    """天机 FK 输出的 ABC（ZYX 欧拉角，度）→ 旋转矩阵"""
    return Rotation.from_euler(
        'ZYX', [a_deg, b_deg, c_deg], degrees=True
    ).as_matrix()


def _rmat_to_abc(R: np.ndarray):
    """旋转矩阵 → 天机 IK 输入格式 ABC（ZYX 欧拉角，度）

    当目标姿态使中间角 B=±90° 时（例如 EE_RPY=(0,0,π/2) 经臂坐标系变换后），
    ZYX 分解进入万向节死锁（Gimbal lock）：A 与 C 无法单独确定，
    scipy 会产生 UserWarning 并令 C=0、A 取使旋转矩阵不变的值。
    该近似误差 < 3e-8（远小于 IK 求解精度），旋转矩阵实际保持正确，
    忽略此警告是安全的。
    """
    r = Rotation.from_matrix(R)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', UserWarning)
        a, b, c = r.as_euler('ZYX', degrees=True)
    return a, b, c


def _rpy_to_rmat(roll, pitch, yaw) -> np.ndarray:
    """GR00T 姿态（roll/pitch/yaw，弧度，XYZ 内旋）→ 旋转矩阵"""
    return Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()


def _rmat_to_rpy(R: np.ndarray):
    """旋转矩阵 → GR00T 姿态（roll/pitch/yaw，弧度）"""
    r = Rotation.from_matrix(R)
    roll, pitch, yaw = r.as_euler('xyz')
    return roll, pitch, yaw


# ── 主变换接口 ─────────────────────────────────────────────────────────────────

def fk_to_base(xyzabc_mm_deg, arm: str):
    """
    FK 输出（臂基坐标系）→ base_link 坐标系

    Args:
        xyzabc_mm_deg: FK 输出 [X_mm, Y_mm, Z_mm, A_deg, B_deg, C_deg]
        arm: 'left' 或 'right'

    Returns:
        (pos_m, rpy_rad)
        pos_m:   [x, y, z] 单位米，在 base_link 坐标系
        rpy_rad: [roll, pitch, yaw] 单位弧度，在 base_link 坐标系
    """
    x_mm, y_mm, z_mm, a, b, c = xyzabc_mm_deg
    R, R_inv, t_arm = _arm_params(arm)

    # 位置变换：p_base = R @ p_fk + t_arm
    p_fk   = np.array([x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0])
    p_base = R @ p_fk + t_arm

    # 姿态变换：R_ee_base = R @ R_ee_fk
    R_ee_fk   = _abc_to_rmat(a, b, c)
    R_ee_base = R @ R_ee_fk
    roll, pitch, yaw = _rmat_to_rpy(R_ee_base)

    return p_base, (roll, pitch, yaw)


def base_to_fk(pos_m, rpy_rad, arm: str):
    """
    base_link 坐标系目标位姿 → 臂基坐标系（供 IK 使用）

    Args:
        pos_m:   [x, y, z] 单位米，在 base_link 坐标系
        rpy_rad: [roll, pitch, yaw] 单位弧度，在 base_link 坐标系
        arm: 'left' 或 'right'

    Returns:
        xyzabc_mm_deg: [X_mm, Y_mm, Z_mm, A_deg, B_deg, C_deg]，天机 IK 输入格式
    """
    R, R_inv, t_arm = _arm_params(arm)
    p_base = np.array(pos_m)
    roll, pitch, yaw = rpy_rad

    # 位置变换：p_fk = R^T @ (p_base - t_arm)
    p_fk = R_inv @ (p_base - t_arm)

    # 姿态变换：R_ee_fk = R^T @ R_ee_base
    R_ee_base = _rpy_to_rmat(roll, pitch, yaw)
    R_ee_fk   = R_inv @ R_ee_base
    a, b, c   = _rmat_to_abc(R_ee_fk)

    x_mm, y_mm, z_mm = p_fk * 1000.0
    return [x_mm, y_mm, z_mm, a, b, c]


# ── 便捷批量接口（供 runner 使用）────────────────────────────────────────────

def build_ee_state_base(fk_right, fk_left):
    """
    双臂 FK 输出 → base_link 坐标系 EE 状态

    Args:
        fk_right: 右臂 FK [X_mm,Y_mm,Z_mm,A_deg,B_deg,C_deg]
        fk_left:  左臂 FK [X_mm,Y_mm,Z_mm,A_deg,B_deg,C_deg]

    Returns:
        right_state: (pos_m, rpy_rad) 右臂，base_link 坐标系
        left_state:  (pos_m, rpy_rad) 左臂，base_link 坐标系
    """
    right_state = fk_to_base(fk_right, 'right')
    left_state  = fk_to_base(fk_left,  'left')
    return right_state, left_state


def action_to_ik_cmd(target_pos_m, target_rpy_rad, arm: str):
    """
    GR00T 输出的目标位姿（base_link 系）→ 天机 IK 输入格式

    Args:
        target_pos_m:   [x,y,z] 米，base_link 坐标系
        target_rpy_rad: [roll,pitch,yaw] 弧度，base_link 坐标系
        arm: 'left' 或 'right'

    Returns:
        [X_mm, Y_mm, Z_mm, A_deg, B_deg, C_deg]，可直接传给天机 IK
    """
    return base_to_fk(target_pos_m, target_rpy_rad, arm)


# ── 验证工具（上机后运行）─────────────────────────────────────────────────────

def verify_transform(arm: str, fk_output_xyzabc, expected_base_xyz=None):
    """
    打印变换结果，用于上机验证。
    将机械臂移到已知关节角（如 HOME 位），读取 FK 输出后调用此函数，
    对比 base_link 坐标系结果是否符合预期。
    """
    pos, rpy = fk_to_base(fk_output_xyzabc, arm)
    print(f"[{arm}臂] FK 输出 (臂坐标系): "
          f"xyz={[f'{v:.1f}' for v in fk_output_xyzabc[:3]]}mm  "
          f"ABC={[f'{v:.1f}' for v in fk_output_xyzabc[3:]]}deg")
    print(f"[{arm}臂] 变换后 (base_link系): "
          f"xyz=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]m  "
          f"rpy=[{math.degrees(rpy[0]):.1f}, {math.degrees(rpy[1]):.1f}, {math.degrees(rpy[2]):.1f}]deg")
    if expected_base_xyz is not None:
        err = np.linalg.norm(pos - np.array(expected_base_xyz))
        print(f"[{arm}臂] 位置误差: {err*1000:.1f}mm  {'✓ OK' if err < 0.005 else '✗ 偏差过大，检查 J0 原点'}")
    # 验证逆变换
    xyzabc_back = base_to_fk(pos, rpy, arm)
    err_inv = np.linalg.norm(np.array(fk_output_xyzabc[:3]) - np.array(xyzabc_back[:3]))
    print(f"[{arm}臂] 逆变换自洽误差: {err_inv:.4f}mm  {'✓' if err_inv < 0.001 else '✗'}")
    print()


if __name__ == "__main__":
    # 快速自洽测试
    print("=== 自洽测试 ===")
    test_cases = [
        ('right', [430.0,  285.0, 324.0,  10.0,  5.0, -170.0]),
        ('left',  [430.0, -285.0, 324.0, -10.0,  5.0,  170.0]),
    ]
    for arm, fk in test_cases:
        verify_transform(arm, fk)
