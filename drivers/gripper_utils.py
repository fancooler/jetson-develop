"""
gripper_utils.py  —  Xense 夹爪量程转换工具

职责：
  GR00T 模型输出的夹爪关节值（URDF 定义）与
  Xense 实物夹爪控制量（TCP 位置指令）之间的互相转换。

URDF 定义：
  每个夹爪有 2 个对称棱柱关节，各自行程 0~50mm：
    右臂：joint_8（+Y）、joint_9（-Y），父链接 Link_7
    左臂：joint_17（+Y）、joint_18（-Y），父链接 Link_16
  两关节在训练中始终对称（joint_8 == joint_9，joint_17 == joint_18）。

Xense TCP 夹爪：
  单一位置指令，0mm = 完全闭合，85mm = 完全张开。

换算关系：
  xense_pos [mm] = joint_value [mm] × (85.0 / 50.0)
  joint_value[mm] = xense_pos [mm] × (50.0 / 85.0)

  其中 joint_value 取两个对称关节的均值（正常情况下两者相等）。
"""

import numpy as np

# ── 量程常数 ───────────────────────────────────────────────────────────────────
URDF_JOINT_MAX  = 50.0    # mm，URDF 单个夹爪关节最大行程
XENSE_POS_MAX   = 85.0    # mm，Xense TCP 最大张开位置
XENSE_POS_MIN   = 2.0     # mm，Xense 实测机械止点（防止顶死触发过流保护）

URDF_TO_XENSE   = XENSE_POS_MAX / URDF_JOINT_MAX   # 1.7
XENSE_TO_URDF   = URDF_JOINT_MAX / XENSE_POS_MAX   # ≈ 0.5882

# 右臂夹爪关节索引（在完整 action/state 向量中，待 metadata.json 确认）
# TODO: 根据算法同事提供的 metadata.json 更新以下索引
RIGHT_GRIPPER_JOINT_NAMES = ["joint_8", "joint_9"]
LEFT_GRIPPER_JOINT_NAMES  = ["joint_17", "joint_18"]


# ── URDF 关节值 → Xense 位置 ─────────────────────────────────────────────────

def urdf_to_xense(joint_value_mm: float) -> float:
    """
    单个夹爪关节位移 → Xense 位置指令

    Args:
        joint_value_mm: 单指位移，mm，范围 [0, 50]

    Returns:
        xense_pos_mm: Xense 位置，mm，范围 [XENSE_POS_MIN, XENSE_POS_MAX]
    """
    xense = float(joint_value_mm) * URDF_TO_XENSE
    return float(np.clip(xense, XENSE_POS_MIN, XENSE_POS_MAX))


def urdf_pair_to_xense(j_pos: float, j_neg: float) -> float:
    """
    一对对称夹爪关节（+Y 和 -Y）→ Xense 位置

    正常情况下 j_pos ≈ j_neg（训练时对称），取均值后换算。

    Args:
        j_pos: 正向关节位移，mm（如 joint_8 / joint_17）
        j_neg: 负向关节位移，mm（如 joint_9 / joint_18）

    Returns:
        xense_pos_mm: Xense 位置，mm
    """
    avg = (float(j_pos) + float(j_neg)) / 2.0
    return urdf_to_xense(avg)


# ── Xense 位置 → URDF 关节值 ─────────────────────────────────────────────────

def xense_to_urdf(xense_pos_mm: float) -> float:
    """
    Xense 位置读数 → 单个夹爪关节位移

    用于将实物状态回填到模型 state 向量（两个对称关节填相同值）。

    Args:
        xense_pos_mm: Xense get_gripper_status() 返回的 position，mm

    Returns:
        joint_value_mm: 单指位移，mm，范围 [0, 50]
    """
    j = float(xense_pos_mm) * XENSE_TO_URDF
    return float(np.clip(j, 0.0, URDF_JOINT_MAX))


def xense_to_urdf_pair(xense_pos_mm: float) -> tuple[float, float]:
    """
    Xense 位置读数 → (joint_pos, joint_neg) 对称关节值

    Returns:
        (j_pos_mm, j_neg_mm): 两个对称关节的位移，单位 mm
    """
    j = xense_to_urdf(xense_pos_mm)
    return j, j


# ── action delta 应用 ────────────────────────────────────────────────────────

def apply_gripper_delta(current_xense_mm: float,
                        delta_joint_mm: float) -> float:
    """
    将模型输出的 delta（URDF 关节空间）叠加到当前 Xense 位置，
    返回新的 Xense 位置指令。

    Args:
        current_xense_mm: 当前 Xense 位置读数，mm
        delta_joint_mm:   模型输出的夹爪 delta，URDF 关节空间，mm

    Returns:
        new_xense_mm: 新目标位置，mm，已 clip 到合法范围
    """
    # 先把当前 Xense 位置转回 URDF 空间
    current_joint = xense_to_urdf(current_xense_mm)
    # 叠加 delta
    new_joint = current_joint + delta_joint_mm
    new_joint = float(np.clip(new_joint, 0.0, URDF_JOINT_MAX))
    # 再转回 Xense 空间
    return urdf_to_xense(new_joint)


# ── 验证工具 ───────────────────────────────────────────────────────────────────

def print_conversion_table():
    """打印换算对照表，用于上机前人工核查"""
    print("=== Xense 夹爪量程换算对照表 ===")
    print(f"  换算系数：URDF→Xense × {URDF_TO_XENSE:.4f}，"
          f"Xense→URDF × {XENSE_TO_URDF:.4f}")
    print(f"  Xense 输出下限（机械止点保护）：{XENSE_POS_MIN} mm\n")
    print(f"  {'URDF单指(mm)':>14} | {'Xense位置(mm)':>14} | {'状态'}")
    print("  " + "-" * 46)
    for j in [0, 10, 20, 25, 30, 40, 50]:
        x = urdf_to_xense(j)
        state = "全闭" if j == 0 else ("全开" if j == 50 else "")
        print(f"  {j:>14.1f} | {x:>14.1f} | {state}")
    print()


if __name__ == "__main__":
    print_conversion_table()

    # 自洽测试
    print("=== 自洽测试 ===")
    for j in [0.0, 25.0, 50.0]:
        x = urdf_to_xense(j)
        j_back = xense_to_urdf(x)
        err = abs(j - j_back)
        print(f"  URDF {j:.1f}mm → Xense {x:.1f}mm → URDF {j_back:.1f}mm  "
              f"误差:{err:.4f}mm  {'✓' if err < 0.01 else '✗'}")

    # delta 应用测试
    print("\n=== delta 应用测试 ===")
    cur = 42.5   # Xense 当前位置（中间）
    for delta in [-5.0, +5.0, +30.0, -50.0]:
        new = apply_gripper_delta(cur, delta)
        print(f"  当前 Xense:{cur:.1f}mm + delta:{delta:+.1f}mm → 新 Xense:{new:.1f}mm")
