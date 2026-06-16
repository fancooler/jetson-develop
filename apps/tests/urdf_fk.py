#!/usr/bin/env python3
"""urdf_fk.py — ros_model_260418_1 的自包含 URDF 正运动学（纯 numpy）

给 calib_joints.py 用：算任意关节角下，base_link → 臂法兰(flange) 的 4x4 位姿。
  右臂链: joint_1..joint_7  → Link_7
  左臂链: joint_10..joint_16 → Link_16
（joint_8/9/17/18 是夹爪 prismatic，忽略）

URDF 约定：joint 变换 = Trans(origin.xyz) · RPY(origin.rpy, 固定轴XYZ) · Rot(axis, theta)
FK = 沿链连乘。base_link 为参考系（identity）。

自检：python3 test/urdf_fk.py
  对 HOME/ZERO/J2_30 三组角，与 verify_ee_frame.URDF_REF 比对，应 <1mm / <0.5°。
"""
import os
import math
import xml.etree.ElementTree as ET

import numpy as np

URDF_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'groot_n15/assets/ros_model_260418_1/ros_model_260418_1_without_tabletop_module.urdf')

RIGHT_JOINTS = [f'joint_{i}' for i in range(1, 8)]      # joint_1..7  → Link_7
LEFT_JOINTS  = [f'joint_{i}' for i in range(10, 17)]    # joint_10..16 → Link_16


def _rpy_to_mat(r, p, y):
    """URDF 固定轴 XYZ：R = Rz(y)·Ry(p)·Rx(r)。"""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _axis_angle_to_mat(axis, theta):
    """Rodrigues：绕单位轴 axis 转 theta(rad)。"""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    x, y, z = a
    c, s, C = math.cos(theta), math.sin(theta), 1 - math.cos(theta)
    return np.array([
        [c + x*x*C,   x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, c + z*z*C],
    ])


def _T(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


class UrdfFK:
    """解析一次 URDF，按关节名查 origin/axis，提供任意角度 FK。"""

    def __init__(self, urdf_path=URDF_PATH):
        root = ET.parse(urdf_path).getroot()
        self.joints = {}
        for j in root.findall('joint'):
            name = j.get('name')
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0').split())] if o is not None else [0, 0, 0]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0').split())] if o is not None else [0, 0, 0]
            ax = j.find('axis')
            axis = [float(v) for v in ax.get('xyz').split()] if ax is not None else [0, 0, 1]
            self.joints[name] = {
                'origin': _T(_rpy_to_mat(*rpy), np.array(xyz)),
                'axis': axis,
                'type': j.get('type'),
            }

    def fk(self, joint_names, q_deg):
        """沿 joint_names 连乘，q_deg 同长（度），返回 base_link→末端 4x4。"""
        T = np.eye(4)
        for name, q in zip(joint_names, q_deg):
            jd = self.joints[name]
            Tj = jd['origin'] @ _T(_axis_angle_to_mat(jd['axis'], math.radians(q)),
                                   np.zeros(3))
            T = T @ Tj
        return T

    def fk_right(self, q_deg):
        return self.fk(RIGHT_JOINTS, q_deg)

    def fk_left(self, q_deg):
        return self.fk(LEFT_JOINTS, q_deg)


def mat_to_pos_rpy(T):
    """4x4 → (pos_m[3], rpy_deg[3])，rpy 为固定轴 XYZ（与 URDF_REF 一致）。"""
    pos = T[:3, 3]
    R = T[:3, :3]
    # 固定轴 XYZ 反解：roll=atan2(R21,R22), pitch=asin(-R20), yaw=atan2(R10,R00)
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(R[2, 0]) < 0.99999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # 万向节锁
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return pos, [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


# ── 自检：对照 verify_ee_frame.URDF_REF ───────────────────────────────────────
_SELF_TEST = {
    'HOME': {'qr': [42, 4, 15, -79, 31, 14, 10], 'ql': [-52, -62, -99, -90, 18, -8, -35],
             'R': ([0.33915, -0.52028, 0.51858], [-122.200, -0.556, -168.714]),
             'L': ([0.54659, 0.01223, -0.29848], [167.529, 43.721, -41.486])},
    'ZERO': {'qr': [0]*7, 'ql': [0]*7,
             'R': ([0.61736, -0.37205, 0.81092], [-90.0, -90.0, 180.0]),
             'L': ([0.61736, -0.37205, -0.81408], [-90.0, -90.0, 180.0])},
    'J2_30': {'qr': [0, 30, 0, 0, 0, 0, 0], 'ql': [0]*7,
              'R': ([0.31686, -0.37205, 0.73040], [-90.0, -60.0, -180.0]),
              'L': ([0.61736, -0.37205, -0.81408], [-90.0, -90.0, 180.0])},
}


def _geo_deg(Ra, Rb):
    """两旋转矩阵的测地角(度)，表示无关，规避欧拉万向节锁多义性。"""
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _self_test():
    fk = UrdfFK()
    worst_p = worst_r = 0.0
    print(f"URDF: {URDF_PATH}\n")
    for name, ref in _SELF_TEST.items():
        for side, q, fn in [('R', ref['qr'], fk.fk_right), ('L', ref['ql'], fk.fk_left)]:
            T = fn(q)
            pos = T[:3, 3]
            rpos = np.array(ref[side][0])
            R_ref = _rpy_to_mat(*[math.radians(v) for v in ref[side][1]])
            dp = np.linalg.norm((pos - rpos) * 1000)         # mm
            dr = _geo_deg(T[:3, :3], R_ref)                  # 旋转测地角(度)
            worst_p = max(worst_p, dp); worst_r = max(worst_r, dr)
            ok = '✓' if (dp < 1.0 and dr < 0.5) else '✗'
            print(f"  {name:6} {side}: |Δpos|={dp:6.2f}mm  Δrot={dr:5.2f}°  {ok}")
    good = worst_p < 1.0 and worst_r < 0.5
    print(f"\n{'✓ URDF FK 与 URDF_REF 一致' if good else '✗ 不一致，FK 实现有问题'} "
          f"(worst pos={worst_p:.2f}mm rot={worst_r:.2f}°)")
    return good


if __name__ == '__main__':
    _self_test()
