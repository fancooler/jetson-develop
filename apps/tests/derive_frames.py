#!/usr/bin/env python3
"""derive_frames.py — 对照 URDF(base_link系) 与 SDK FK(SDK臂系) 的关节轴，
解出正确的 base_link←SDK臂系 旋转 _R 和各关节符号。在 Jetson 跑。

原理（与基座系朝向无关地求 _R）：
  同一根物理关节，URDF 给出它在 base_link 中的轴 a_k^base，
  SDK FK 给出它在 SDK 臂系中的轴 a_k^sdk（零位有限差分提取，最稳）。
  二者是同一物理轴(差一个符号)：  _R · a_k^sdk = sign_k · a_k^base。
  7 根轴 → 用带符号的 Kabsch 解出旋转 _R 和 sign_k。
  （offset 零偏不在此处求；本脚本只定 _R 与符号。）
"""
import os, sys, math, contextlib
import numpy as np

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP)
import config as cfg
sys.path.insert(0, cfg.TJ_SDK)
from SDK_PYTHON.fx_kine import Marvin_Kine
from urdf_fk import UrdfFK, RIGHT_JOINTS, LEFT_JOINTS
import frame_transform as ft

np.set_printoptions(precision=3, suppress=True)


@contextlib.contextmanager
def _q():
    d = os.open(os.devnull, os.O_WRONLY); s1 = os.dup(1); s2 = os.dup(2)
    try:
        os.dup2(d, 1); os.dup2(d, 2); yield
    finally:
        os.dup2(s1, 1); os.dup2(s2, 2); os.close(d); os.close(s1); os.close(s2)


def _rotvec(R):
    c = max(-1., min(1., (np.trace(R) - 1) * .5)); ang = math.acos(c)
    if ang < 1e-9: return np.zeros(3)
    s = math.sin(ang)
    ax = np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])
    return ax/(2*s)*ang


def urdf_axes(ufk, joint_names):
    """各关节轴在 base_link 中的方向(零位)。"""
    Rc = np.eye(3); axes = []
    for n in joint_names:
        jd = ufk.joints[n]
        Rc = Rc @ jd['origin'][:3, :3]          # 零位：关节转角=0，只累乘 origin 旋转
        a = np.asarray(jd['axis'], float); a = a/np.linalg.norm(a)
        axes.append(Rc @ a)
    return np.array(axes)


def sdk_axes(kk, n=7, delta_deg=0.5):
    """各关节轴在 SDK 臂系中的方向(零位有限差分)。"""
    with _q():
        R0 = np.array(kk.fk([0.0]*n), float)[:3, :3]
        axes = []
        for k in range(n):
            q = [0.0]*n; q[k] = delta_deg
            Rk = np.array(kk.fk(q), float)[:3, :3]
            v = _rotvec(Rk @ R0.T)
            axes.append(v/np.linalg.norm(v))
    return np.array(axes)


def kabsch(P, Q):
    """求 R 使 R@P_i ≈ Q_i。P,Q: (n,3)。"""
    H = P.T @ Q
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    return Vt.T @ np.diag([1, 1, d]) @ U.T


def solve_R_signs(a_sdk, a_base):
    signs = np.ones(len(a_sdk))
    for _ in range(20):
        Q = signs[:, None] * a_base
        R = kabsch(a_sdk, Q)                       # R@a_sdk ≈ signs*a_base
        new = np.sign(np.einsum('ij,ij->i', (R @ a_sdk.T).T, a_base))
        new[new == 0] = 1
        if np.all(new == signs): break
        signs = new
    resid = np.max(np.linalg.norm((R @ a_sdk.T).T - signs[:, None]*a_base, axis=1))
    return R, signs.astype(int), resid


def main():
    ufk = UrdfFK()
    kk0 = Marvin_Kine(); kk0.log_switch(0)
    with _q():
        ini = kk0.load_config(arm_type=0, config_path=cfg.CFG_FILE)
    for arm, jn, idx, R_cur in (('right', RIGHT_JOINTS, 1, ft._R_RIGHT),
                                ('left',  LEFT_JOINTS,  0, ft._R_LEFT)):
        kk = Marvin_Kine(); kk.log_switch(0)
        with _q():
            kk.initial_kine(robot_type=ini['TYPE'][idx], dh=ini['DH'][idx],
                            pnva=ini['PNVA'][idx], j67=ini['BD'][idx])
        a_base = urdf_axes(ufk, jn)
        a_sdk = sdk_axes(kk)
        R, signs, resid = solve_R_signs(a_sdk, a_base)
        print("=" * 70)
        print(f"  {arm.upper()} 臂")
        print("=" * 70)
        print("  URDF 轴(base_link):")
        for k, a in enumerate(a_base): print(f"    J{k+1}: {np.round(a,3)}")
        print("  SDK  轴(SDK臂系):")
        for k, a in enumerate(a_sdk): print(f"    J{k+1}: {np.round(a,3)}")
        print(f"  → 解出 base_link←SDK臂系 旋转 _R(列=SDK轴在base中方向):\n{np.round(R,3)}")
        print(f"     轴对齐残差 max = {resid:.4f}  {'✓ 解出干净旋转' if resid<0.05 else '✗ 不是单纯旋转'}")
        print(f"     关节符号(SDK 轴相对 URDF 轴) = {signs.tolist()}")
        print(f"  当前 frame_transform._R_{arm.upper()}:\n{np.round(R_cur,3)}")
        same = np.allclose(R, R_cur, atol=1e-2)
        print(f"     与当前 _R {'一致 ✓' if same else '★不一致 —— 当前 _R 是错的，应替换为上面解出的 _R★'}")
        print()


if __name__ == '__main__':
    main()
