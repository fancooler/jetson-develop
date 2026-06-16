#!/usr/bin/env python3
"""find_R.py — 决定性地定出每臂 base_link←SDK臂系 的旋转 _R + 关节符号 + offset。

思路（无歧义、无需原点）：
  关节轴限定 _R 必为"对角 ±1"且 det=+1 的真旋转 —— 每臂只有 4 个候选。
  _R 固定后，关节符号 sign_k 由 (_R·a_sdk_k 与 a_base_k 同向?) 唯一确定，无 sign 搜索歧义。
  再只拟 offset，看常量 C=inv(SDK_base)·URDF_base 的【散布】是否→0（散布与原点 t 无关）。
  散布→0 的那个 _R 即正确，连同其 sign/offset 就是干净的关节映射。
"""
import os, sys, math, itertools, contextlib
import numpy as np
from scipy.optimize import least_squares

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP)
import config as cfg
sys.path.insert(0, cfg.TJ_SDK)
from SDK_PYTHON.fx_kine import Marvin_Kine
from urdf_fk import UrdfFK, RIGHT_JOINTS, LEFT_JOINTS
from arm_utils import _JOINT_LIMITS_DEG

np.set_printoptions(precision=3, suppress=True)


@contextlib.contextmanager
def _q():
    d = os.open(os.devnull, os.O_WRONLY); s1 = os.dup(1); s2 = os.dup(2)
    try: os.dup2(d, 1); os.dup2(d, 2); yield
    finally: os.dup2(s1, 1); os.dup2(s2, 2); os.close(d); os.close(s1); os.close(s2)


def _rotvec(R):
    c = max(-1., min(1., (np.trace(R)-1)*.5)); a = math.acos(c)
    if a < 1e-9: return np.zeros(3)
    s = math.sin(a)
    if s < 1e-9: return np.array([a, 0, 0])
    return np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])/(2*s)*a


def urdf_axes(ufk, jn):
    Rc = np.eye(3); ax = []
    for n in jn:
        Rc = Rc @ ufk.joints[n]['origin'][:3, :3]
        a = np.asarray(ufk.joints[n]['axis'], float); ax.append(Rc @ (a/np.linalg.norm(a)))
    return np.array(ax)


def sdk_axes(kk, dd=0.5):
    with _q():
        R0 = np.array(kk.fk([0.]*7), float)[:3, :3]; ax = []
        for k in range(7):
            q = [0.]*7; q[k] = dd
            v = _rotvec(np.array(kk.fk(q), float)[:3, :3] @ R0.T); ax.append(v/np.linalg.norm(v))
    return np.array(ax)


def diag_rotations():
    out = []
    for d in itertools.product((1., -1.), repeat=3):
        if d[0]*d[1]*d[2] > 0:           # det=+1 真旋转
            out.append(np.diag(d))
    return out                            # 4 个


def samples(seed, n=14):
    rng = np.random.default_rng(seed)
    return [[rng.uniform(0.6*lo, 0.6*hi) for lo, hi in _JOINT_LIMITS_DEG] for _ in range(n)]


def eval_R(kk, U, smp, R, signs):
    """给定 _R 与 signs，拟 offset，返回 (C平移散布mm, C旋转散布deg, offset)。"""
    def Cmat(off):
        Ms = []
        for q, Uk in zip(smp, U):
            fk = np.array(kk.fk(list(signs*np.asarray(q)+off)), float)
            Sb = np.eye(4); Sb[:3, :3] = R @ fk[:3, :3]; Sb[:3, 3] = R @ (fk[:3, 3]/1000.)
            Ms.append(np.linalg.inv(Sb) @ Uk)
        return Ms
    def res(off):
        Ms = Cmat(off); M0i = np.linalg.inv(Ms[0]); r = []
        for M in Ms[1:]:
            D = M0i @ M; r += list(D[:3, 3]); r += list(_rotvec(D[:3, :3]))
        return r
    with _q():
        sol = least_squares(res, np.zeros(7), method='lm', max_nfev=200)
        Ms = Cmat(sol.x)
    Ts = np.array([M[:3, 3] for M in Ms])*1000.
    dp = np.max(np.linalg.norm(Ts - Ts.mean(0), axis=1))
    M0i = np.linalg.inv(Ms[0])
    dr = max(math.degrees(np.linalg.norm(_rotvec((M0i @ M)[:3, :3]))) for M in Ms[1:])
    return dp, dr, sol.x


def main():
    ufk = UrdfFK()
    kk0 = Marvin_Kine(); kk0.log_switch(0)
    with _q():
        ini = kk0.load_config(arm_type=0, config_path=cfg.CFG_FILE)
    for arm, jn, idx, seed in (('right', RIGHT_JOINTS, 1, 0), ('left', LEFT_JOINTS, 0, 1)):
        kk = Marvin_Kine(); kk.log_switch(0)
        with _q():
            kk.initial_kine(robot_type=ini['TYPE'][idx], dh=ini['DH'][idx],
                            pnva=ini['PNVA'][idx], j67=ini['BD'][idx])
        a_base = urdf_axes(ufk, jn); a_sdk = sdk_axes(kk)
        U = [np.asarray(ufk.fk_right(q) if arm == 'right' else ufk.fk_left(q)) for q in samples(seed)]
        smp = samples(seed)
        print("=" * 64); print(f"  {arm.upper()} 臂 —— 测 4 个候选 _R"); print("=" * 64)
        best = None
        for R in diag_rotations():
            # _R 固定 → 每关节符号唯一：_R·a_sdk_k 与 a_base_k 同向取+1，反向-1
            ok = True; signs = np.ones(7)
            for k in range(7):
                v = R @ a_sdk[k]; d = np.dot(v, a_base[k])
                if abs(abs(d)-1) > 1e-3: ok = False; break   # 轴线对不上 → 此 _R 不合法
                signs[k] = np.sign(d)
            diag = np.diag(R).astype(int).tolist()
            if not ok:
                print(f"  _R=diag{diag}: 轴线对不上，跳过"); continue
            dp, dr, off = eval_R(kk, U, smp, R, signs)
            good = dp < 2 and dr < 1
            print(f"  _R=diag{diag}: 符号={signs.astype(int).tolist()}  "
                  f"C散布 平移{dp:7.2f}mm 旋转{dr:6.2f}°  {'✓✓ 就是它' if good else ''}")
            if good:
                print(f"        offset°={[round(v,2) for v in off]}")
                best = (diag, signs.astype(int).tolist(), [round(v, 2) for v in off])
        print(f"  → {arm}: {'找到干净解 '+str(best) if best else '4 个候选都不干净（连杆/轴数据另有问题）'}\n")


if __name__ == '__main__':
    main()
