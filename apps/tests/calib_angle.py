#!/usr/bin/env python3
"""calib_angle.py — 用【旋转角共轭不变量】标定关节 sign/offset，与 base 系无关。

动机（见与用户讨论）：原 calib 依赖 frame_transform 把 SDK FK 转到 base_link，
若某臂 base 旋转 _R_xxx 错了，M=inv(SDK_base)·URDF_base 会被构型共轭→散布（右臂 ±40mm）。

本判据绕开 base 系：相对位姿 inv(P_i)·P_j 的【旋转角】在共轭下不变，故对正确 sign/offset：
    angle( URDF_FK(q_i)→(q_j) )  ==  angle( SDK_armFK(map q_i)→(map q_j) )
对所有样本对成立——与 base 旋转、法兰偏移都无关。只用原始 URDF FK 与原始 SDK 臂坐标系 FK。

判读：
  - 某臂能标到残差 ~0° → sign/offset 正确且连杆一致；原 calib 的 mm 散布纯是 base 系问题。
    → 该 sign/offset 可直接用于【关节回放】（回放不需要 base 系）。
  - 残差仍大 → 连杆模型本身不一致（URDF 链 ≠ SDK DH）。

用法（Jetson）：python3 test/calib_angle.py
"""
import os, sys, math, itertools
import numpy as np
from scipy.optimize import least_squares

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP)
import config as cfg
sys.path.insert(0, cfg.TJ_SDK)
from calib_joints import SdkFK, _samples, _suppress_cio
from urdf_fk import UrdfFK

np.set_printoptions(precision=4, suppress=True)


def _angle(R):
    return math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) * 0.5)))


def _wrap(off):
    out = []
    for v in off:
        w = ((v + 180) % 360) - 180
        for s in (-180, -90, 0, 90, 180):
            if abs(w - s) < 2.0:
                w = float(s); break
        out.append(round(w, 2))
    return out


def solve(arm, fk_u, sdk, samples):
    U = [np.asarray(fk_u(q))[:3, :3] for q in samples]
    angU = np.array([_angle(U[0].T @ U[k]) for k in range(1, len(U))])
    kk = sdk._kk[arm]

    def res(off, sign):
        S = [np.asarray(kk.fk(list(np.asarray(sign) * np.asarray(q) + off)),
                        float)[:3, :3] for q in samples]
        return [_angle(S[0].T @ S[k]) - angU[k - 1] for k in range(1, len(S))]

    rng = np.random.default_rng(0)
    seeds = [np.zeros(7)] + [rng.choice([-180., -90., 0., 90., 180.], 7) for _ in range(5)]
    best = None
    with _suppress_cio():
        for sign in itertools.product((1.0, -1.0), repeat=7):
            for seed in seeds:
                sol = least_squares(res, seed, args=(sign,), method='lm', max_nfev=150)
                c = 2 * sol.cost
                if best is None or c < best[0]:
                    best = (c, sign, sol.x)
        # 最优解的逐对角残差(度)
        _, sign, off = best
        S = [np.asarray(kk.fk(list(np.asarray(sign) * np.asarray(q) + off)),
                        float)[:3, :3] for q in samples]
        resid_deg = [math.degrees(abs(_angle(S[0].T @ S[k]) - angU[k - 1]))
                     for k in range(1, len(S))]
    return sign, off, max(resid_deg)


def main():
    ufk = UrdfFK()
    with _suppress_cio():
        sdk = SdkFK()
    for arm, fk_u, seed in (('right', ufk.fk_right, 0), ('left', ufk.fk_left, 1)):
        samples = _samples(None, n=14, seed=seed)
        sign, off, resid = solve(arm, fk_u, sdk, samples)
        good = resid < 0.5
        print(f"=== {arm} 臂（旋转角不变量，与 base 系无关）===")
        print(f"  sign    = {[int(s) for s in sign]}")
        print(f"  offset° = {_wrap(off)}")
        print(f"  逐对旋转角残差 max = {resid:.3f}°  "
              f"{'✓ 连杆/符号一致 → 是 base 系问题, 此 sign/offset 可用于回放' if good else '✗ 连杆本身不一致'}")
        print()


if __name__ == '__main__':
    main()
