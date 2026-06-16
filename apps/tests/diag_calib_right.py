#!/usr/bin/env python3
"""诊断右臂 calib 残差：逐样本看 C=inv(SDK_FK)·URDF_FK 的平移分布，
判断 61mm 是【单样本离群】还是【系统性】。在 Jetson 上跑。"""
import os, sys, json, contextlib
import numpy as np

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP)
import config as cfg
sys.path.insert(0, cfg.TJ_SDK)
from calib_joints import SdkFK, _samples, _rotvec_np
from urdf_fk import UrdfFK


@contextlib.contextmanager
def _q():
    d = os.open(os.devnull, os.O_WRONLY); s1 = os.dup(1); s2 = os.dup(2)
    try:
        os.dup2(d, 1); os.dup2(d, 2); yield
    finally:
        os.dup2(s1, 1); os.dup2(s2, 2); os.close(d); os.close(s1); os.close(s2)


def diag(arm, seed):
    m = json.load(open(os.path.join(os.path.dirname(__file__), 'joint_map.json')))[arm]
    sign = np.array(m['sign'], float); off = np.array(m['offset'], float)
    ufk = UrdfFK(); fk_u = ufk.fk_right if arm == 'right' else ufk.fk_left
    with _q():
        sdk = SdkFK()
    samples = _samples(None, n=14, seed=seed)
    Cs = []
    with _q():
        for qd in samples:
            U = fk_u(qd)
            S = sdk.fk(arm, sign * np.array(qd) + off)
            Cs.append(np.linalg.inv(S) @ U)
    Ts = np.array([C[:3, 3] for C in Cs]) * 1000.0   # mm
    mean = Ts.mean(0)
    dist = np.linalg.norm(Ts - mean, axis=1)
    print(f"=== {arm} 臂  sign={sign.astype(int).tolist()} off={np.round(off,1).tolist()} ===")
    print(f"  C 平移均值(mm)={np.round(mean,1).tolist()}  各轴std(mm)={np.round(Ts.std(0),1).tolist()}")
    for i, (t, d) in enumerate(zip(Ts, dist)):
        flag = '  <== 离群' if d > 10 else ''
        print(f"  样本{i:2d}: C_t=[{t[0]:7.1f},{t[1]:7.1f},{t[2]:7.1f}]  距均值={d:6.1f}mm{flag}")
    print(f"  → 距均值 max={dist.max():.1f}mm  median={np.median(dist):.1f}mm  "
          f"{'(疑似单样本离群)' if (dist>10).sum()<=2 else '(系统性，非离群)'}")


if __name__ == '__main__':
    diag('right', seed=0)
    print()
    diag('left', seed=1)
