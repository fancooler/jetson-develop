#!/usr/bin/env python3
"""test_seeds.py — 解右臂 61mm 悖论：两臂各跑多个采样种子，看残差是否随种子变。
若右臂任何种子都 ~61mm、左臂任何种子都 ~0 → 真·右臂专属(非采样)。"""
import os, sys, contextlib
APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP)
import config as cfg
sys.path.insert(0, cfg.TJ_SDK)
from calib_joints import SdkFK, _samples, solve_arm, _wrap
from urdf_fk import UrdfFK


@contextlib.contextmanager
def _q():
    d = os.open(os.devnull, os.O_WRONLY); s1 = os.dup(1); s2 = os.dup(2)
    try: os.dup2(d, 1); os.dup2(d, 2); yield
    finally: os.dup2(s1, 1); os.dup2(s2, 2); os.close(d); os.close(s1); os.close(s2)


def main():
    ufk = UrdfFK()
    with _q():
        sdk = SdkFK()
    fns = {'right': ufk.fk_right, 'left': ufk.fk_left}
    for arm in ('right', 'left'):
        for seed in (0, 1, 2, 3):
            smp = _samples(None, n=14, seed=seed)
            with _q():
                b = solve_arm(arm, fns[arm], sdk, smp)
            print(f"  {arm:5} seed={seed}: 残差 {b['max_dpos_mm']:8.2f}mm / {b['max_drot_deg']:6.2f}°  "
                  f"sign={[int(s) for s in b['sign']]} off={_wrap(b['offset'])}")
        print()


if __name__ == '__main__':
    main()
