#!/usr/bin/env python3
"""calib_joints.py — 离线解出 训练(URDF)关节 → 天机SDK关节 的 sign/offset 映射

要解决的问题（见 memory: sim-real-coord-mismatch）：
  模型 action / 训练轨迹的关节角是 URDF/Isaac 约定；真机 SDK 是另一套关节约定
  （符号翻转 + 零偏）。直接把训练关节角喂给 SDK，真机姿态 ≠ 仿真姿态。

模型：对每条臂，存在 per-joint 线性映射
      q_sdk[i] = sign[i] * q_urdf[i] + offset[i]     (sign∈{+1,-1}, offset 常数°)
使得真机物理构型与仿真一致。

求解（纯离线，不连机器人、不开 Isaac）：
  - URDF_FK(q)  : test/urdf_fk.py，base_link→flange 4x4（已用 URDF_REF 自检过）
  - SDK_FK(q)   : 天机 Marvin_Kine + frame_transform → base_link 4x4
  判据：若 sign/offset 正确，则
      M(q) = SDK_FK(sign*q+offset)^{-1} · URDF_FK(q)
  对所有 q 应为同一个常数刚体变换 C（C = SDK法兰↔URDF Link_7 的固定工具/帧偏移，
  即 issue A 的 95mm 工具 + flange 帧旋转）。
  → 遍历 128 种 sign 组合，每组用 least_squares 拟 offset 使 {M(q_k)} 方差最小；
    取残差最小的组合。残差≈0 即说明 sign/offset 模型成立且解唯一。

用法（在 Jetson 上）：
  cd ~/work/app
  python3 test/calib_joints.py                 # 解双臂，打印映射表 + 自检
  python3 test/calib_joints.py --save          # 同时写 test/joint_map.json

输出可被 replay_arm.py --map 直接使用。
"""
import os
import sys
import json
import math
import argparse
import itertools
import contextlib

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


@contextlib.contextmanager
def _suppress_cio():
    """屏蔽 C 层 stdout+stderr（天机 kk.fk 每次都 print 整个矩阵，且刷在 fd2，
    log_switch 压不住；只挡 fd1 会漏）。仅在 FK 密集段使用，期间不打日志。"""
    devnull = os.open(os.devnull, os.O_WRONLY)
    saved1 = os.dup(1)
    saved2 = os.dup(2)
    try:
        os.dup2(devnull, 1)
        os.dup2(devnull, 2)
        yield
    finally:
        os.dup2(saved1, 1)
        os.dup2(saved2, 2)
        os.close(devnull)
        os.close(saved1)
        os.close(saved2)

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP_DIR)

import config as cfg
# SDK 路径用 cfg.TJ_SDK（= app/../TJ_marvin/...），ThinkBook / Jetson 均正确，
# 不再硬编码 ~/work/... （ThinkBook 上工作目录在 ~/work/jetson-work/）。
sys.path.insert(0, cfg.TJ_SDK)
from SDK_PYTHON.fx_kine import Marvin_Kine
import frame_transform as ft
from urdf_fk import UrdfFK
from arm_utils import _JOINT_LIMITS_DEG

np.set_printoptions(precision=4, suppress=True)

# 两段式求解参数：
#   Stage1 对全部 128 符号组合单起点(0)粗排；Stage2 只对 cost 最小的 _TOP_K 组
#   做【多起点】精修，逃出 LM 在角度周期空间的局部极小（右臂单起点曾卡 61mm）。
# 比"全部组合×多起点"快一个数量级，又不丢正确解（正确符号的旋转残差最小、必进 top-K）。
_TOP_K = 6             # Stage2 精修的候选符号组合数
_N_OFFSET_SEEDS = 6    # Stage2 每组的结构化随机起点数（另加 全0 与 Stage1 粗解）


# ── SDK FK → base_link 4x4 ───────────────────────────────────────────────────
class SdkFK:
    """天机 SDK 离线 FK，返回 base_link 4x4。每臂初始化一次后复用。"""

    def __init__(self):
        kk0 = Marvin_Kine(); kk0.log_switch(0)
        self.ini = kk0.load_config(arm_type=0, config_path=cfg.CFG_FILE)
        assert self.ini, "SDK config 加载失败"
        self._kk = {}
        for arm, idx in (('right', 1), ('left', 0)):
            kk = Marvin_Kine(); kk.log_switch(0)
            kk.initial_kine(robot_type=self.ini['TYPE'][idx], dh=self.ini['DH'][idx],
                            pnva=self.ini['PNVA'][idx], j67=self.ini['BD'][idx])
            self._kk[arm] = kk

    def fk(self, arm, q_deg):
        fk_mat = np.array(self._kk[arm].fk(list(q_deg)), float)  # 4x4 臂坐标系(mm)
        R_a2b, _, t_arm = ft._arm_params(arm)
        T = np.eye(4)
        T[:3, :3] = R_a2b @ fk_mat[:3, :3]
        T[:3, 3] = R_a2b @ (fk_mat[:3, 3] / 1000.0) + t_arm
        return T


def _pose_diff(A, B):
    """两 4x4 之差 → (平移 m[3], 旋转向量 rad[3])。"""
    D = np.linalg.inv(A) @ B
    return D[:3, 3], Rotation.from_matrix(D[:3, :3]).as_rotvec()


def _samples(arm_idx_unused, n=14, seed=0):
    """在各关节限位的 60% 区间内均匀采样 n 组关节角（度）。"""
    rng = np.random.default_rng(seed)
    qs = []
    for _ in range(n):
        q = [rng.uniform(0.6*lo, 0.6*hi) for (lo, hi) in _JOINT_LIMITS_DEG]
        qs.append(q)
    return qs


def _rotvec_np(R):
    """旋转矩阵 → 旋转向量(rad)，纯 numpy 实现（替代 scipy Rotation，热路径提速）。
    与 rotvec 同度量：远离最优时梯度足够，不像 (R−I) 那样饱和导致 LM 卡住。"""
    cos = max(-1.0, min(1.0, (np.trace(R) - 1.0) * 0.5))
    angle = math.acos(cos)
    if angle < 1e-9:
        return np.zeros(3)
    s = math.sin(angle)
    if s < 1e-9:                      # angle≈π，残差里极少精确命中，给个粗略大值即可
        return np.array([angle, 0.0, 0.0])
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return ax / (2.0 * s) * angle


def solve_arm(arm, urdf_fk_fn, sdk, samples):
    """遍历 128 sign 组合，拟 offset，返回最优 (sign, offset, 残差信息)。"""
    U = [urdf_fk_fn(q) for q in samples]          # URDF 真值（不随 sign/offset 变）

    def residual(off, sign):
        # 热路径：纯 numpy（scipy Rotation 建对象太慢）。判据不变：sign/offset 对时
        # M(q)=inv(SDK_FK)·URDF_FK 对所有 q 应为同一常量，故各 M 相对 M0 的差应全为 0。
        # 旋转用 angle-axis(rotvec)，远离最优时梯度足够（勿用 (R−I)，会饱和卡住）。
        sq = np.asarray(sign)
        Ms = [np.linalg.inv(sdk.fk(arm, sq * np.asarray(q) + off)) @ U[k]
              for k, q in enumerate(samples)]
        M0inv = np.linalg.inv(Ms[0])
        res = []
        for M in Ms[1:]:
            D = M0inv @ M                       # 一致则为单位阵
            res += list(D[:3, 3])               # 平移差(米)
            res += list(_rotvec_np(D[:3, :3]))  # 旋转差(rad)
        return res

    rng = np.random.default_rng(2025)

    # ── Stage1：每个符号组合单起点(0)粗解，按 cost 排序 ──────────────────────────
    ranked = []
    with _suppress_cio():
        for sign in itertools.product((1.0, -1.0), repeat=7):
            sol = least_squares(residual, np.zeros(7), args=(sign,),
                                method='lm', max_nfev=100)
            ranked.append((2 * sol.cost, sign, sol.x))
    ranked.sort(key=lambda t: t[0])

    # ── Stage2：对 cost 最小的 _TOP_K 组做【多起点】精修，逃出 offset 局部极小 ──
    #   起点 = 全 0 ＋ Stage1 粗解 ＋ 若干"每轴∈{-180,-90,0,90,180}"结构化随机种子，
    #   覆盖 offset 真值的 basin（真值几乎都是 90° 整数倍）。
    best = None
    with _suppress_cio():
        for _c0, sign, x0 in ranked[:_TOP_K]:
            seeds = [np.zeros(7), np.array(x0, float)]
            seeds += [rng.choice([-180.0, -90.0, 0.0, 90.0, 180.0], size=7)
                      for _ in range(_N_OFFSET_SEEDS)]
            for seed in seeds:
                sol = least_squares(residual, seed, args=(sign,),
                                    method='lm', max_nfev=150)
                cost = 2 * sol.cost  # sum of squares
                if best is None or cost < best['cost']:
                    best = {'cost': cost, 'sign': sign, 'offset': sol.x}

    # 用最优解评估常量 C 的离散度（mm / deg），并取 C 均值
    sign, off = best['sign'], best['offset']
    with _suppress_cio():
        Ms = [np.linalg.inv(sdk.fk(arm, np.array(sign)*np.array(q) + off)) @ U[k]
              for k, q in enumerate(samples)]
    M0 = Ms[0]
    dpos_mm, drot_deg = [], []
    for M in Ms[1:]:
        dt, dr = _pose_diff(M0, M)
        dpos_mm.append(np.linalg.norm(dt) * 1000)
        drot_deg.append(np.degrees(np.linalg.norm(dr)))
    C_t = M0[:3, 3] * 1000
    C_rpy = [math.degrees(v) for v in Rotation.from_matrix(M0[:3, :3]).as_euler('xyz')]
    best.update(max_dpos_mm=max(dpos_mm), max_drot_deg=max(drot_deg),
                C_t_mm=C_t, C_rpy_deg=C_rpy)
    return best


def _wrap(off, tol=12.0):
    """offset 规整到 (-180,180] 并吸附到最近的 90° 整数倍（物理零偏应为 90 倍数）。
    返回 (snapped[7], far_idx)：far_idx = 离最近 90 倍数 > tol 的关节下标（可疑）。
    -180 统一表示为 180，便于跨种子比较。"""
    out, far = [], []
    for i, v in enumerate(off):
        w = ((v + 180) % 360) - 180
        nearest = min((-180.0, -90.0, 0.0, 90.0, 180.0), key=lambda s: abs(w - s))
        if abs(w - nearest) <= tol:
            out.append(180.0 if nearest == -180.0 else nearest)
        else:
            out.append(round(w, 2)); far.append(i)
    return out, far


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--save', action='store_true', help='写 test/joint_map.json')
    ap.add_argument('--n', type=int, default=14, help='采样组数')
    args = ap.parse_args()

    ufk = UrdfFK()
    with _suppress_cio():
        sdk = SdkFK()
    fns = {'right': ufk.fk_right, 'left': ufk.fk_left}

    # 多种子【多数表决】稳健标定：跳过退化 seed=0，逐关节按多数定 sign/offset，
    # 自动剔除"自由工具偏移 C 吸收"造成的少数派 0 残差异类解（实测见 seed=3）。
    import collections
    SEEDS = (1, 2, 3, 4, 5)
    result = {}
    for arm in ('right', 'left'):
        print("=" * 72)
        print(f"  标定 {arm.upper()} 臂（多种子多数表决，跳过退化 seed=0）")
        print("=" * 72)
        cands = []
        for sd in SEEDS:
            smp = _samples(None, n=args.n, seed=sd)
            b = solve_arm(arm, fns[arm], sdk, smp)
            sg = [int(s) for s in b['sign']]
            of, far = _wrap(b['offset'])
            cands.append((sd, b, sg, of, far))
            print(f"  seed={sd}: 残差 {b['max_dpos_mm']:7.3f}mm/{b['max_drot_deg']:5.2f}°  "
                  f"sign={sg}  offset={of}" + (f"  ⚠非90°整倍:{[f'J{i+1}' for i in far]}" if far else ""))

        clean = [c for c in cands if c[1]['max_dpos_mm'] < 0.5]
        if not clean:
            print("  ✗ 所有种子残差都偏大 → sign/offset 模型不足或有 bug，请检查")
            clean = [min(cands, key=lambda c: c[1]['max_dpos_mm'])]

        # 逐关节多数表决符号；再仅用符号=多数的种子表决偏移
        sign = [collections.Counter(c[2][j] for c in clean).most_common(1)[0][0]
                for j in range(7)]
        match = [c for c in clean if c[2] == sign] or clean
        n_agree = sum(1 for c in clean if c[2] == sign)
        offset, amb = [], []
        for j in range(7):
            cnt = collections.Counter(c[3][j] for c in match)
            offset.append(cnt.most_common(1)[0][0])
            if len(cnt) > 1:
                amb.append(f"J{j+1}")
        resid = min(c[1]['max_dpos_mm'] for c in match)

        print(f"  → 多数表决 sign={sign}  (符合的干净种子 {n_agree}/{len(clean)})")
        print(f"     offset={offset}   残差≈{resid:.3f}mm")
        if amb:
            print(f"     ⚠ 偏移无共识: {amb}（FK 不可观测/欠定，需上机微调；"
                  f"J7=腕roll、95mm 工具在轴线上是典型）")
        if n_agree < len(clean):
            print(f"     注: {len(clean)-n_agree} 个种子符号不同(少数派异类解，已剔除)")
        print()
        result[arm] = {'sign': sign, 'offset': offset,
                       'resid_mm': round(resid, 3),
                       'sign_votes': f"{n_agree}/{len(clean)}",
                       'ambiguous_offset_joints': amb}

    print("=" * 72)
    print("  映射用法: q_sdk[i] = sign[i] * q_train[i] + offset[i]  (q 单位:度)")
    print("=" * 72)
    for arm in ('right', 'left'):
        r = result[arm]
        print(f"  {arm:5}: sign={r['sign']}  offset={r['offset']}  "
              f"残差{r['resid_mm']}mm  符号票数={r['sign_votes']}  "
              f"待上机微调={r['ambiguous_offset_joints'] or '无'}")

    if args.save:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'joint_map.json')
        with open(path, 'w') as f:
            json.dump(result, f, indent=2)
        print(f"\n已写入 {path}")


if __name__ == '__main__':
    main()
