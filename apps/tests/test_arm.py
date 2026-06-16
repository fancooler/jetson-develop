#!/usr/bin/env python3
"""
机械臂测试脚本

步骤：
  1. 连接机械臂，验证 UDP 通道
  2. 读取关节角 + FK 计算末端位姿
  3. IK 回算验证（正解 → 逆解误差）
  4. 可选：执行一段小幅安全运动（需用户确认）
"""
import sys
import os
import time
import logging

logging.basicConfig(
    format='%(asctime)s %(levelname)-5s: %(message)s',
    datefmt='%H:%M:%S',
    level=logging.INFO,
)
logger = logging.getLogger("test_arm")

sys.path.insert(0, os.path.dirname(__file__))
import config
sys.path.insert(0, config.TJ_SDK)

from SDK_PYTHON.fx_robot import Concise_Marvin_Robot, DCSS
from SDK_PYTHON.fx_kine  import Marvin_Kine, FX_InvKineSolvePara


# ── 1. 连接 ───────────────────────────────────────────────────────────────────

def connect(robot, dcss) -> bool:
    logger.info(f"连接机械臂 {config.ROBOT_IP} ...")
    if not robot.connect(robot_ip=config.ROBOT_IP, log_switch=0):
        logger.error("连接失败，请检查网线和 IP")
        return False

    prev, cnt = None, 0
    for _ in range(10):
        d = robot.subscribe(dcss)
        f = d['outputs'][config.ARM_IDX]['frame_serial']
        if f != 0 and f != prev:
            cnt += 1
            prev = f
        time.sleep(0.01)

    if cnt == 0:
        logger.error("UDP 数据帧未更新，请检查防火墙")
        robot.release_robot()
        return False

    robot.check_error_and_clear()
    logger.info("连接成功，UDP 通道正常")
    return True


# ── 2. 读取状态 ───────────────────────────────────────────────────────────────

def read_state(robot, dcss, kk) -> dict:
    d       = robot.subscribe(dcss)
    out     = d['outputs'][config.ARM_IDX]
    state   = d['states'][config.ARM_IDX]
    joints  = out['fb_joint_pos']
    vel     = out['fb_joint_vel']
    cur_st  = state['cur_state']
    err     = state['err_code']

    print("\n─── 当前状态 ─────────────────────────────────────────")
    print(f"  cur_state : {cur_st}  err_code : {err}")
    print(f"  关节角(度): {[round(j, 3) for j in joints]}")
    print(f"  关节速度  : {[round(v, 3) for v in vel]}")

    # FK
    fk_mat = kk.fk(joints)
    xyzabc = kk.mat4x4_to_xyzabc(fk_mat) if fk_mat else None
    if xyzabc:
        x, y, z, a, b, c = xyzabc
        print(f"  末端位置  : X={x:.2f}mm  Y={y:.2f}mm  Z={z:.2f}mm")
        print(f"  末端姿态  : A={a:.2f}°  B={b:.2f}°  C={c:.2f}°")
        # 换算为 GR00T 格式
        groot_state = config.mm_deg_to_m_rad(xyzabc)
        print(f"  GR00T格式 : xyz=[{groot_state[0]:.4f}, {groot_state[1]:.4f}, {groot_state[2]:.4f}]m")
        print(f"              rpy=[{groot_state[3]:.4f}, {groot_state[4]:.4f}, {groot_state[5]:.4f}]rad")
    print("──────────────────────────────────────────────────────")

    return {'joints': joints, 'xyzabc': xyzabc}


# ── 3. IK 回算验证 ────────────────────────────────────────────────────────────

def verify_ik(kk, joints, xyzabc):
    if xyzabc is None:
        logger.warning("FK 失败，跳过 IK 验证")
        return

    print("\n─── IK 回算验证 ──────────────────────────────────────")
    tcp_mat  = kk.xyzabc_to_mat4x4(xyzabc)
    tcp_flat = [tcp_mat[r][c] for r in range(4) for c in range(4)]

    sp = FX_InvKineSolvePara()
    sp.set_input_ik_target_tcp(tcp_flat)
    ref = list(joints)
    if abs(ref[3]) < 0.5:
        ref[3] = 1.0
    sp.set_input_ik_ref_joint(ref)
    sp.set_input_ik_zsp_type(0)

    res = kk.ik(sp)
    if not res:
        print("  IK 无解（当前姿态本身处于奇异点附近）")
        return
    if res.m_Output_IsOutRange:
        print("  IK 超出工作空间")
        return

    ik_joints = res.m_Output_RetJoint.to_list()
    errors    = [abs(a - b) for a, b in zip(joints, ik_joints)]
    print(f"  FK关节角: {[round(j, 4) for j in joints]}")
    print(f"  IK回算 : {[round(j, 4) for j in ik_joints]}")
    print(f"  误差(度): {[round(e, 4) for e in errors]}  最大={max(errors):.4f}°")
    if max(errors) < 0.1:
        print("  ✓ IK 回算误差正常")
    else:
        print("  ✗ 误差偏大，请检查运动学配置文件是否匹配机型")
    print("──────────────────────────────────────────────────────")


# ── 4. 小幅运动测试 ───────────────────────────────────────────────────────────

def motion_test(robot, dcss, kk, current_joints):
    print("\n─── 运动测试 ─────────────────────────────────────────")
    print("  将在当前位置基础上，关节 1 偏转 +5°，再回原位")
    print("  速度/加速度均为 10%")
    ans = input("  确认执行？(y/N) ").strip().lower()
    if ans != 'y':
        print("  已跳过运动测试")
        return

    # 切换位置模式
    if not robot.set_position_state(
        arm=config.ARM, velRatio=config.VEL_RATIO, AccRatio=config.ACC_RATIO
    ):
        logger.error("切换位置模式失败")
        return
    time.sleep(0.5)

    # 目标：关节 1 偏转 +5°
    target = list(current_joints)
    target[0] += 5.0
    logger.info(f"运动到: {[round(j, 2) for j in target]}")
    robot.set_joint_position_cmd(arm=config.ARM, joint=target)

    # 等待到位
    _wait(robot, dcss, target, tol=0.3)

    # 读取实际到位位置
    d      = robot.subscribe(dcss)
    fb     = d['outputs'][config.ARM_IDX]['fb_joint_pos']
    err    = abs(fb[0] - target[0])
    print(f"  目标关节1: {target[0]:.2f}°  实际: {fb[0]:.2f}°  误差: {err:.3f}°")

    # 回原位
    time.sleep(0.5)
    logger.info(f"回原位: {[round(j, 2) for j in current_joints]}")
    robot.set_joint_position_cmd(arm=config.ARM, joint=current_joints)
    _wait(robot, dcss, current_joints, tol=0.3)

    print("  运动测试完成")
    print("──────────────────────────────────────────────────────")


def _wait(robot, dcss, target, tol=0.3, timeout=20):
    deadline = time.time() + timeout
    while time.time() < deadline:
        d  = robot.subscribe(dcss)
        fb = d['outputs'][config.ARM_IDX]['fb_joint_pos']
        if fb and all(abs(c - t) < tol for c, t in zip(fb, target)):
            return True
        time.sleep(0.05)
    logger.warning("等待超时")
    return False


# ── 主函数 ────────────────────────────────────────────────────────────────────

def main():
    print("=" * 55)
    print("  天机机械臂测试")
    print("=" * 55)

    dcss  = DCSS()
    robot = Concise_Marvin_Robot()

    if not connect(robot, dcss):
        return

    # 加载运动学
    kk = Marvin_Kine()
    kk.log_switch(0)
    ini = kk.load_config(arm_type=config.ARM_IDX, config_path=config.CFG_FILE)
    if not ini:
        logger.error(f"运动学配置加载失败: {config.CFG_FILE}")
        robot.release_robot()
        return
    kk.initial_kine(
        robot_type=ini['TYPE'][config.ARM_IDX],
        dh=ini['DH'][config.ARM_IDX],
        pnva=ini['PNVA'][config.ARM_IDX],
        j67=ini['BD'][config.ARM_IDX],
    )
    logger.info("运动学配置加载完成")

    # 读取当前状态
    state = read_state(robot, dcss, kk)

    # IK 回算
    verify_ik(kk, state['joints'], state['xyzabc'])

    # 可选运动测试
    motion_test(robot, dcss, kk, state['joints'])

    # 释放
    robot.release_robot()
    logger.info("机械臂已释放，测试结束")


if __name__ == "__main__":
    main()
