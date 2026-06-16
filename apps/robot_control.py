#!/usr/bin/env python3
"""
MARVIN机械臂控制程序：笛卡尔坐标输入 → 逆解 → 关节阻抗运动

使用逻辑：
  1. 连接机器人，验证 UDP 数据通道
  2. 位置模式运动到零位（全关节 0 度）
  3. 切换关节阻抗模式
  4. 循环：输入末端 XYZABC → 逆解为关节角 → 下发指令 → 等待到位
  5. 输入 q 退出，自动下使能并释放机器人

参数配置（修改下方 CONFIG 区域）：
  ROBOT_IP   : 机器人控制器 IP 地址
  CONFIG_FILE: 机型配置文件路径（ccs_m3.MvKDCfg / SRS.MvKDCfg 等）
  ARM        : 控制哪条臂，'A'=左臂，'B'=右臂
  VEL_RATIO  : 速度百分比（1‒100），初期建议 10
  ACC_RATIO  : 加速度百分比（1‒100），初期建议 10
"""

import os
import sys
import time
import logging

# ─── 路径：将项目根目录加入 Python 搜索路径 ──────────────────────────────────
#_DEMO_DIR   = os.path.dirname(os.path.abspath(__file__))
#_SDK_ROOT   = os.path.dirname(_DEMO_DIR)
#sys.path.insert(0, _SDK_ROOT)

SDK_ROOT = '/home/dky/work/TJ_marvin/TJ_FX_ROBOT_CONTRL_SDK-master'
sys.path.insert(0, SDK_ROOT)


from SDK_PYTHON.fx_robot import Concise_Marvin_Robot, DCSS
from SDK_PYTHON.fx_kine  import Marvin_Kine, FX_InvKineSolvePara

# ─── 用户配置区域 ──────────────────────────────────────────────────────────────
ROBOT_IP    = '192.168.1.190'
#CONFIG_FILE = os.path.join(_SDK_ROOT, 'ccs_m3.MvKDCfg')  # 按机型选择配置文件
CONFIG_FILE = os.path.join(SDK_ROOT, 'ccs_m6_40.MvKDCfg')  # 按机型选择配置文件
ARM         = 'A'    # 'A'=左臂, 'B'=右臂
ARM_IDX     = 0      # A→0, B→1

VEL_RATIO   = 10     # 速度百分比，安全起见初期设 10，充分测试后可调高
ACC_RATIO   = 10     # 加速度百分比

# 关节阻抗刚度 K[7] 和阻尼 D[7]，可根据机器人实际情况调整
IMP_K = [10.0, 10.0, 10.0, 1.6, 1.0, 1.0, 1.0]
IMP_D = [0.8,  0.8,  0.8,  0.4, 0.4, 0.4, 0.4]

REACH_TOL   = 0.1    # 到位判断阈值（度）
MOVE_TIMEOUT = 60    # 单次运动超时时间（秒）
# ──────────────────────────────────────────────────────────────────────────────

logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
logger = logging.getLogger(__name__)


# ─── 辅助函数 ─────────────────────────────────────────────────────────────────

def _joints_reached(current: list, target: list, tol: float = REACH_TOL) -> bool:
    if not (current and target and len(current) == 7 and len(target) == 7):
        return False
    return all(abs(c - t) < tol for c, t in zip(current, target))


def _wait_stopped(robot: Concise_Marvin_Robot, dcss: DCSS,
                  target: list, timeout_s: float = MOVE_TIMEOUT) -> bool:
    """等待机械臂到达目标位置或低速停止，返回是否成功到位。"""
    deadline = time.time() + timeout_s
    time.sleep(0.05)  # 等待机械臂开始加速后再开始判断
    while time.time() < deadline:
        data = robot.subscribe(dcss)
        fb  = data['outputs'][ARM_IDX]['fb_joint_pos']
        lsf = data['outputs'][ARM_IDX]['low_speed_flag']
        if lsf[0] == 1 or _joints_reached(fb, target):
            return True
        time.sleep(0.001)
    logger.warning('运动等待超时，机械臂可能未完全到位')
    return False


def _check_state(robot: Concise_Marvin_Robot, dcss: DCSS) -> bool:
    """检查当前臂是否存在错误（状态码 100）。"""
    data  = robot.subscribe(dcss)
    state = data['states'][ARM_IDX]['cur_state']
    err   = data['states'][ARM_IDX]['err_code']
    if state == 100 or err != 0:
        logger.error(f'机械臂存在错误：cur_state={state}, err_code={err}，请先清错再运行')
        return False
    return True


def _solve_ik(kk: Marvin_Kine,
              target_xyzabc: list,
              ref_joints: list) -> list | None:
    """
    将目标末端 XYZABC 转换为关节角度。

    target_xyzabc: [X, Y, Z, A, B, C]，单位 mm/度
    ref_joints   : 7 个参考关节角，用于选取最近解（防止构型跳变）
    返回         : 7 个目标关节角（度），或 None 表示逆解失败
    """
    # XYZABC → 4×4 齐次变换矩阵
    tcp_mat = kk.xyzabc_to_mat4x4(target_xyzabc)
    if not tcp_mat:
        logger.error('XYZABC → 4×4 矩阵转换失败')
        return None

    # 4×4 矩阵展开为行优先 16 元素列表
    tcp_flat = [tcp_mat[r][c] for r in range(4) for c in range(4)]

    # 构造逆解参数结构体
    sp = FX_InvKineSolvePara()
    sp.set_input_ik_target_tcp(tcp_flat)

    # 参考关节：第 4 关节（index 3）绝对值不能过小，否则奇异
    safe_ref = list(ref_joints)
    if abs(safe_ref[3]) < 0.5:
        safe_ref[3] = 1.0  # 给予一个安全的非零偏置
    sp.set_input_ik_ref_joint(safe_ref)

    # 零空间约束类型 0：与参考关节的欧式距离最小（防止构型跳变）
    sp.set_input_ik_zsp_type(0)

    result = kk.ik(sp)
    if not result:
        logger.error('逆解失败（目标超出工作空间或处于奇异点）')
        return None
    if result.m_Output_IsOutRange:
        logger.error('目标位姿超出机器人可达空间（IsOutRange=True）')
        return None
    if result.m_Output_IsJntExd:
        logger.error('逆解关节角超出软限位（IsJntExd=True）')
        return None

    return result.m_Output_RetJoint.to_list()


def connect_robot(robot, dcss, robot_ip: str, arm_idx: int = 0) -> bool:                                                                 
    """                                                                                                                 
    连接机械臂并验证 UDP 数据通道。                                                                                                      
 
    robot   : Marvin_Robot 实例                                                                                                          
    dcss    : DCSS 订阅结构体实例                                                                                        
    robot_ip: 控制器 IP 地址，如 '192.168.1.190'                                                                                         
    arm_idx : 验证哪条臂的数据帧，0=左臂, 1=右臂                                                                                         
    返回    : True=连接并验证成功, False=失败                                                                                            
    """                                                                                                                                  
    # 建立 TCP 连接                                                                                                                      
    if not robot.connect(robot_ip):                                                                                                      
        logger.error(f'连接失败，端口被占用或 IP 不可达: {robot_ip}')                                                                  
        return False                                                                                                                     
 
    # 验证 UDP 数据通道（frame_serial 持续变化才说明数据真正到达）                                                                       
    prev_frame, update_cnt = None, 0                                                                                     
    for _ in range(10):                                                                                                                  
        data  = robot.subscribe(dcss)                                                                                                    
        frame = data['outputs'][arm_idx]['frame_serial']                                                                               
        if frame != 0 and frame != prev_frame:                                                                                           
            update_cnt += 1                                                                                                              
            prev_frame = frame                                                                                                           
        time.sleep(0.01)                                                                                                                 
                                                                                                                                         
    if update_cnt == 0:                                                                                                                
        logger.error('UDP 数据帧未更新，请检查防火墙或网络配置')                                                                         
        robot.release_robot()                                                                                                            
        return False                                                                                                                   
                                                                                                                                         
    # 检查并清除已有错误                                                                                                 
    robot.check_error_and_clear(dcss)                                                                                                  
                                                                                                                                         
    logger.info(f'机械臂连接成功: {robot_ip}')
    return True                                                                                                                          
                                                                                                                         
                                                                                                                                         
def release_robot(robot, arm: str = None) -> None:
    """                                                                                                                                  
    下使能并释放机械臂，使其他程序可以接管控制权。                                                                       
                                                                                                                                         
    robot: Marvin_Robot 实例
    arm  : 若指定 'A'/'B' 则先下使能该臂再释放；                                                                                         
           传 None 则直接释放（适用于已手动下使能的情况）                                                                                
    """                                                                                                                                  
    if arm is not None:                                                                                                                  
        robot.clear_set()                                                                                                                
        robot.set_state(arm=arm, state=0)  # state=0 下使能                                                                            
        robot.send_cmd()                                                                                                                 
        time.sleep(0.3)                                                                                                                  
        logger.info(f'臂 {arm} 已下使能')                                                                                                
                                                                                                                                         
    robot.release_robot()                                                                                                              
    logger.info('机械臂已释放')                                                                                                        
                                 

def write_485(robot, arm: str, hex_data: str, com: int = 2) -> bool:
    """
    向末端 485 串口发送 HEX 数据。

    robot   : Marvin_Robot 实例
    arm     : 'A'=左臂, 'B'=右臂
    hex_data: HEX 字节字符串，如 "01 06 00 00 00 01 48 0A"
    com     : 通道号，1=CAN, 2=COM1(485), 3=COM2(485)
    返回    : True=成功, False=失败
    """
    success, sdk_ret = robot.set_485_data(arm, hex_data, len(hex_data), com)
    if not success:
        logger.error(f'write_485 失败: arm={arm}, com={com}, data={hex_data}')
        return False
    logger.debug(f'write_485 成功: sdk_ret={sdk_ret}')
    return True


def read_485(robot, arm: str, com: int = 2, timeout_ms: int = 500) -> str | None:
    """
    从末端 485 串口接收 HEX 数据，超时返回 None。

    robot      : Marvin_Robot 实例
    arm        : 'A'=左臂, 'B'=右臂
    com        : 通道号，1=CAN, 2=COM1(485), 3=COM2(485)
    timeout_ms : 等待超时（毫秒）
    返回       : 有效字节的 HEX 字符串，如 "01 06 00 01 48 0A"；
                 超时无数据则返回 None

    注意：发送前建议先调用 robot.clear_485_cache(arm) 清空接收缓存，
          避免读到上一次残留的数据。
    """
    deadline = time.time() + timeout_ms / 1000.0
    while time.time() < deadline:
        size, hex_str = robot.get_485_data(arm, com)
        if size >= 1:
            # get_485_data 返回 256 字节全量 HEX，只取前 size 个有效字节
            valid = ' '.join(hex_str.split()[:size])
            logger.debug(f'read_485: 收到 {size} 字节: {valid}')
            return valid
        time.sleep(0.001)
    logger.warning(f'read_485 超时 ({timeout_ms}ms)，未收到数据')
    return None


# ─── 主程序 ────────────────────────────────────────────────────────────────────

def test485():
    dcss  = DCSS()
    robot = Marvin_Robot()

    # 连接
    if not connect_robot(robot, dcss, robot_ip='192.168.1.190', arm_idx=0):
        exit(1)

    # 先清缓存，再写，再读
    robot.clear_485_cache('A')
    write_485(robot, arm='A', hex_data="01 06 00 00 00 01 48 0A", com=2)
    resp = read_485(robot, arm='A', com=2, timeout_ms=500)
    if resp:
        print(f'收到回复: {resp}')
    else:
        print('无回复')


    # 释放（先下使能左臂，再释放）
    release_robot(robot, arm='A')


def main():
    logger.info('=' * 55)
    logger.info('  MARVIN 机械臂笛卡尔坐标 → 关节阻抗控制程序')
    logger.info('=' * 55)

    dcss  = DCSS()
    robot = Concise_Marvin_Robot()

    # ── 步骤 1：连接机器人 ────────────────────────────────────────────────────
    logger.info(f'[1/4] 连接机器人 {ROBOT_IP} ...')
    if not robot.connect(robot_ip=ROBOT_IP, log_switch=0):
        logger.error('连接失败，请检查网线和 IP 地址配置')
        return

    # 验证 UDP 数据通道（frame_serial 持续变化则说明通信正常）
    prev_frame, update_cnt = None, 0
    for _ in range(10):
        data  = robot.subscribe(dcss)
        frame = data['outputs'][ARM_IDX]['frame_serial']
        if frame != 0 and frame != prev_frame:
            update_cnt += 1
            prev_frame = frame
        time.sleep(0.01)

    if update_cnt == 0:
        logger.error('UDP 数据帧未更新，请检查防火墙或网络配置')
        robot.release_robot()
        return
    logger.info('UDP 数据通道正常')

    # 检查机械臂错误状态
    if not _check_state(robot, dcss):
        robot.release_robot()
        return

    # ── 步骤 2：加载运动学配置 ────────────────────────────────────────────────
    logger.info(f'[2/4] 加载运动学配置: {os.path.basename(CONFIG_FILE)}')
    if not os.path.exists(CONFIG_FILE):
        logger.error(f'配置文件不存在: {CONFIG_FILE}')
        robot.release_robot()
        return

    kk = Marvin_Kine()
    kk.log_switch(0)  # 关闭运动学库内部日志，避免刷屏

    ini = kk.load_config(arm_type=ARM_IDX, config_path=CONFIG_FILE)
    if not ini:
        logger.error('运动学配置加载失败，请确认配置文件和机型是否匹配')
        robot.release_robot()
        return

    kk.initial_kine(
        robot_type=ini['TYPE'][ARM_IDX],
        dh=ini['DH'][ARM_IDX],
        pnva=ini['PNVA'][ARM_IDX],
        j67=ini['BD'][ARM_IDX],
    )
    logger.info('运动学初始化完成')

    # ── 步骤 3：位置模式 → 运动到零位 ─────────────────────────────────────────
    zero_joints = [0.0] * 7
    logger.info('[3/4] 切换位置模式，运动到关节零位 [0,0,0,0,0,0,0]...')

    if not robot.set_position_state(arm=ARM, velRatio=VEL_RATIO, AccRatio=ACC_RATIO):
        logger.error('切换位置模式失败')
        robot.release_robot()
        return
    time.sleep(0.5)  # 等待状态切换完成

    if not robot.set_joint_position_cmd(arm=ARM, joint=zero_joints):
        logger.error('发送零位指令失败')
        robot.release_robot()
        return

    logger.info('等待机械臂到达零位（最长等待 60 秒）...')
    _wait_stopped(robot, dcss, zero_joints, timeout_s=60)

    data      = robot.subscribe(dcss)
    cur_fb    = data['outputs'][ARM_IDX]['fb_joint_pos']
    fk_mat    = kk.fk(cur_fb)
    init_pose = kk.mat4x4_to_xyzabc(fk_mat) if fk_mat else None
    if init_pose:
        logger.info(f'零位末端位姿: '
                    f'X={init_pose[0]:.1f}mm Y={init_pose[1]:.1f}mm '
                    f'Z={init_pose[2]:.1f}mm '
                    f'A={init_pose[3]:.2f}° B={init_pose[4]:.2f}° '
                    f'C={init_pose[5]:.2f}°')

    # ── 步骤 4：切换关节阻抗模式 ─────────────────────────────────────────────
    logger.info('[4/4] 切换关节阻抗模式...')
    #if not robot.set_imp_joint_state(arm=ARM, velRatio=VEL_RATIO, AccRatio=ACC_RATIO,
    #                                 K=IMP_K, D=IMP_D):
    if not robot.set_position_state(arm=ARM, velRatio=VEL_RATIO, AccRatio=ACC_RATIO):
        logger.error('切换关节阻抗模式失败')
        robot.disable(arm=ARM)
        robot.release_robot()
        return
    time.sleep(0.5)
    logger.info('关节阻抗模式已激活')

    # ── 控制循环 ──────────────────────────────────────────────────────────────
    print()
    logger.info('─' * 55)
    logger.info('  输入末端笛卡尔坐标进行运动控制')
    logger.info('  格式: X Y Z A B C  (mm / 度，空格分隔)')
    logger.info('  示例: 400 0 500 180 0 0')
    logger.info('  输入 q 退出程序')
    logger.info('─' * 55)

    while True:
        try:
            user_in = input('\n目标 [X Y Z A B C] > ').strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if user_in.lower() == 'q':
            break

        # 解析输入
        try:
            vals = list(map(float, user_in.split()))
        except ValueError:
            logger.warning('解析错误：请输入 6 个数值')
            continue

        #if len(vals) != 6:
        if len(vals) != 3:
            logger.warning(f'需要 6 个数值，实际输入了 {len(vals)} 个')
            continue

        x, y, z = vals
        a, b, c = -90,30,-90
        vals.extend([a,b,c])
        logger.info(f'目标: X={x:.2f}mm  Y={y:.2f}mm  Z={z:.2f}mm  '
                    f'A={a:.2f}°  B={b:.2f}°  C={c:.2f}°')

        # 获取当前关节位置作为 IK 参考（防止构型跳变）
        data       = robot.subscribe(dcss)
        cur_joints = data['outputs'][ARM_IDX]['fb_joint_pos']

        # 逆解
        target_joints = _solve_ik(kk, vals, cur_joints)
        if target_joints is None:
            logger.warning('逆解失败，请修改目标坐标后重试')
            continue

        logger.info(f'逆解关节角: {[round(j, 3) for j in target_joints]}°')

        # 下发关节指令
        if not robot.set_joint_position_cmd(arm=ARM, joint=target_joints):
            logger.error('发送关节指令失败，跳过本次运动')
            continue

        logger.info('运动中，等待到位...')
        reached = _wait_stopped(robot, dcss, target_joints)

        # 到位后回报实际末端位姿
        data      = robot.subscribe(dcss)
        fb_joints = data['outputs'][ARM_IDX]['fb_joint_pos']
        fk_mat    = kk.fk(fb_joints)
        if fk_mat:
            actual = kk.mat4x4_to_xyzabc(fk_mat)
            if actual:
                logger.info(f'实际末端位姿: '
                            f'X={actual[0]:.2f}  Y={actual[1]:.2f}  '
                            f'Z={actual[2]:.2f}  '
                            f'A={actual[3]:.2f}°  B={actual[4]:.2f}°  '
                            f'C={actual[5]:.2f}°')
        status = '已到位' if reached else '超时停止'
        logger.info(f'状态: {status}，等待下一个目标')

    # ── 退出：下使能并释放 ────────────────────────────────────────────────────
    logger.info('\n正在退出：下伺服...')
    robot.disable(arm=ARM)
    time.sleep(0.5)
    robot.release_robot()
    logger.info('机器人已释放，程序结束')


if __name__ == '__main__':
    main()
