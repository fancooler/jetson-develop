#!/usr/bin/env python3
"""
runner_dual_ros2.py — 纯 ROS2 接口的双臂+夹爪推理主循环【示例 / 模板】

给算法同事的参考程序：演示如何在【自己的算法服务器】上，只通过 ROS2 接口访问
机械臂、摄像头、夹爪来跑一个推理控制循环，**不依赖天机 SDK / Xense SDK / jetson-work 内部模块**。
把你自己的策略（GR00T 等）填进 MockPolicy.infer() 即可。

对照 app/runner_dual.py（跑在 Jetson 上、机械臂+夹爪直接用本地 SDK、相机走 ROS2）：
  本文件把【机械臂、夹爪也都换成 ROS2 接口】，于是整个程序变成一个纯客户端，可在任意能
  连到机器人 ROS2 网络的机器上运行：
    - 机械臂：arm_client.ArmClient —— 离散动作(connect/go_home) + 流式 /arm/joint_command
              （见 jetson-ros2/src/arm_client、tj_marvin_driver）
    - 摄像头：rclpy 直接订阅 /camera_*/color/image_raw[/compressed]
    - 夹爪：  GripperClient —— 流式发 /gripper/command(GripperCommand)、订阅 /gripper/status
              （见 jetson-ros2/src/xense_gripper_driver、gripper_interfaces）
    - 策略：  MockPolicy 占位（J1 小幅正弦 + 夹爪缓慢开合，仅演示链路，不做任何智能）。

═══ 运行前提 ═══
  1) 机械臂驱动节点在跑：
       Jetson:  ros2 launch tj_marvin_driver tj_marvin.launch.py use_mock:=true
  2) 夹爪驱动节点在跑（不在线也行：指令无害丢弃、状态为空，链路照常跑；或用 --no-gripper）：
       Jetson:  ros2 launch xense_gripper_driver xense_gripper.launch.py \
                  mock_left:=true mock_right:=true        # 无硬件先 mock
  3) 摄像头：
       - 真机：在 Jetson 跑 camera_driver 节点；跨网络订阅建议用 --compressed（省带宽）
       - 没有相机：加 --mock-cameras，本进程内发布合成图，链路照样跑通
  4) 本机已 source jetson-ros2 的 install（拿到 arm_interfaces/arm_client/gripper_interfaces），
     且 ROS_DOMAIN_ID 与各节点一致（如 ROBOT_ID=robot1 → 1）：
       source ~/work/jetson-ros2/install/setup.bash
       export ROS_DOMAIN_ID=1

═══ 用法 ═══
  # 无相机、无真机：Jetson 起 mock 臂+夹爪节点后，本机这样跑通整条链路
  python3 runner_dual_ros2.py --mock-cameras

  # 接真实 camera_driver（跨网络用压缩流）
  python3 runner_dual_ros2.py --compressed

  python3 runner_dual_ros2.py --rate 20 --max-cycles 100 --arm both
  python3 runner_dual_ros2.py --no-gripper        # 完全不碰夹爪接口
"""

import argparse
import math
import threading
import time

import numpy as np

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

from arm_client import ArmClient
from gripper_interfaces.msg import GripperCommand, GripperStatus

# ── 摄像头 topic（与 camera_driver / camera_topics.py 对齐；此处内联以便独立分发）──
ROLES = ('front', 'left_wrist', 'right_wrist')
CAM_TOPICS = {
    'front':       '/camera_front/color/image_raw',
    'left_wrist':  '/camera_left_wrist/color/image_raw',
    'right_wrist': '/camera_right_wrist/color/image_raw',
}
CAM_W, CAM_H = 640, 480

# 图像 QoS：与 camera_driver 一致（BestEffort + 只留最新一帧）
IMG_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=1,
                     durability=DurabilityPolicy.VOLATILE)

# ── 夹爪（与 gripper_interfaces / app/gripper.py 量程对齐）──────────────────────
# /gripper/command 用 BestEffort/depth10（与节点订阅一致）——depth>1 让一周期内 left+right
# 两条都不被覆盖；/gripper/status 用 BestEffort 订阅可同时兼容 Reliable/BestEffort 的发布端。
GRIP_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=10,
                      durability=DurabilityPolicy.VOLATILE)
POS_OPEN, POS_CLOSE = 85.0, 2.0      # mm，完全张开 / 闭合（防顶死取 2）


# ════════════════════════════════════════════════════════════════════════════
# 摄像头订阅（这是算法服务器拿图像的标准姿势）
# ════════════════════════════════════════════════════════════════════════════

class CameraSubscriber(Node):
    """订阅三路相机，后台 spin，read() 拿最新帧（非阻塞，缺帧给黑帧）。"""

    def __init__(self, compressed: bool = False):
        super().__init__('runner_ros2_cam_sub')
        self._compressed = compressed
        self._bridge = CvBridge()
        self._latest = {r: None for r in ROLES}
        self._locks = {r: threading.Lock() for r in ROLES}
        for role in ROLES:
            if compressed:
                self.create_subscription(CompressedImage, CAM_TOPICS[role] + '/compressed',
                                         self._make_cb(role), IMG_QOS)
            else:
                self.create_subscription(Image, CAM_TOPICS[role],
                                         self._make_cb(role), IMG_QOS)

    def _make_cb(self, role: str):
        def cb(msg):
            try:
                if self._compressed:
                    arr = np.frombuffer(msg.data, np.uint8)
                    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                else:
                    bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                rgb = bgr[:, :, ::-1].copy()           # 推理一般吃 RGB
            except Exception as e:                      # noqa: BLE001
                self.get_logger().warn(f'[cam/{role}] 解码失败: {e}', throttle_duration_sec=5.0)
                return
            with self._locks[role]:
                self._latest[role] = rgb
        return cb

    def have_all(self) -> bool:
        return all(self._latest[r] is not None for r in ROLES)

    def read(self) -> dict:
        out = {}
        for r in ROLES:
            with self._locks[r]:
                f = self._latest[r]
            out[r] = f.copy() if f is not None else np.zeros((CAM_H, CAM_W, 3), np.uint8)
        return out


class MockCameraPublisher(Node):
    """【仅无相机时用】在本进程发布三路合成图，让示例脱离真实相机也能跑通。
    真机上请改用 camera_driver 节点，不要起这个。"""

    def __init__(self, compressed: bool = False, fps: float = 15.0):
        super().__init__('runner_ros2_mock_cam_pub')
        self._compressed = compressed
        self._bridge = CvBridge()
        self._pubs = {}
        for role in ROLES:
            if compressed:
                self._pubs[role] = self.create_publisher(
                    CompressedImage, CAM_TOPICS[role] + '/compressed', IMG_QOS)
            else:
                self._pubs[role] = self.create_publisher(Image, CAM_TOPICS[role], IMG_QOS)
        self._n = 0
        self.create_timer(1.0 / fps, self._tick)

    def _tick(self):
        self._n += 1
        for role in ROLES:
            img = np.zeros((CAM_H, CAM_W, 3), np.uint8)
            # 画点动态内容：角色名 + 帧号 + 一个移动的方块，便于肉眼确认是“活”的
            x = int((self._n * 5) % (CAM_W - 60))
            cv2.rectangle(img, (x, 200), (x + 60, 260), (0, 180, 255), -1)
            cv2.putText(img, f'MOCK {role}', (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.putText(img, f'frame {self._n}', (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            stamp = self.get_clock().now().to_msg()
            if self._compressed:
                ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    continue
                msg = CompressedImage()
                msg.header.stamp = stamp
                msg.format = 'jpeg'
                msg.data = buf.tobytes()
            else:
                msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
                msg.header.stamp = stamp
            self._pubs[role].publish(msg)


# ════════════════════════════════════════════════════════════════════════════
# 夹爪客户端（对应 jetson-ros2 的 xense_gripper_driver 节点 + gripper_interfaces）
# ════════════════════════════════════════════════════════════════════════════

class GripperClient(Node):
    """ROS2 夹爪客户端：流式发布 /gripper/command(GripperCommand) 控位置(mm)，
    订阅 /gripper/status(GripperStatus) 读各爪当前位置。对应 xense_gripper_driver 节点。

    - 需与该节点同 ROS_DOMAIN_ID；节点 auto_connect=true 时无需显式连接，直接下发即可。
    - 本节点交给外部 executor spin（与相机共用一个后台线程）。
    - 夹爪节点不在线时：command() 发出去无人收（无害），positions() 返回 None，链路照常跑。
    - 精确到位/阻塞抓取可改用 /gripper/grip 动作；本示例走流式（与 20Hz 控制循环匹配）。
    """

    def __init__(self):
        super().__init__('runner_ros2_gripper_cli')
        self._pub = self.create_publisher(GripperCommand, '/gripper/command', GRIP_QOS)
        self._pos = {'left': None, 'right': None}
        self._lock = threading.Lock()
        self.create_subscription(GripperStatus, '/gripper/status', self._on_status, GRIP_QOS)

    def _on_status(self, msg: GripperStatus):
        with self._lock:
            for g in msg.grippers:
                if g.name in self._pos:
                    self._pos[g.name] = g.position

    def positions(self) -> dict:
        """最新夹爪位置(mm)：{'left': float|None, 'right': float|None}。"""
        with self._lock:
            return dict(self._pos)

    def command(self, side: str, position_mm: float, max_effort: float = 0.0):
        """流式下发目标位置(mm)。side: 'left'/'right'/'both'。非阻塞。
        max_effort<=0 时节点用其默认力（fmax 参数）。"""
        msg = GripperCommand()
        msg.side = side
        msg.position = float(max(POS_CLOSE, min(POS_OPEN, position_mm)))
        msg.max_effort = float(max_effort)
        self._pub.publish(msg)

    def open(self, side: str = 'both'):
        self.command(side, POS_OPEN)

    def close(self, side: str = 'both'):
        self.command(side, POS_CLOSE)


# ════════════════════════════════════════════════════════════════════════════
# 占位策略 —— 算法同事把这里换成自己的 GR00T 推理
# ════════════════════════════════════════════════════════════════════════════

class MockPolicy:
    """占位策略：输入观测，输出每臂 7 个绝对关节角目标(度) + 每侧夹爪目标(mm)。

    真实策略（GR00T 等）应当：
      - 消费 obs['images']（3 路 RGB ndarray）、obs['joints_deg']、obs['ee']、obs['gripper_mm']；
      - 输出绝对关节目标（本驱动 /arm/joint_command 约定单位是【度】）和夹爪目标(mm)。
    这里仅在“起始关节角”基础上对 J1 叠加小幅正弦、夹爪缓慢开合，证明控制链路通、不做任何智能。
    """

    def __init__(self, base_joints_deg: dict, amp_deg: float = 8.0, freq_hz: float = 0.25):
        self._base = {k: list(v) for k, v in base_joints_deg.items()}
        self._amp = amp_deg
        self._freq = freq_hz
        self._t0 = time.time()

    def infer(self, obs: dict) -> dict:
        t = time.time() - self._t0
        delta = self._amp * math.sin(2 * math.pi * self._freq * t)
        # 夹爪：~10s 周期在 张开(85) ↔ 半合(30) 之间缓慢往返（cos → 从张开起步），
        #       证明夹爪链路通；演示故意不全闭，避免在真机上夹到东西。
        g_lo, g_hi = 30.0, POS_OPEN
        g = 0.5 * (g_hi + g_lo) + 0.5 * (g_hi - g_lo) * math.cos(2 * math.pi * 0.1 * t)
        out = {'gripper': {}}
        for arm in ('left', 'right'):
            if arm not in self._base:
                continue
            j = list(self._base[arm])
            j[0] += delta            # 仅动 J1，安全、可见
            out[arm] = j
            out['gripper'][arm] = g
        return out


# ════════════════════════════════════════════════════════════════════════════
# 主程序
# ════════════════════════════════════════════════════════════════════════════

def read_observation(cams: CameraSubscriber, arm: ArmClient,
                     gripper: 'GripperClient | None' = None) -> dict:
    """组装一帧观测：3 路图像 + 双臂关节角(度) + 双臂末端位姿 + 双爪位置(mm)。"""
    imgs = cams.read()
    jd = arm.joints_dict()                      # {'left':[7 rad],'right':[7 rad]}
    joints_deg = ({k: [math.degrees(v) for v in jd[k]] for k in jd} if jd else {})
    gmm = gripper.positions() if gripper is not None else {'left': None, 'right': None}
    return {
        'images': imgs,                          # {role: HxWx3 RGB uint8}
        'joints_deg': joints_deg,                # {'left':[7],'right':[7]} 度
        'ee': {'left': arm.ee_pose('left'),      # PoseStamped 或 None(mock 无 FK)
               'right': arm.ee_pose('right')},
        'gripper_mm': gmm,                       # {'left':mm|None,'right':mm|None} 来自 /gripper/status
    }


def _fmt_mm(v) -> str:
    return f'{v:5.1f}' if v is not None else '   --'


def main():
    ap = argparse.ArgumentParser(description='纯 ROS2 双臂推理示例（机械臂/夹爪/相机均走 ROS2）')
    ap.add_argument('--rate', type=float, default=20.0, help='控制频率 Hz（默认 20，对齐 EXEC_HZ）')
    ap.add_argument('--max-cycles', type=int, default=0, help='跑多少 cycle 后退出（0=无限，Ctrl-C 停）')
    ap.add_argument('--arm', choices=['left', 'right', 'both'], default='both', help='驱动哪条臂/爪')
    ap.add_argument('--mock-cameras', action='store_true', help='本进程发布合成图（无真实相机时用）')
    ap.add_argument('--compressed', action='store_true', help='相机走压缩流(/compressed)，跨网络省带宽')
    ap.add_argument('--no-gripper', action='store_true', help='完全不碰夹爪接口（不建节点、不下发）')
    ap.add_argument('--server-timeout', type=float, default=10.0, help='等待机械臂节点就绪超时(秒)')
    args = ap.parse_args()
    sides = ['left', 'right'] if args.arm == 'both' else [args.arm]

    print('=' * 64)
    print(f'  纯 ROS2 双臂推理示例   rate={args.rate}Hz  arm={args.arm}  '
          f'gripper={"off" if args.no_gripper else "on"}  '
          f'cameras={"mock(本进程)" if args.mock_cameras else ("压缩" if args.compressed else "原始")}')
    print('=' * 64)

    # ── 机械臂（ROS2 客户端）─────────────────────────────────────────────────
    arm = ArmClient()                            # 内部 init/spin rclpy
    cam_exec = cam_thread = None
    cams = mock_pub = gripper = None
    streaming_on = False
    t_obs_l, t_infer_l, t_exec_l = [], [], []
    try:
        if not arm.wait_for_servers(timeout=args.server_timeout):
            print('❌ 未发现机械臂节点。先在 Jetson 起：\n'
                  '   ros2 launch tj_marvin_driver tj_marvin.launch.py use_mock:=true\n'
                  '   并确认本机 ROS_DOMAIN_ID 与之一致。')
            return 1

        ok, msg = arm.connect()
        print(f'  connect: {ok} ({msg})')
        if not ok:
            return 1
        print(f'  go_home({args.arm}): {arm.go_home(args.arm)}')
        print(f'  enter_position_mode: {arm.enter_position_mode()}')   # 真机回零后必做

        # ── 摄像头 + 夹爪（共用一个 executor / 后台 spin 线程）──────────────────
        cams = CameraSubscriber(compressed=args.compressed)
        cam_exec = MultiThreadedExecutor()
        cam_exec.add_node(cams)
        if args.mock_cameras:
            mock_pub = MockCameraPublisher(compressed=args.compressed)
            cam_exec.add_node(mock_pub)
        if not args.no_gripper:
            gripper = GripperClient()
            cam_exec.add_node(gripper)
        cam_thread = threading.Thread(target=cam_exec.spin, daemon=True)
        cam_thread.start()

        # 等三路首帧（拿不到不致命，用黑帧继续）
        deadline = time.time() + 6.0
        while time.time() < deadline and not cams.have_all():
            time.sleep(0.05)
        print('  摄像头首帧: ' + ('三路就位' if cams.have_all()
              else '超时（用黑帧继续；真机请确认 camera_driver 在跑或加 --mock-cameras）'))

        # 夹爪起始张开（对应 runner_dual.py 回零后 grip.open()；节点不在线则无害）
        if gripper is not None:
            gripper.open('both')
            print('  夹爪：已发起始张开指令（/gripper/command），状态读 /gripper/status')

        # ── 用当前关节角初始化占位策略，并开启流式 ──────────────────────────────
        jd = arm.joints_dict()
        base_deg = ({k: [math.degrees(v) for v in jd[k]] for k in jd} if jd else
                    {'left': [0.0] * 7, 'right': [0.0] * 7})
        policy = MockPolicy(base_deg)

        ok, msg = arm.enable_streaming(True)
        print(f'  enable_streaming: {ok} ({msg})')
        if not ok:
            return 1
        streaming_on = True

        print('-' * 64)
        print('  进入控制循环（Ctrl-C 退出）')
        print('-' * 64)

        # ── 控制循环（obs → infer → 下发，定频）──────────────────────────────
        step_dt = 1.0 / args.rate
        cycle = 0
        while rclpy.ok():
            cycle += 1
            t_cycle = time.time()

            t0 = time.time()
            obs = read_observation(cams, arm, gripper)
            t_obs = (time.time() - t0) * 1000

            t0 = time.time()
            action = policy.infer(obs)            # ← 换成你的推理
            t_infer = (time.time() - t0) * 1000

            t0 = time.time()
            left_cmd = action.get('left') if args.arm in ('left', 'both') else None
            right_cmd = action.get('right') if args.arm in ('right', 'both') else None
            arm.stream_joints(args.arm, left=left_cmd, right=right_cmd)
            # 夹爪：按策略输出的每侧目标位置(mm)流式下发（独立节点 xense_gripper_driver）。
            # 两侧目标相同时发一条 side='both'，避免一周期发两条在 depth 队列里互相覆盖
            # （曾导致先发的 left 被后发的 right 挤掉、左爪不跟）。
            if gripper is not None:
                gact = action.get('gripper', {})
                gl, gr = gact.get('left'), gact.get('right')
                if args.arm == 'both' and gl is not None and gr is not None and abs(gl - gr) < 0.5:
                    gripper.command('both', gl)
                else:
                    for s in sides:
                        gp = gact.get(s)
                        if gp is not None:
                            gripper.command(s, gp)
            t_exec = (time.time() - t0) * 1000

            t_obs_l.append(t_obs); t_infer_l.append(t_infer); t_exec_l.append(t_exec)
            jr = obs['joints_deg'].get('right', [0] * 7)
            jl = obs['joints_deg'].get('left', [0] * 7)
            gm = obs['gripper_mm']
            if cycle % 10 == 0:
                hz = 1000.0 / max(1e-6, t_obs + t_infer + t_exec)
                print(f'[{cycle:5d}] obs={t_obs:4.1f} infer={t_infer:4.1f} exec={t_exec:4.1f} ms  '
                      f'L_J1={jl[0]:6.1f}° R_J1={jr[0]:6.1f}°  '
                      f'grip(L/R)={_fmt_mm(gm["left"])}/{_fmt_mm(gm["right"])}mm  (~{hz:.0f}Hz 计算)')

            if args.max_cycles and cycle >= args.max_cycles:
                print(f'  达到 --max-cycles={args.max_cycles}，退出')
                break

            remaining = (t_cycle + step_dt) - time.time()
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print('\n  Ctrl-C，安全退出 ...')
    finally:
        # 关流式 → 夹爪张开复位 → 停相机/夹爪 executor → 拆 arm（顺序重要）
        if streaming_on:
            try:
                arm.enable_streaming(False)
            except Exception:
                pass
        if gripper is not None:
            try:
                gripper.open('both')          # 退出前张开，安全
            except Exception:
                pass
        if cam_exec is not None:
            try:
                cam_exec.shutdown()
            except Exception:
                pass
            if cam_thread is not None and cam_thread.is_alive():
                cam_thread.join(timeout=2.0)
            for n in (cams, mock_pub, gripper):
                if n is not None:
                    try:
                        n.destroy_node()
                    except Exception:
                        pass
        arm.shutdown()

        if t_obs_l:
            print('\n' + '=' * 64)
            print(f'  统计  {len(t_obs_l)} cycles')
            for name, lst in [('读观测', t_obs_l), ('推理  ', t_infer_l), ('下发  ', t_exec_l)]:
                a = np.array(lst)
                print(f'  {name}: avg={a.mean():5.1f}ms  min={a.min():5.1f}  max={a.max():5.1f}ms')
            tot = np.array(t_obs_l) + np.array(t_infer_l) + np.array(t_exec_l)
            print(f'  cycle avg {tot.mean():.1f}ms ≈ {1000/tot.mean():.0f}Hz（计算耗时，不含 sleep）')
            print('=' * 64)
        print('  程序结束')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
