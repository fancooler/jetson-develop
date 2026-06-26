#!/usr/bin/env bash
# robot_up.sh — 一键重启天机臂/夹爪/相机/视触觉四个 ROS2 节点 + 双臂回零（算法调试用）
#
# 在 Jetson 上运行。免去开三个终端分别 launch 的麻烦。
#
# 用法：
#   ./robot_up.sh              # 真机：杀旧 → 起三节点 → 使能伺服 → 双臂回零
#   ./robot_up.sh --mock       # 全 mock（无硬件联调）
#   ./robot_up.sh --no-home    # 起节点但不回零
#   ./robot_up.sh --stop       # 只杀掉三节点（+runner），不重启
#
# 节点后台运行、各自写日志：~/ros2_logs/{arm,gripper,camera,tactile}.log
# 看状态： ros2 node list   /   tail -f ~/ros2_logs/arm.log

# 不用 set -u/-e：ROS setup.bash 会引用未定义变量(AMENT_TRACE_SETUP_FILES 等)，-u 会中断；
# pkill 无匹配返回非 0，-e 也会中断。脚本内变量均有默认值/显式赋值，无需 -u。

USE_MOCK=false
DO_HOME=true
STOP_ONLY=false
for a in "$@"; do
  case "$a" in
    --mock)    USE_MOCK=true ;;
    --no-home) DO_HOME=false ;;
    --stop)    STOP_ONLY=true ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "未知参数: $a（--mock|--no-home|--stop）"; exit 2 ;;
  esac
done

WS="$HOME/develop/nodes"
LOGDIR="$HOME/ros2_logs"
mkdir -p "$LOGDIR"

# ~/ros2_logs 的三份日志是覆盖写(>)、不累积，无需清。
# ~/.ros/log 则每次 ros2 launch 都新建一个运行目录、从不自动清 → 会慢慢堆满盘。
# 这里只清后者，保留最近 KEEP_ROS_LOG_RUNS 次运行供事后排查。
ROS_LOG_DIR="$HOME/.ros/log"
KEEP_ROS_LOG_RUNS=30

# ── 环境（脚本非交互，显式 source）──
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROBOT_ID="${ROBOT_ID:-robot1}"
# 自包含：按 ROBOT_ID 从车队注册表导出 ROS_DOMAIN_ID + ROBOTS_YAML
# （camera.launch.py 依赖 ROBOTS_YAML；不依赖调用方是否 source 过 robot_env.sh）。
if [ -f "$HOME/develop/config/robot_env.sh" ]; then
  source "$HOME/develop/config/robot_env.sh"
else
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"   # 兜底（注册表缺失时维持旧行为）
fi

KILL_PAT="tj_marvin_driver|xense_gripper_driver|camera_driver|tactile_driver|runner_dual"

echo "==== [1] 杀掉旧节点 ===="
pkill -f tj_marvin_driver     2>/dev/null
pkill -f xense_gripper_driver 2>/dev/null
pkill -f camera_driver        2>/dev/null
pkill -f tactile_driver       2>/dev/null
pkill -f runner_dual          2>/dev/null   # 会抢控制器单连接，必须清掉
# 等进程真正退出（天机 SDK 端口要进程退出 OS 才回收，否则重启会 port is occupied）
for _ in $(seq 1 20); do
  pgrep -f "$KILL_PAT" >/dev/null || break
  sleep 0.5
done
if pgrep -f "$KILL_PAT" >/dev/null; then
  echo "  ⚠️ 仍有残留进程，强杀："; pgrep -af "$KILL_PAT"
  pkill -9 -f "$KILL_PAT" 2>/dev/null; sleep 1
fi
echo "  旧节点已清理"

if $STOP_ONLY; then echo "==== --stop：只杀不启，完成 ===="; exit 0; fi

# ── 清理 ~/.ros/log 历史运行目录（保留最近 KEEP_ROS_LOG_RUNS 个）──
# 按修改时间倒序，跳过最近 N 个，其余整目录删除。
# 只匹配真实目录(-type d)，latest 等软链是 -type l 不会被删。
if [ -d "$ROS_LOG_DIR" ]; then
  find "$ROS_LOG_DIR" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null \
    | sort -rn | tail -n +$((KEEP_ROS_LOG_RUNS + 1)) | cut -d' ' -f2- \
    | while read -r d; do rm -rf "$d"; done
  echo "==== [1.5] ~/.ros/log 历史已清理（保留最近 $KEEP_ROS_LOG_RUNS 次运行）===="
fi

# ── 启动三个节点（后台、脱离终端、各自日志）──
echo "==== [2] 启动三节点（模式: $([ $USE_MOCK = true ] && echo mock || echo 真机)）===="
if $USE_MOCK; then
  ARM_ARGS="use_mock:=true app_dir:=$HOME/develop/apps drivers_dir:=$HOME/develop/drivers";  GRIP_ARGS="mock_left:=true mock_right:=true app_dir:=$HOME/develop/apps"
else
  ARM_ARGS="use_mock:=false app_dir:=$HOME/develop/apps drivers_dir:=$HOME/develop/drivers"; GRIP_ARGS="mock_left:=false mock_right:=false app_dir:=$HOME/develop/apps"
fi

# arm 节点 25Hz 的 FK 调试输出占日志 ~98%（天机 SDK 的 fk 矩阵 print，log_switch 压不住）；
# 写盘前用 grep 滤掉，省硬盘 + 日志可读；连接/错误/INFO 等有用行保留。
setsid bash -c "ros2 launch tj_marvin_driver tj_marvin.launch.py $ARM_ARGS 2>&1 | grep --line-buffered -vE 'fk result, matrix|Pose mat to xyzabc|xyzabc:' > $LOGDIR/arm.log" </dev/null &
setsid ros2 launch xense_gripper_driver xense_gripper.launch.py $GRIP_ARGS \
       >"$LOGDIR/gripper.log" 2>&1 </dev/null &
setsid ros2 launch camera_driver       camera.launch.py                   \
       >"$LOGDIR/camera.log"  2>&1 </dev/null &
setsid ros2 launch tactile_driver      tactile.launch.py                  \
       >"$LOGDIR/tactile.log" 2>&1 </dev/null &
echo "  已后台启动；日志：$LOGDIR/{arm,gripper,camera,tactile}.log"

# ── 等臂节点就绪且已连接，再回零 ──
echo "==== [3] 等臂连接（最多 ~25s）===="
ARM_OK=false
for _ in $(seq 1 50); do
  if timeout 2 ros2 topic echo /arm/status --once 2>/dev/null | grep -q "connected: true"; then
    ARM_OK=true; break
  fi
  sleep 0.5
done

if ! $ARM_OK; then
  echo “  ⚠️ 臂未在预期时间内连上，跳过使能/回零。查 $LOGDIR/arm.log（port is occupied / 网线 / 控制器）”
else
  # 关键顺序：先 enter-pos 使能伺服，再 go_home（不能反过来）。
  # enter-pos 做 servo_reset（松刹车、出”咔哒”）+ set_position_state，必须先于任何运动指令。
  # 否则首发 go_home 时伺服常未使能 → set_joint_position_cmd 被控制器拒（返回 False）→ 回零失败；
  # 手动重试”碰巧”成功，正是因为那次的 enter-pos 已先把伺服使能了。
  echo “  → 进入位置跟随模式（伺服使能，应听到”咔哒”一声）”
  ros2 run arm_client arm_cli enter-pos
  if $DO_HOME; then
    echo “  伺服已使能 → 双臂回零”
    ros2 run arm_client arm_cli home both
  else
    echo “  （--no-home，跳过回零）”
  fi
fi

# ── 夹爪归位 ──
if $DO_HOME; then
  echo "==== [4] 等夹爪连接（最多 ~20s）===="
  GRIP_OK=false
  for _ in $(seq 1 40); do
    if timeout 2 ros2 topic echo /gripper/status --once 2>/dev/null | grep -q "connected: true"; then
      GRIP_OK=true; break
    fi
    sleep 0.5
  done

  if ! $GRIP_OK; then
    echo "  ⚠️ 夹爪未在预期时间内连上，跳过归位。查 $LOGDIR/gripper.log"
  else
    echo "  → 夹爪归位（两爪 → 42.5 mm）"
    ros2 topic pub --once /gripper/command gripper_interfaces/msg/GripperCommand \
      "{side: 'both', position: 42.5, max_effort: 0.0}"
  fi
fi

echo "==== 完成 ===="
echo "  节点：ros2 node list      日志：tail -f $LOGDIR/arm.log"
echo "  停止：$0 --stop"
