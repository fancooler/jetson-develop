#!/usr/bin/env bash
# robot_env.sh — 按 ROBOT_ID 从中央注册表 robots.yaml 导出 ROS_DOMAIN_ID
#
# 位置：仓库顶层 config/（与 robots.yaml 并排）；直接 source 源文件，无需 colcon build。
# 用法（写进每台机器的 ~/.bashrc，放在 source ROS 之后）：
#     export ROBOT_ID=robot1
#     source <repo>/config/robot_env.sh      # Jetson: ~/ros2_ws/config  ThinkBook: ~/work/jetson-ros2/config
# 它额外 export ROBOTS_YAML（指向同目录 robots.yaml），供 camera.launch.py / config_dual.py 定位注册表。
#
# 这样 domain 的唯一来源就是 robots.yaml，不会出现 .bashrc 与注册表写两遍对不上。
# 开发机（跑 viewer）想看哪台机器人，就把 ROBOT_ID 设成那台再 source。

_RE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_RE_YAML="$_RE_DIR/robots.yaml"

# 导出注册表路径，供夹爪等非 ROS 程序定位它（config_dual.py 读 $ROBOTS_YAML）。
export ROBOTS_YAML="$_RE_YAML"

if [ -z "$ROBOT_ID" ]; then
    echo "[robot_env] 警告：未设 ROBOT_ID，未修改 ROS_DOMAIN_ID（保持默认 0）" >&2
else
    _RE_DOMAIN=$(ROBOTS_YAML="$_RE_YAML" RID="$ROBOT_ID" python3 -c '
import yaml, os, sys
try:
    d = yaml.safe_load(open(os.environ["ROBOTS_YAML"]))
    print(d["robots"][os.environ["RID"]]["domain_id"])
except Exception as e:
    sys.stderr.write("lookup failed: %s\n" % e)
')
    if [ -n "$_RE_DOMAIN" ]; then
        export ROS_DOMAIN_ID="$_RE_DOMAIN"
        echo "[robot_env] ROBOT_ID=$ROBOT_ID -> ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    else
        echo "[robot_env] 警告：robots.yaml 中找不到 ROBOT_ID=$ROBOT_ID，ROS_DOMAIN_ID 未设" >&2
    fi
    unset _RE_DOMAIN
fi
unset _RE_DIR _RE_YAML

# drivers/ 下的封装层（arm_utils, gripper_utils, wrench_source, tactile_utils）
# 加入 PYTHONPATH，让 apps/ 脚本无需修改即可 import
export PYTHONPATH=$HOME/develop/drivers${PYTHONPATH:+:$PYTHONPATH}
