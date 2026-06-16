#!/usr/bin/env python3
"""test_joint_map.py — joint_map 模块离线单元测试（纯 numpy，无需 SDK / 机器人）

可在 ThinkBook 直接跑：
    cd ~/work/jetson-work/app   # 或 Jetson: cd ~/work/app
    python3 test/test_joint_map.py

覆盖：
  1. 缺 json / 禁用 → 恒等映射（不崩、不动数据）
  2. 有 json → urdf_to_sdk / sdk_to_urdf 数值正确 + 往返一致
  3. json 非法（sign≠±1、维度错）→ 退回恒等 + 不抛
  4. is_active() 状态正确
"""
import os
import sys
import json
import tempfile

import numpy as np

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP_DIR)

import joint_map

_PASS = 0
_FAIL = 0


def check(cond, msg):
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print(f"  ✓ {msg}")
    else:
        _FAIL += 1
        print(f"  ✗ {msg}")


def _use_map_file(d: dict | None):
    """把映射写到临时 json 并指向它（None=删除指向，触发缺失分支）。"""
    if d is None:
        os.environ.pop('JOINT_MAP_FILE', None)
        os.environ['JOINT_MAP_FILE'] = '/nonexistent/joint_map.json'
    else:
        f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False)
        json.dump(d, f)
        f.close()
        os.environ['JOINT_MAP_FILE'] = f.name
    joint_map.reload()


SAMPLE = {
    "right": {"sign": [1, -1, -1, 1, -1, -1, -1], "offset": [0, 0, 90, 0, -90, 0, 180]},
    "left":  {"sign": [-1, 1, 1, 1, 1, -1, 1],    "offset": [0, 0, -90, 0, 90, 0, -180]},
}


def main():
    q = np.array([10., -20., 30., -40., 50., -15., 25.])

    print("[1] 缺 json → 恒等映射")
    _use_map_file(None)
    check(not joint_map.is_active(), "is_active() == False")
    check(np.allclose(joint_map.urdf_to_sdk('right', q), q), "urdf_to_sdk 恒等")
    check(np.allclose(joint_map.sdk_to_urdf('left', q), q), "sdk_to_urdf 恒等")

    print("[2] 有 json → 数值正确 + 往返一致")
    _use_map_file(SAMPLE)
    check(joint_map.is_active(), "is_active() == True")
    for arm in ('right', 'left'):
        sign = np.array(SAMPLE[arm]['sign'], float)
        off = np.array(SAMPLE[arm]['offset'], float)
        expect = sign * q + off
        got = joint_map.urdf_to_sdk(arm, q)
        check(np.allclose(got, expect), f"[{arm}] urdf_to_sdk = sign*q+offset")
        back = joint_map.sdk_to_urdf(arm, got)
        check(np.allclose(back, q), f"[{arm}] sdk_to_urdf∘urdf_to_sdk == 恒等")

    print("[3] 非法 json（sign 含非 ±1）→ 退回恒等，不抛")
    bad = {"right": {"sign": [1, 2, 1, 1, 1, 1, 1], "offset": [0]*7},
           "left":  {"sign": [1]*7, "offset": [0]*7}}
    try:
        _use_map_file(bad)
        ok = not joint_map.is_active()
    except Exception:
        ok = False
    check(ok, "非法 sign → is_active()==False 且未抛异常")

    print("[4] 非法 json（维度错）→ 退回恒等，不抛")
    bad2 = {"right": {"sign": [1]*6, "offset": [0]*6},
            "left":  {"sign": [1]*7, "offset": [0]*7}}
    try:
        _use_map_file(bad2)
        ok = not joint_map.is_active()
    except Exception:
        ok = False
    check(ok, "维度错 → is_active()==False 且未抛异常")

    os.environ.pop('JOINT_MAP_FILE', None)
    print("\n" + "=" * 48)
    print(f"  结果: {_PASS} 通过, {_FAIL} 失败")
    print("=" * 48)
    sys.exit(1 if _FAIL else 0)


if __name__ == '__main__':
    main()
