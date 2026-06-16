#!/usr/bin/env python3
"""
RealSense 3摄像头压力测试（1×D435 头部 + 2×D405 腕部）

评估 Jetson 算力与 USB 带宽能否同时稳定驱动3个 RealSense 摄像头。

用法：
    python3 camera_stress_test.py                      # 默认 640×480 30fps 30s
    python3 camera_stress_test.py --res high --fps 30  # 1280×720 30fps
    python3 camera_stress_test.py --res low  --fps 60  # 640×480  60fps
    python3 camera_stress_test.py --res high --fps 60  # 1280×720 60fps（可能不支持）
    python3 camera_stress_test.py --no-color           # 仅深度流
    python3 camera_stress_test.py --duration 60        # 测试60秒
"""

import sys, os, time, threading, argparse, glob, signal
from collections import deque

PIPELINE_START_TIMEOUT = 8   # 单个相机初始化超时秒数

class _PipelineTimeout(Exception):
    pass

def _alarm_handler(signum, frame):
    raise _PipelineTimeout()

# ── 依赖检查 ──────────────────────────────────────────────────────────────────
try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit("❌  pyrealsense2 未安装")

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False
    print("⚠   psutil 未安装，CPU/内存监控不可用。pip3 install psutil\n")

# ── 常量 ──────────────────────────────────────────────────────────────────────
FPS_PASS_RATIO = 0.90

RESOLUTIONS = {
    "low":  (640,  480),
    "high": (1280, 720),
}

# ── Jetson 温度 ────────────────────────────────────────────────────────────────
def read_temps():
    temps = {}
    for path in sorted(glob.glob("/sys/class/thermal/thermal_zone*/temp")):
        try:
            zone = open(path.replace("temp","type")).read().strip()
            val  = int(open(path).read().strip()) / 1000.0
            if 10.0 < val < 120.0:
                temps[zone] = val
        except Exception:
            pass
    if not temps:
        return "N/A"
    top = sorted(temps.items(), key=lambda x: -x[1])[:3]
    return "  ".join(f"{k}:{v:.0f}°C" for k, v in top)


# ── 采帧线程（只做采帧统计，不做初始化）─────────────────────────────────────
class GrabWorker(threading.Thread):
    def __init__(self, pipeline, label, target_fps):
        super().__init__(daemon=True)
        self.pipeline    = pipeline
        self.label       = label
        self.target_fps  = target_fps
        self._lock       = threading.Lock()
        self._fps_buf    = deque(maxlen=10)
        self._frames     = 0
        self._timeouts   = 0
        self._stop       = threading.Event()

    def run(self):
        t_last, cnt = time.time(), 0
        while not self._stop.is_set():
            try:
                self.pipeline.wait_for_frames(timeout_ms=2000)
                with self._lock:
                    self._frames += 1
                cnt += 1
            except RuntimeError:
                with self._lock:
                    self._timeouts += 1
                continue

            now = time.time()
            if now - t_last >= 1.0:
                with self._lock:
                    self._fps_buf.append(cnt / (now - t_last))
                cnt, t_last = 0, now

    def stop(self):
        self._stop.set()

    @property
    def stats(self):
        with self._lock:
            fps    = sum(self._fps_buf)/len(self._fps_buf) if self._fps_buf else 0.0
            return fps, self._frames, self._timeouts


# ── 主程序 ────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="RealSense 3摄像头压力测试")
    parser.add_argument("--duration", type=int, default=30)
    parser.add_argument("--res",  choices=["low","high"], default="low")
    parser.add_argument("--fps",  type=int, default=30)
    parser.add_argument("--no-color",  action="store_true", help="仅深度流")
    parser.add_argument("--color-only", action="store_true", help="仅彩色流（不开深度）")
    args = parser.parse_args()

    width, height = RESOLUTIONS[args.res]
    enable_color  = not args.no_color
    enable_depth  = not args.color_only

    # ── 1. 枚举设备 ──
    print("\n🔍  扫描 RealSense 设备...", flush=True)
    ctx  = rs.context()
    devs = list(ctx.query_devices())
    if not devs:
        sys.exit("❌  未找到设备，请检查 USB 连接")

    print(f"    找到 {len(devs)} 个：")
    for d in devs:
        name   = d.get_info(rs.camera_info.name)
        serial = d.get_info(rs.camera_info.serial_number)
        usb    = d.get_info(rs.camera_info.usb_type_descriptor) if d.supports(rs.camera_info.usb_type_descriptor) else "?"
        kind   = "D435" if "D435" in name or "D455" in name else ("D405" if "D405" in name else "Unknown")
        print(f"    [{kind}] {name}  SN:{serial}  USB:{usb}")

    if args.color_only:
        streams = "仅彩色"
    elif args.no_color:
        streams = "仅深度"
    else:
        streams = "深度 + 彩色"
    # 估算 USB 带宽占用
    depth_bw = width * height * 2 * args.fps / 1e6   # z16, MB/s
    color_bw = width * height * 3 * args.fps / 1e6   # bgr8, MB/s
    per_cam  = (depth_bw if enable_depth else 0) + (color_bw if enable_color else 0)
    total_bw = per_cam * len(devs)
    bw_warn  = "⚠  可能超出 USB 带宽！" if total_bw > 350 else "✓ 带宽估算正常"
    print(f"\n▶   参数：{width}×{height}  {args.fps}fps  {streams}  {args.duration}s", flush=True)
    print(f"    合格线：≥ {args.fps * FPS_PASS_RATIO:.0f} fps", flush=True)
    print(f"    带宽估算：每路 {per_cam:.0f} MB/s × {len(devs)} = {total_bw:.0f} MB/s  {bw_warn}\n", flush=True)

    # ── 2. 在主线程顺序初始化每个相机（避免多线程初始化竞争）──
    workers   = []
    pipelines = []
    d435_n = d405_n = 0

    for d in devs:
        name   = d.get_info(rs.camera_info.name)
        serial = d.get_info(rs.camera_info.serial_number)
        kind   = "D435" if "D435" in name or "D455" in name else ("D405" if "D405" in name else "Unknown")
        if kind == "D435":
            d435_n += 1
            label = f"D435-{d435_n}(头部)"
        elif kind == "D405":
            d405_n += 1
            label = f"D405-{d405_n}(腕部)"
        else:
            label = f"{kind}-{serial[-4:]}"

        print(f"    初始化 {label} ...", end=" ", flush=True)
        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        if enable_depth:
            cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16,   args.fps)
        if enable_color:
            cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, args.fps)

        # 用 SIGALRM 做超时（Linux 专用）：OS 级信号可打断卡死的 C++ 调用
        signal.signal(signal.SIGALRM, _alarm_handler)
        signal.alarm(PIPELINE_START_TIMEOUT)
        try:
            pipeline.start(cfg)
            signal.alarm(0)   # 取消闹钟
            print("✅  OK", flush=True)
            pipelines.append(pipeline)
            workers.append(GrabWorker(pipeline, label, args.fps))
        except _PipelineTimeout:
            print(f"❌  超时（>{PIPELINE_START_TIMEOUT}s），USB 带宽不足，跳过", flush=True)
        except Exception as e:
            signal.alarm(0)
            print(f"❌  失败: {e}", flush=True)
        finally:
            signal.alarm(0)   # 确保闹钟被取消

    if not workers:
        sys.exit("\n所有相机初始化失败，退出")

    # ── 3. 启动采帧线程 ──
    print(f"\n    {len(workers)} 路相机就绪，开始采集...\n", flush=True)
    for w in workers:
        w.start()

    # ── 4. 监控循环 ──
    t_start  = time.time()
    n_report = 0

    try:
        while True:
            elapsed = time.time() - t_start
            if elapsed >= args.duration:
                break

            if n_report % 10 == 0:
                hdr = f"  {'摄像头':<18} {'FPS':>6} {'总帧':>7} {'超时':>5}"
                if HAS_PSUTIL:
                    hdr += f"  {'CPU%':>5} {'内存%':>6}"
                hdr += "  温度"
                print(hdr, flush=True)
                print("  " + "-"*70, flush=True)
            n_report += 1

            cpu = f"{psutil.cpu_percent(interval=None):5.1f}%" if HAS_PSUTIL else ""
            mem = f"{psutil.virtual_memory().percent:5.1f}%"   if HAS_PSUTIL else ""
            tmp = read_temps()

            for i, w in enumerate(workers):
                fps, frames, drops = w.stats
                ok  = "✓" if fps >= args.fps * FPS_PASS_RATIO else "✗"
                row = f"  {w.label:<18} {fps:6.1f}{ok} {frames:7d} {drops:5d}"
                if HAS_PSUTIL:
                    row += f"  {cpu if i==0 else '      '} {mem if i==0 else '      '}"
                row += f"  {tmp if i==0 else ''}"
                print(row, flush=True)

            print(f"  ── 剩余 {args.duration - elapsed:.0f}s\n", flush=True)
            time.sleep(2.0)

    except KeyboardInterrupt:
        print("\n⚠   用户中断", flush=True)

    # ── 5. 汇总 ──
    total = max(time.time() - t_start, 0.1)
    print(f"\n{'='*60}", flush=True)
    print("📊  测试汇总", flush=True)
    print(f"{'='*60}", flush=True)

    all_pass = True
    for w in workers:
        fps, frames, drops = w.stats
        avg  = frames / total
        ok   = avg >= args.fps * FPS_PASS_RATIO and drops == 0
        if not ok:
            all_pass = False
        status = "✅ 通过" if ok else "❌ 失败"
        print(f"  {status}  {w.label:<20} 平均:{avg:5.1f}/{args.fps}fps  总帧:{frames:6d}  超时:{drops}", flush=True)
        w.stop()

    for p in pipelines:
        try:
            p.stop()
        except Exception:
            pass

    if HAS_PSUTIL:
        print(f"\n  CPU: {psutil.cpu_percent():.1f}%  内存: {psutil.virtual_memory().percent:.1f}%", flush=True)
    print(f"  温度: {read_temps()}", flush=True)
    print(f"\n  {'='*56}", flush=True)
    if all_pass:
        print(f"  ✅  Jetson 可稳定驱动 {len(workers)} 路摄像头", flush=True)
    else:
        print(f"  ❌  存在问题，建议降低分辨率/帧率或检查 USB 分布", flush=True)
    print(f"  {'='*56}\n", flush=True)


if __name__ == "__main__":
    main()
