#!/usr/bin/env bash
# package_sdk.sh — 将 sdk/ 打包为可独立分发的副本
#
# 用法：
#   bash tools/package_sdk.sh                  # 输出到 /tmp/sdk_dist/
#   bash tools/package_sdk.sh ./dist           # 输出到 ./dist/
#   bash tools/package_sdk.sh ./dist --zip     # 同时生成 sdk_dist.zip
#
# 效果：
#   - 展开所有符号链接（src/ demo/ 变为真实文件）
#   - 剔除 __pycache__、*.pyc、build/、install/、log/ 等
#   - 可选打包为 zip，直接发给算法同事

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_SRC="$REPO_DIR/sdk"
OUT_DIR="${1:-/tmp/sdk_dist}"
DO_ZIP=false
[[ "${2:-}" == "--zip" ]] && DO_ZIP=true

if [ ! -d "$SDK_SRC" ]; then
    echo "错误：sdk/ 目录不存在（$SDK_SRC）" >&2
    exit 1
fi

# 清理旧输出
rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"

# 展开符号链接复制（-rL：递归 + follow symlinks）
cp -rL "$SDK_SRC/." "$OUT_DIR/"

# 清理不需要分发的内容
find "$OUT_DIR" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find "$OUT_DIR" -name "*.pyc" -o -name "*.pyo" | xargs rm -f 2>/dev/null || true
find "$OUT_DIR" -type d \( -name "build" -o -name "install" -o -name "log" \) \
  -exec rm -rf {} + 2>/dev/null || true
rm -f "$OUT_DIR/fastdds_unicast_profile.xml" 2>/dev/null || true  # 只保留 .template

echo "✅  SDK 已生成：$OUT_DIR"
echo ""
echo "目录结构："
find "$OUT_DIR" -not -path '*/__pycache__/*' | sort | sed "s|$OUT_DIR||" | sed 's|^/||'

if $DO_ZIP; then
    ZIP_FILE="$(dirname "$OUT_DIR")/sdk_dist.zip"
    (cd "$(dirname "$OUT_DIR")" && zip -qr "$ZIP_FILE" "$(basename "$OUT_DIR")")
    echo ""
    echo "✅  压缩包：$ZIP_FILE"
fi
