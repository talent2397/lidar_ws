#!/bin/bash
# ==========================================
#  免 sudo 安装 Foxglove 桥 (本地解包 deb, 不改系统)
#  产物: tools/foxglove_bridge/rootfs (已被 .gitignore 排除)
#  重复运行会重新下载覆盖
# ==========================================
set -e

TOOLS_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG=ros-humble-foxglove-bridge
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

cd "$TMP"
apt-get download "$PKG"
dpkg-deb -x "$PKG"*.deb rootfs

mkdir -p "$TOOLS_DIR/foxglove_bridge"
rm -rf "$TOOLS_DIR/foxglove_bridge/rootfs"
mv rootfs "$TOOLS_DIR/foxglove_bridge/rootfs"

echo "OK: $TOOLS_DIR/foxglove_bridge/rootfs"
