#!/bin/bash
# Foxglove Studio 启动脚本 (Jetson 需禁用 GPU 进程, 否则窗口白屏/起不来)
exec /opt/Foxglove\ Studio/foxglove-studio --disable-gpu "$@"
