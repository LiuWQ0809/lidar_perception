#!/bin/bash

# Fusion Perception C++ - Stop Script
# 停止脚本

PROJECT_DIR="/home/nvidia/liuwq/fusion_cpp"
PROCESS_NAME="fusion_perception_node"
PID_FILE="$PROJECT_DIR/.fusion_node.pid"

echo "========================================"
echo "Stopping Fusion Perception Node"
echo "========================================"
echo ""

# 先尝试使用保存的PID文件
if [ -f "$PID_FILE" ]; then
    SAVED_PID=$(cat "$PID_FILE" 2>/dev/null)
    if [ ! -z "$SAVED_PID" ] && kill -0 "$SAVED_PID" 2>/dev/null; then
        echo "Killing process with saved PID: $SAVED_PID"
        kill -9 "$SAVED_PID" 2>/dev/null
        rm -f "$PID_FILE"
        sleep 1
    fi
fi

# 再通过进程名查找任何残留进程
EXISTING_PIDS=$(pgrep -f "$PROCESS_NAME")
if [ ! -z "$EXISTING_PIDS" ]; then
    echo "Found remaining $PROCESS_NAME process(es):"
    echo "$EXISTING_PIDS"
    echo "Killing all remaining processes..."
    echo "$EXISTING_PIDS" | xargs kill -9 2>/dev/null
    sleep 1
fi

# 最后验证进程是否都已被清理
if pgrep -f "$PROCESS_NAME" > /dev/null; then
    echo "Warning: Some processes may still be running. Please check manually."
    pgrep -f "$PROCESS_NAME"
else
    echo "Successfully stopped all Fusion Perception processes."
fi

echo ""
echo "========================================"
