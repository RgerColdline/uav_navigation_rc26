#!/bin/bash

# 监控 rosout.log 的脚本
# 日志来源：~/.ros/log/latest/rosout.log

LOG_FILE="$HOME/.ros/log/latest/rosout.log"

# 检查日志文件是否存在
if [ ! -f "$LOG_FILE" ]; then
    echo "错误：日志文件 $LOG_FILE 不存在"
    echo "请确保 ROS 节点正在运行"
    exit 1
fi

echo "=== 开始监控 rosout ==="
echo "日志文件：$LOG_FILE"
echo "按 Ctrl+C 停止监控"
echo "=============================================="

# 清理格式函数
clean_log() {
    sed -u 's/^[0-9]*\.[0-9]* //' | \
    sed -u 's/ \[\/home[^]]*//' | \
    sed -u 's/ \[tmp\/binarydeb[^]]*//' | \
    sed -u 's/ \[topics: [^]]*\]//'
}

# 1. 先显示历史日志中这两个节点的所有记录
echo ""
echo ">>> 历史日志："
echo "-------------------------------------------"
grep -E "/demo_fsm_node|/ego_controller_node" "$LOG_FILE" | clean_log

# 2. 实时监控新日志
echo ""
echo ">>> 实时监控："
echo "-------------------------------------------"
tail -F "$LOG_FILE" 2>/dev/null | \
    grep --line-buffered -E "/demo_fsm_node|/ego_controller_node" | \
    clean_log
