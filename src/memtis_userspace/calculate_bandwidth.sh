#!/bin/bash

# 计算带宽的脚本
# 用法: ./calculate_bandwidth.sh <log_file_path>

if [ $# -ne 1 ]; then
    echo "用法: $0 <log_file_path>"
    echo "示例: $0 /path/to/zipfan_first_touch_27G.read.log"
    exit 1
fi

LOG_FILE="$1"

if [ ! -f "$LOG_FILE" ]; then
    echo "错误: 文件 '$LOG_FILE' 不存在"
    exit 1
fi

# 提取所有带宽值
BANDWIDTH_VALUES=$(grep "Bandwidth(MB/s)" "$LOG_FILE" | sed 's/.*\[\([0-9.]*\)\].*/\1/')

if [ -z "$BANDWIDTH_VALUES" ]; then
    echo "错误: 在文件中未找到带宽数据"
    exit 1
fi

# 计算平均值
AVERAGE_BANDWIDTH=$(echo "$BANDWIDTH_VALUES" | awk '{
    sum += $1
    count++
} END {
    if (count > 0) {
        printf "%.6f", sum / count
    } else {
        printf "0"
    }
}')

# 输出结果（仅保留平均带宽）
echo "============================="
echo "带宽平均值"
echo "============================="
echo "文件: $LOG_FILE"
echo "平均带宽: ${AVERAGE_BANDWIDTH} MB/s"
echo "============================="

# 将结果追加到日志文件（仅平均带宽）
STATS_TEXT="
=============================
带宽平均值
=============================
平均带宽: ${AVERAGE_BANDWIDTH} MB/s
============================="

# 尝试直接写入，如果失败则使用sudo
if echo "$STATS_TEXT" >> "$LOG_FILE" 2>/dev/null; then
    echo "结果已追加到日志文件: $LOG_FILE"
else
    echo "需要sudo权限来写入文件，尝试使用sudo..."
    # 使用sudo tee写入文件，如果失败则提示用户
    if echo "$STATS_TEXT" | sudo tee -a "$LOG_FILE" > /dev/null 2>&1; then
        echo "结果已追加到日志文件: $LOG_FILE"
    else
        echo "警告: 无法自动写入文件，可能需要手动执行以下命令："
        echo "echo '$STATS_TEXT' | sudo tee -a '$LOG_FILE'"
        echo "或者确保脚本以root权限运行"
    fi
fi
