#!/bin/bash

# NUMA Node0 命中情况监控脚本
# 专门监控 node0 的 numa_hit 和 numa_foreign
# 用法: ./monitor-hit.sh [间隔秒数] [数据目录] [日志文件名] [静默模式]

INTERVAL=${1:-1}  # 默认1秒间隔
DATA_DIR=${2:-"/home/mrgeek/hmm/memtis/memtis-userspace/monitor/data"}
LOG_FILENAME=${3:-"numa_node0_$(date +%Y%m%d_%H%M%S).log"}
SILENT_MODE=${4:-"true"}  # 默认非静默模式
LOG_FILE="$DATA_DIR/$LOG_FILENAME"

if [[ "$SILENT_MODE" != "true" ]]; then
    echo "开始监控 Node0 NUMA 命中情况..."
    echo "监控间隔: ${INTERVAL}秒"
    echo "日志文件: ${LOG_FILE}"
    echo "按 Ctrl+C 停止监控"
    echo "=========================================="
fi

# 创建日志文件头
if ! echo "时间戳,本地命中(numa_hit),远程命中(numa_foreign),本地访问率(%),远程访问率(%)" > "$LOG_FILE" 2>/dev/null; then
    echo "错误: 无法创建日志文件 $LOG_FILE"
    echo "请检查目录权限或使用sudo运行脚本"
    exit 1
fi

# 获取初始值
get_numa_stats() {
    local hit=$(grep "numa_hit" /sys/devices/system/node/node0/numastat | awk '{print $2}')
    local foreign=$(grep "numa_foreign" /sys/devices/system/node/node0/numastat | awk '{print $2}')
    echo "$hit $foreign"
}

# 计算命中率
calculate_rates() {
    local hit=$1
    local foreign=$2
    local total=$((hit + foreign))
    
    if [ $total -gt 0 ]; then
        local local_rate=$((hit * 100 / total))
        local remote_rate=$((foreign * 100 / total))
        echo "$local_rate $remote_rate"
    else
        echo "0 0"
    fi
}

# 实时监控函数
monitor_numa() {
    local prev_hit=0
    local prev_foreign=0
    
    while true; do
        TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
        
        # 获取当前统计值
        read -r current_hit current_foreign <<< $(get_numa_stats)
        
        # 计算增量
        local hit_delta=$((current_hit - prev_hit))
        local foreign_delta=$((current_foreign - prev_foreign))
        
        # 计算命中率
        read -r local_rate remote_rate <<< $(calculate_rates $hit_delta $foreign_delta)
        
        # 显示当前状态
        if [[ "$SILENT_MODE" != "true" ]]; then
            clear
            echo "时间: $TIMESTAMP"
            echo "=========================================="
            echo "Node0 NUMA 统计 (每秒新增访问次数):"
            echo "  本地命中: $(numfmt --to=iec $hit_delta) 次/秒"
            echo "  远程命中: $(numfmt --to=iec $foreign_delta) 次/秒"
            echo "  本地访问率: ${local_rate}%"
            echo "  远程访问率: ${remote_rate}%"
            echo ""
            echo "Node0 NUMA 统计 (累计访问次数):"
            echo "  本地命中: $(numfmt --to=iec $current_hit) 次"
            echo "  远程命中: $(numfmt --to=iec $current_foreign) 次"
            echo ""
            echo "日志文件: $LOG_FILE"
            echo "按 Ctrl+C 停止监控"
        fi
        
        # 记录到日志文件
        if ! echo "$TIMESTAMP,$hit_delta,$foreign_delta,$local_rate,$remote_rate" >> "$LOG_FILE" 2>/dev/null; then
            echo "错误: 无法写入日志文件 $LOG_FILE"
            echo "监控停止"
            exit 1
        fi
        
        # 更新前一次的值
        prev_hit=$current_hit
        prev_foreign=$current_foreign
        
        sleep "$INTERVAL"
    done
}

# 捕获中断信号
if [[ "$SILENT_MODE" != "true" ]]; then
    trap 'echo -e "\n监控已停止"; echo "日志文件保存在: $LOG_FILE"; exit 0' INT
else
    trap 'exit 0' INT
fi

# 开始监控
monitor_numa
