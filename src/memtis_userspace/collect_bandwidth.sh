#!/bin/bash

# 收集CPU带宽脚本
# 执行 pqos -I -m 监控，每秒收集一次所有CPU的local和remote带宽之和
# 输出格式：时间(秒) local_bandwidth(MB/s) remote_bandwidth(MB/s)

# 检查pqos命令是否存在
if ! command -v pqos &> /dev/null; then
    echo "错误: pqos 命令未找到，请确保已安装 Intel RDT工具" >&2
    exit 1
fi

# 默认输出文件
OUTPUT_FILE="${1:-bandwidth_data.txt}"

# 清理函数
cleanup() {
    echo ""
    echo "=========================================="
    echo "正在停止监控..."
    echo "=========================================="
    # 停止pqos监控
    sudo pqos -I -r 2>/dev/null || true
    # 确保文件缓冲区刷新到磁盘
    if [ -f "$OUTPUT_FILE" ]; then
        sync "$OUTPUT_FILE" 2>/dev/null || true
    fi
    exit 0
}

# 注册信号处理器
trap cleanup INT TERM

# 在启动监控之前，先停止任何现有的监控
echo "检查并停止现有监控..."
sudo pqos -I -r 2>/dev/null || true
sleep 1  # 等待一下确保监控完全停止

# 配置监控命令（每次采样 1 秒，返回 CSV 格式，便于解析）
PQOS_CMD=(sudo pqos -I -m 'mbl:96-143;mbr:96-143' -t 1 -u csv)

echo "=========================================="
echo "启动 pqos 监控..."
echo "监控CPU: 96-143"
echo "输出文件: $OUTPUT_FILE"
echo "按 Ctrl+C 停止监控"
echo "=========================================="

# 创建输出文件并写入表头
echo "time_sec	local_bandwidth_mbps	remote_bandwidth_mbps" > "$OUTPUT_FILE"

# 记录开始时间
START_TIME=$(date +%s)

echo "开始收集数据..."
echo ""

# 循环收集数据
while true; do
    # 获取当前时间（相对于开始时间的秒数）
    CURRENT_TIME=$(date +%s)
    ELAPSED_TIME=$((CURRENT_TIME - START_TIME))
    
    # 读取 pqos 监控数据（命令自身等待约 1 秒）
    PQOS_OUTPUT=$("${PQOS_CMD[@]}" 2>/dev/null)
    
    if [ $? -eq 0 ] && [ -n "$PQOS_OUTPUT" ]; then
        # 初始化累加值
        TOTAL_MBL=0
        TOTAL_MBR=0
        
        # 解析 pqos 输出（CSV 格式）并累加 MBL/MBR
        READ_RESULT=$(echo "$PQOS_OUTPUT" \
            | awk -F',' '
                $1 ~ /^NOTE/ { next }
                NR == 1 { next }
                NF >= 6 {
                    mbl = $5 + 0
                    mbr = $6 + 0
                    total_mbl += mbl
                    total_mbr += mbr
                    data_rows++
                }
                END {
                    if (data_rows > 0) {
                        printf "%.2f %.2f", total_mbl, total_mbr
                    }
                }
            ')

        if [ -n "$READ_RESULT" ]; then
            read -r TOTAL_MBL TOTAL_MBR <<< "$READ_RESULT"
            TOTAL_MBL=${TOTAL_MBL:-0}
            TOTAL_MBR=${TOTAL_MBR:-0}
        else
            TOTAL_MBL=0
            TOTAL_MBR=0
        fi
        
        # 确保有有效的数值（如果为空则设为0）
        if [ -z "$TOTAL_MBL" ]; then
            TOTAL_MBL=0
        fi
        if [ -z "$TOTAL_MBR" ]; then
            TOTAL_MBR=0
        fi
        
        # 输出到文件（使用制表符分隔，方便后续绘图）
        printf "%d\t%.2f\t%.2f\n" "$ELAPSED_TIME" "$TOTAL_MBL" "$TOTAL_MBR" >> "$OUTPUT_FILE"
        # 立即刷新文件缓冲区，确保数据写入磁盘
        sync "$OUTPUT_FILE" 2>/dev/null || true
        
        # 同时输出到控制台
        printf "[%ds] Local: %.2f MB/s, Remote: %.2f MB/s\n" "$ELAPSED_TIME" "$TOTAL_MBL" "$TOTAL_MBR"
    else
        # 如果读取失败，输出0值
        printf "%d\t%.2f\t%.2f\n" "$ELAPSED_TIME" "0.00" "0.00" >> "$OUTPUT_FILE"
        # 立即刷新文件缓冲区
        sync "$OUTPUT_FILE" 2>/dev/null || true
        printf "[%ds] 警告: 无法读取pqos数据\n" "$ELAPSED_TIME" >&2
    fi
done

