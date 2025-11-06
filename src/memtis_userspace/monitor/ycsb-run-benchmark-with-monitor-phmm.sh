#!/bin/bash

# 整合脚本：运行 YCSB 基准测试
# 用法: ./ycsb-run-benchmark-with-monitor-phmm.sh [参数]

INTERVAL=${1:-1}  # 参数（保留用于兼容性）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCHMARK_SCRIPT="$SCRIPT_DIR/../hottest_10G_read-phmm.sh"

# 全局变量用于存储进程ID
BENCHMARK_PID=""

# 信号处理函数
cleanup() {
    echo ""
    echo "=========================================="
    echo "检测到中断信号，正在停止所有进程..."
    
    # 停止基准测试脚本
    if [ -n "$BENCHMARK_PID" ] && kill -0 "$BENCHMARK_PID" 2>/dev/null; then
        echo "正在停止基准测试脚本 (PID: $BENCHMARK_PID)..."
        kill -INT "$BENCHMARK_PID" 2>/dev/null
        sleep 1
        if kill -0 "$BENCHMARK_PID" 2>/dev/null; then
            kill -TERM "$BENCHMARK_PID" 2>/dev/null
            sleep 1
            if kill -0 "$BENCHMARK_PID" 2>/dev/null; then
                kill -KILL "$BENCHMARK_PID" 2>/dev/null
            fi
        fi
        echo "基准测试脚本已停止"
    fi
    
    echo "所有进程已停止"
    echo "=========================================="
    exit 130
}

# 注册信号处理器
trap cleanup INT TERM

# 检查必要文件是否存在
if [ ! -f "$BENCHMARK_SCRIPT" ]; then
    echo "错误: 基准测试脚本不存在: $BENCHMARK_SCRIPT"
    exit 1
fi

# 确保脚本有执行权限
chmod +x "$BENCHMARK_SCRIPT"

echo "=========================================="
echo "开始运行 YCSB 基准测试"
echo "=========================================="
echo "基准测试脚本: $BENCHMARK_SCRIPT"
echo "=========================================="

# 运行基准测试
echo "正在启动 YCSB 基准测试..."
echo "命令: sudo $BENCHMARK_SCRIPT"
echo "=========================================="

# 记录开始时间
START_TIME=$(date '+%Y-%m-%d %H:%M:%S')
echo "基准测试开始时间: $START_TIME"

# 运行基准测试（后台运行）
sudo "$BENCHMARK_SCRIPT" &
BENCHMARK_PID=$!

# 等待基准测试完成
wait $BENCHMARK_PID
BENCHMARK_EXIT_CODE=$?

# 记录结束时间
END_TIME=$(date '+%Y-%m-%d %H:%M:%S')
echo "基准测试结束时间: $END_TIME"

echo "=========================================="
echo "基准测试完成"
echo "开始时间: $START_TIME"
echo "结束时间: $END_TIME"
echo "退出代码: $BENCHMARK_EXIT_CODE"

if [ $BENCHMARK_EXIT_CODE -eq 0 ]; then
    echo "基准测试执行成功"
else
    echo "基准测试执行失败"
fi

echo "=========================================="

exit $BENCHMARK_EXIT_CODE