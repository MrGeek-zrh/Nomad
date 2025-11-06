#!/bin/bash

# 使用方法:
#   1) 仅指定 DRAM 大小
#      ./ycsb-run-benchmark-with-monitor-tpp-no-perf.sh 10G
#   2) 同时指定监控间隔(秒) 与 DRAM 大小
#      ./ycsb-run-benchmark-with-monitor-tpp-no-perf.sh 1 10G
# 参数解析规则:
#   - 若第一个参数为纯数字，则视为监控间隔(秒)，第二个参数为 DRAM 大小（可选）
#   - 若仅传入一个非数字参数，则视为 DRAM 大小，监控间隔默认 1 秒

# 整合脚本：运行 YCSB 基准测试
# 用法: ./ycsb-run-benchmark-with-monitor-tpp-no-perf.sh [DRAM_SIZE]
# 示例: ./ycsb-run-benchmark-with-monitor-tpp-no-perf.sh 10G

# 参数解析：若仅传入一个非数字参数，则视为 DRAM 大小，间隔默认 1；
# 若第一个参数为数字，则作为间隔，第二个参数作为 DRAM 大小
if [[ -z "$1" ]]; then
    INTERVAL=1
    DRAM_SIZE_ARG=""
elif [[ "$1" =~ ^[0-9]+$ ]]; then
    INTERVAL=$1
    DRAM_SIZE_ARG=${2:-}
else
    INTERVAL=1
    DRAM_SIZE_ARG=$1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCHMARK_SCRIPT="$SCRIPT_DIR/../hottest_10G_read-tpp.sh"
CALC_SCRIPT="$SCRIPT_DIR/../calculate_pgmetrics.sh"
CALC_BW="$SCRIPT_DIR/../calculate_bandwidth.sh"
WORKING_DIR="$SCRIPT_DIR/.."

# 确定输出路径（与现有的 results 格式一致）
MODE_DIR="tpp"
RESULT_BASE="/home/iscas/hmm/Nomad/src/memtis_userspace/results/ycsb/10G-hottest/read/run-memtis-10g-read/microbench/$MODE_DIR"
if [ -n "$DRAM_SIZE_ARG" ]; then
    RESULT_BASE="$RESULT_BASE/$DRAM_SIZE_ARG"
    CSV_SUFFIX="$DRAM_SIZE_ARG"
else
    CSV_SUFFIX="10G"
fi

# 确保输出目录存在
mkdir -p "$RESULT_BASE"

# 结果日志文件
RESULT_LOG="$RESULT_BASE/zipfan_hottest_10G.read.log"

# 检查必要文件是否存在
missing_components=()

if [ ! -f "$BENCHMARK_SCRIPT" ]; then
    missing_components+=("基准测试脚本 ($BENCHMARK_SCRIPT)")
fi

if [ ! -f "$CALC_SCRIPT" ]; then
    missing_components+=("calculate_pgmetrics.sh ($CALC_SCRIPT)")
fi

if [ ! -f "$CALC_BW" ]; then
    missing_components+=("calculate_bandwidth.sh ($CALC_BW)")
fi

if [ ${#missing_components[@]} -ne 0 ]; then
    echo "错误: 以下必需组件缺失:" >&2
    for item in "${missing_components[@]}"; do
        echo "  - $item" >&2
    done
    exit 1
fi

# 确保脚本有执行权限
chmod +x "$BENCHMARK_SCRIPT"

# 确保结果目录存在
mkdir -p "$RESULT_BASE"

echo "=========================================="
echo "开始运行 YCSB 基准测试"
echo "=========================================="
echo "基准测试脚本: $BENCHMARK_SCRIPT"
echo "DRAM 大小: ${DRAM_SIZE_ARG:-未指定}"
echo "输出路径: $RESULT_BASE"
echo "=========================================="

# 记录开始时间
START_TIME=$(date '+%Y-%m-%d %H:%M:%S')
echo "基准测试开始时间: $START_TIME"

# 构建要执行的命令（使用绝对路径确保正确执行）
if [ -n "$DRAM_SIZE_ARG" ]; then
    RUN_CMD="cd $WORKING_DIR && sudo env DRAM_SIZE=$DRAM_SIZE_ARG $BENCHMARK_SCRIPT"
else
    RUN_CMD="cd $WORKING_DIR && sudo env DRAM_SIZE= $BENCHMARK_SCRIPT"
fi

# 直接运行基准测试
echo "正在启动基准测试..."
echo "命令: $RUN_CMD"
echo "=========================================="

if eval "$RUN_CMD"; then
    BENCHMARK_EXIT_CODE=0
else
    BENCHMARK_EXIT_CODE=$?
fi

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
    echo "输出目录: $RESULT_BASE"
    # 注意：原有的输出（output.log, before_vmstat.log, after_vmstat.log 等）会输出到 results/tpp/ 目录
else
    echo "基准测试执行失败"
fi

echo "=========================================="

# 追加迁移统计到 hottest 输出日志
BEFORE_VMSTAT="$RESULT_BASE/before_vmstat.log"
AFTER_VMSTAT="$RESULT_BASE/after_vmstat.log"
TARGET_OUTPUT_LOG="$RESULT_BASE/output.log"

if [ -x "$CALC_SCRIPT" ] && [ -f "$TARGET_OUTPUT_LOG" ] && [ -f "$BEFORE_VMSTAT" ] && [ -f "$AFTER_VMSTAT" ]; then
    {
        echo ""
        echo "========================================="
        echo "PG 迁移统计"
        "$CALC_SCRIPT" "$BEFORE_VMSTAT" "$AFTER_VMSTAT"
    } | sudo tee -a "$TARGET_OUTPUT_LOG" >/dev/null
else
    echo ""
    echo "========================================="
    echo "PG 迁移统计未执行，原因如下："
    if [ ! -x "$CALC_SCRIPT" ]; then
        echo "- 统计脚本不可执行或不存在: $CALC_SCRIPT"
    fi
    if [ ! -f "$TARGET_OUTPUT_LOG" ]; then
        echo "- 目标日志缺失: $TARGET_OUTPUT_LOG"
    fi
    if [ ! -f "$BEFORE_VMSTAT" ]; then
        echo "- before_vmstat.log 缺失: $BEFORE_VMSTAT"
    fi
    if [ ! -f "$AFTER_VMSTAT" ]; then
        echo "- after_vmstat.log 缺失: $AFTER_VMSTAT"
    fi
fi

if [ -x "$CALC_BW" ] && [ -f "$TARGET_OUTPUT_LOG" ] && [ -f "$RESULT_LOG" ]; then
    {
        echo ""
        echo "========================================="
        echo "带宽平均值"
        "$CALC_BW" "$RESULT_LOG"
    } | sudo tee -a "$TARGET_OUTPUT_LOG" >/dev/null
else
    echo ""
    echo "========================================="
    echo "带宽平均值未计算，原因如下："
    if [ ! -x "$CALC_BW" ]; then
        echo "- 带宽脚本不可执行或不存在: $CALC_BW"
    fi
    if [ ! -f "$TARGET_OUTPUT_LOG" ]; then
        echo "- 目标日志缺失: $TARGET_OUTPUT_LOG"
    fi
    if [ ! -f "$RESULT_LOG" ]; then
        echo "- 结果日志缺失: $RESULT_LOG"
    fi
fi

exit $BENCHMARK_EXIT_CODE