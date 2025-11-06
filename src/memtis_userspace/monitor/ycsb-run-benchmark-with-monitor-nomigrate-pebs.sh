#!/bin/bash

# 使用方法:
#   1) 仅指定 DRAM 大小（监控间隔默认 1 秒）
#      ./ycsb-run-benchmark-with-monitor-nomigrate.sh 10G
#   2) 同时指定监控间隔(秒) 与 DRAM 大小
#      ./ycsb-run-benchmark-with-monitor-nomigrate.sh 1 10G
# 参数解析规则:
#   - 若第一个参数为纯数字，则视为监控间隔(秒)，第二个参数为 DRAM 大小（可选）
#   - 若仅传入一个非数字参数，则视为 DRAM 大小，监控间隔默认 1 秒
#
# 整合脚本：运行 YCSB 基准测试并监控 NUMA 命中情况
# 用法: ./ycsb-run-benchmark-with-monitor.sh [监控间隔秒数]
# 示例: ./ycsb-run-benchmark-with-monitor.sh 1

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
BENCHMARK_SCRIPT="$SCRIPT_DIR/../hottest_10G_read-nomigrate.sh"
CALC_SCRIPT="$SCRIPT_DIR/../calculate_pgmetrics.sh"
CALC_BW="$SCRIPT_DIR/../calculate_bandwidth.sh"
WORKING_DIR="$SCRIPT_DIR/.."
MODE_DIR="nomigrate"
RESULT_BASE="/home/iscas/hmm/Nomad/src/memtis_userspace/results/ycsb/10G-hottest/read/run-memtis-10g-read/microbench/$MODE_DIR"
if [ -n "$DRAM_SIZE_ARG" ]; then
    RESULT_BASE="$RESULT_BASE/$DRAM_SIZE_ARG"
fi
RESULT_LOG="$RESULT_BASE/zipfan_hottest_10G.read.log"

PEBS_DIR="$(cd "$SCRIPT_DIR/../htmm_pebs_sampler" 2>/dev/null && pwd)"
PEBS_MODULE="htmm_pebs_sampler"
PEBS_KO="$PEBS_DIR/${PEBS_MODULE}.ko"
PEBS_TRACE_ROOT=""
PEBS_MONITOR_STARTED=0
PEBS_VIEW_WRAPPER_PID=""
PEBS_VIEW_PGID=""
PEBS_VIEW_OUTPUT=""
PEBS_SUMMARY_FILE=""
PEBS_SUMMARY_DONE=0
LLC_SAMPLE_PERIOD=${LLC_SAMPLE_PERIOD:-199}
STORE_SAMPLE_PERIOD=${STORE_SAMPLE_PERIOD:-100003}
ENABLE_DRAM=${ENABLE_DRAM:-1}
ENABLE_STORE=${ENABLE_STORE:-1}

detect_trace_root() {
    if [ -d /sys/kernel/tracing ]; then
        echo "/sys/kernel/tracing"
    elif [ -d /sys/kernel/debug/tracing ]; then
        echo "/sys/kernel/debug/tracing"
    else
        if sudo mount -t debugfs none /sys/kernel/debug 2>/dev/null; then
            if [ -d /sys/kernel/debug/tracing ]; then
                echo "/sys/kernel/debug/tracing"
            fi
        fi
    fi
}

start_pebs_monitor() {
    if [ -z "$PEBS_DIR" ] || [ ! -d "$PEBS_DIR" ]; then
        echo "错误: 无法定位 PEBS 目录: $SCRIPT_DIR/../htmm_pebs_sampler" >&2
        return 1
    fi

    if [ ! -f "$PEBS_KO" ]; then
        echo "错误: 找不到 PEBS 模块文件: $PEBS_KO" >&2
        return 1
    fi

    PEBS_TRACE_ROOT=$(detect_trace_root)
    if [ -z "$PEBS_TRACE_ROOT" ]; then
        echo "错误: 找不到 tracing 目录，无法启动 PEBS 监控" >&2
        return 1
    fi

    echo "PEBS: 使用 tracing 路径: $PEBS_TRACE_ROOT"

    echo nop | sudo tee "$PEBS_TRACE_ROOT/current_tracer" >/dev/null
    echo 1 | sudo tee "$PEBS_TRACE_ROOT/tracing_on" >/dev/null
    echo 0 | sudo tee "$PEBS_TRACE_ROOT/trace" >/dev/null

    if lsmod | grep -q "^${PEBS_MODULE} "; then
        echo "PEBS: 检测到模块已加载，正在卸载旧实例..."
        sudo rmmod "$PEBS_MODULE" || true
        sleep 1
    fi

    local sample_pid="${PEBS_SAMPLE_PID:--1}"
    echo "PEBS: 加载模块 (sample_pid=${sample_pid}, llc=${LLC_SAMPLE_PERIOD}, store=${STORE_SAMPLE_PERIOD})..."
    sudo insmod "$PEBS_KO" \
        sample_pid="$sample_pid" \
        llc_sample_period="$LLC_SAMPLE_PERIOD" \
        store_sample_period="$STORE_SAMPLE_PERIOD" \
        enable_dram="$ENABLE_DRAM" \
        enable_store="$ENABLE_STORE"

    PEBS_MONITOR_STARTED=1
}

stop_pebs_monitor() {
    if [ "$PEBS_MONITOR_STARTED" -eq 1 ]; then
        echo "PEBS: 卸载模块..."
        sudo rmmod "$PEBS_MODULE" >/dev/null 2>&1 || true
        PEBS_MONITOR_STARTED=0
    fi

    if [ -n "$PEBS_TRACE_ROOT" ]; then
        echo 0 | sudo tee "$PEBS_TRACE_ROOT/tracing_on" >/dev/null 2>&1 || true
    fi
}

start_pebs_view() {
    if [ -z "$PEBS_TRACE_ROOT" ]; then
        echo "错误: 未检测到 tracing 路径，无法启动实时输出" >&2
        return 1
    fi

    if [ -z "$PEBS_VIEW_OUTPUT" ]; then
        echo "错误: 未指定实时输出文件" >&2
        return 1
    fi

    : > "$PEBS_VIEW_OUTPUT"

    local cmd="cat \"$PEBS_TRACE_ROOT/trace_pipe\" | grep \"htmm_pebs_sampler\" --line-buffered"
    sudo bash -c "$cmd" >> "$PEBS_VIEW_OUTPUT" &
    PEBS_VIEW_WRAPPER_PID=$!
    sleep 0.2
    if ! sudo ps -p "$PEBS_VIEW_WRAPPER_PID" >/dev/null 2>&1; then
        echo "错误: 实时输出进程启动失败 (PID=$PEBS_VIEW_WRAPPER_PID)" >&2
        PEBS_VIEW_WRAPPER_PID=""
        return 1
    fi
    PEBS_VIEW_PGID=$(sudo ps -o pgid= -p "$PEBS_VIEW_WRAPPER_PID" 2>/dev/null | tr -d ' ')
    echo "PEBS: 实时输出已写入 $PEBS_VIEW_OUTPUT (PID=$PEBS_VIEW_WRAPPER_PID)"
}

stop_pebs_view() {
    if [ -n "$PEBS_VIEW_WRAPPER_PID" ]; then
        if sudo ps -p "$PEBS_VIEW_WRAPPER_PID" >/dev/null 2>&1; then
            echo "PEBS: 停止实时输出 (PID=$PEBS_VIEW_WRAPPER_PID)..."
            if [ -n "$PEBS_VIEW_PGID" ]; then
                sudo kill -TERM "-$PEBS_VIEW_PGID" >/dev/null 2>&1 || true
                sleep 0.2
                sudo kill -KILL "-$PEBS_VIEW_PGID" >/dev/null 2>&1 || true
            else
                sudo kill -TERM "$PEBS_VIEW_WRAPPER_PID" >/dev/null 2>&1 || true
            fi
            wait "$PEBS_VIEW_WRAPPER_PID" 2>/dev/null || true
        fi
    fi
    PEBS_VIEW_WRAPPER_PID=""
    PEBS_VIEW_PGID=""
}

capture_pebs_summary() {
    if [ "$PEBS_MONITOR_STARTED" -ne 1 ]; then
        return
    fi

    if [ -z "$PEBS_DIR" ] || [ ! -f "$PEBS_DIR/view_results.sh" ]; then
        return
    fi

    if [ "$PEBS_SUMMARY_DONE" -eq 1 ]; then
        return
    fi

    PEBS_SUMMARY_FILE="${PEBS_SUMMARY_FILE:-$RESULT_BASE/pebs_summary.txt}"
    echo "PEBS: 保存汇总结果到 $PEBS_SUMMARY_FILE"
    sudo "$PEBS_DIR/view_results.sh" all -o "$PEBS_SUMMARY_FILE" >/dev/null 2>&1 || \
        sudo "$PEBS_DIR/view_results.sh" stats -o "$PEBS_SUMMARY_FILE" >/dev/null 2>&1 || true
    if [ -f "$PEBS_SUMMARY_FILE" ]; then
        sudo chown "$(id -u)":"$(id -g)" "$PEBS_SUMMARY_FILE" >/dev/null 2>&1 || true
    fi
    PEBS_SUMMARY_DONE=1
}

finalize() {
    stop_pebs_view
    capture_pebs_summary
    stop_pebs_monitor
}

cleanup_signal() {
    echo ""
    echo "=========================================="
    echo "检测到中断信号，正在退出..."
    echo "=========================================="
    exit 130
}

trap finalize EXIT
trap cleanup_signal INT TERM

# 检查必要文件是否存在
missing_components=()

if [ ! -f "$BENCHMARK_SCRIPT" ]; then
    missing_components+=("基准测试脚本 ($BENCHMARK_SCRIPT)")
fi

if [ -z "$PEBS_DIR" ] || [ ! -d "$PEBS_DIR" ]; then
    missing_components+=("PEBS 目录 ($SCRIPT_DIR/../htmm_pebs_sampler)")
else
    if [ ! -f "$PEBS_KO" ]; then
        missing_components+=("PEBS 模块 ($PEBS_KO)")
    fi
    if [ ! -f "$PEBS_DIR/view_results.sh" ]; then
        missing_components+=("view_results.sh ($PEBS_DIR/view_results.sh)")
    fi
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
if [ -f "$PEBS_DIR/view_results.sh" ]; then
    chmod +x "$PEBS_DIR/view_results.sh"
fi

# 确保结果目录存在
mkdir -p "$RESULT_BASE"
PEBS_VIEW_OUTPUT="$RESULT_BASE/pebs_live_trace.txt"

echo "=========================================="
echo "开始运行 YCSB 基准测试"
echo "=========================================="
echo "基准测试脚本: $BENCHMARK_SCRIPT"
echo "输出路径: $RESULT_BASE"
echo "PEBS 实时输出将写入: $PEBS_VIEW_OUTPUT"
echo "=========================================="

echo "正在启动 YCSB 基准测试..."

# 构建要执行的命令
if [ -n "$DRAM_SIZE_ARG" ]; then
    RUN_CMD="cd $WORKING_DIR && sudo env DRAM_SIZE=$DRAM_SIZE_ARG $BENCHMARK_SCRIPT"
else
    RUN_CMD="cd $WORKING_DIR && sudo env DRAM_SIZE= $BENCHMARK_SCRIPT"
fi

echo "命令: $RUN_CMD"
echo "=========================================="

echo "验证 sudo 权限..."
if ! sudo -v; then
    echo "错误: 无法获取 sudo 权限" >&2
    exit 1
fi

if ! start_pebs_monitor; then
    exit 1
fi

if ! start_pebs_view; then
    stop_pebs_monitor
    exit 1
fi

# 记录开始时间
START_TIME=$(date '+%Y-%m-%d %H:%M:%S')
echo "基准测试开始时间: $START_TIME"

# 直接运行基准测试，命令中已设置 DRAM_SIZE 环境变量（若指定）
eval "$RUN_CMD"
BENCHMARK_EXIT_CODE=$?

# 记录结束时间
END_TIME=$(date '+%Y-%m-%d %H:%M:%S')

stop_pebs_view
capture_pebs_summary
stop_pebs_monitor

echo "基准测试结束时间: $END_TIME"

echo "=========================================="
echo "基准测试完成"
echo "开始时间: $START_TIME"
echo "结束时间: $END_TIME"
echo "退出代码: $BENCHMARK_EXIT_CODE"
if [ $BENCHMARK_EXIT_CODE -eq 0 ]; then
    echo "基准测试执行成功"
    echo "输出目录: $RESULT_BASE"
else
    echo "基准测试执行失败"
fi

echo "=========================================="
if [ -f "$PEBS_VIEW_OUTPUT" ]; then
    echo "PEBS 实时输出文件: $PEBS_VIEW_OUTPUT"
fi
if [ -n "$PEBS_SUMMARY_FILE" ] && [ -f "$PEBS_SUMMARY_FILE" ]; then
    echo "PEBS 汇总输出文件: $PEBS_SUMMARY_FILE"
fi

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
