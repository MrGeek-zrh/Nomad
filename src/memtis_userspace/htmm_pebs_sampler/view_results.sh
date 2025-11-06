#!/bin/bash
#
# 查看 PEBS 采样器结果的脚本
# 支持多种查看方式
#
# 使用方法:
#   ./view_results.sh [mode] [-o|--output output_file]
#   模式: trace(t), dmesg(d), live(l), stats(s), all(a)
#   默认: trace
#   -o, --output: 指定输出文件路径（静默模式，不输出提示信息）

TRACE_ROOT=""
if [ -d /sys/kernel/tracing ]; then
    TRACE_ROOT="/sys/kernel/tracing"
elif [ -d /sys/kernel/debug/tracing ]; then
    TRACE_ROOT="/sys/kernel/debug/tracing"
else
    echo "错误: 找不到 tracing 目录" >&2
    exit 1
fi

# 解析参数
OUTPUT_FILE=""
MODE=""
QUIET=0

while [[ $# -gt 0 ]]; do
    case $1 in
        -o|--output)
            if [ -z "$2" ] || [[ "$2" =~ ^- ]]; then
                echo "错误: -o/--output 选项需要指定输出文件路径" >&2
                exit 1
            fi
            OUTPUT_FILE="$2"
            QUIET=1
            shift 2
            ;;
        -q|--quiet)
            QUIET=1
            shift
            ;;
        trace|t|dmesg|d|live|l|pipe|p|stats|s|all|a)
            MODE="$1"
            shift
            ;;
        *)
            echo "用法: $0 [mode] [-o|--output output_file] [-q|--quiet]" >&2
            echo "" >&2
            echo "模式:" >&2
            echo "  trace, t    - 从 trace 文件查看（默认）" >&2
            echo "  dmesg, d    - 从 dmesg 查看" >&2
            echo "  live, l     - 实时监控（trace_pipe）" >&2
            echo "  stats, s    - 显示统计信息" >&2
            echo "  all, a      - 显示所有来源" >&2
            echo "" >&2
            echo "选项:" >&2
            echo "  -o, --output file  - 指定输出文件路径（静默模式）" >&2
            echo "  -q, --quiet        - 静默模式（不显示提示信息）" >&2
            exit 1
            ;;
    esac
done

# 如果没有指定模式，使用默认值
MODE=${MODE:-trace}

# 如果指定了输出文件，创建输出重定向
if [ -n "$OUTPUT_FILE" ]; then
    # 确保输出目录存在
    OUTPUT_DIR=$(dirname "$OUTPUT_FILE")
    if [ -n "$OUTPUT_DIR" ] && [ "$OUTPUT_DIR" != "." ]; then
        mkdir -p "$OUTPUT_DIR" 2>/dev/null || {
            echo "错误: 无法创建输出目录: $OUTPUT_DIR" >&2
            exit 1
        }
    fi
    exec > "$OUTPUT_FILE"
    QUIET=1
fi

# 输出函数：根据 QUIET 模式决定是否输出
output() {
    if [ $QUIET -eq 0 ]; then
        echo "$@"
    fi
}

case "$MODE" in
    trace|t)
        output "=== 从 trace 文件查看 ==="
        sudo cat "$TRACE_ROOT/trace" | grep "htmm_pebs_sampler" | tail -50
        ;;
    dmesg|d)
        output "=== 从 dmesg 查看 ==="
        sudo dmesg | grep "htmm_pebs_sampler" | tail -50
        ;;
    live|l|pipe|p)
        output "=== 实时监控 (按 Ctrl+C 退出) ==="
        if [ -n "$OUTPUT_FILE" ]; then
            # 如果指定了输出文件，实时监控模式不适合，给出提示
            echo "错误: 实时监控模式不支持输出到文件" >&2
            exit 1
        fi
        sudo cat "$TRACE_ROOT/trace_pipe" | grep "htmm_pebs_sampler" --line-buffered
        ;;
    stats|s)
        output "=== 统计信息 ==="
        TRACE_COUNT=$(sudo cat "$TRACE_ROOT/trace" 2>/dev/null | grep -c "htmm_pebs_sampler" || echo "0")
        DMESG_COUNT=$(sudo dmesg | grep -c "htmm_pebs_sampler" || echo "0")
        echo "trace 文件记录数: $TRACE_COUNT"
        echo "dmesg 记录数: $DMESG_COUNT"
        echo ""
        if [ -d /sys/module/htmm_pebs_sampler ]; then
            output "模块参数:"
            cat /sys/module/htmm_pebs_sampler/parameters/* 2>/dev/null | while read line; do
                [ -n "$line" ] && echo "  $line"
            done
        fi
        ;;
    all|a)
        output "=== 所有来源 ==="
        output ""
        output "--- trace 文件 ---"
        sudo cat "$TRACE_ROOT/trace" 2>/dev/null | grep "htmm_pebs_sampler" | tail -20
        output ""
        output "--- dmesg ---"
        sudo dmesg | grep "htmm_pebs_sampler" | tail -20
        ;;
    *)
        echo "用法: $0 [mode] [-o|--output output_file] [-q|--quiet]" >&2
        echo "" >&2
        echo "模式:" >&2
        echo "  trace, t    - 从 trace 文件查看（默认）" >&2
        echo "  dmesg, d    - 从 dmesg 查看" >&2
        echo "  live, l     - 实时监控（trace_pipe）" >&2
        echo "  stats, s    - 显示统计信息" >&2
        echo "  all, a      - 显示所有来源" >&2
        echo "" >&2
        echo "选项:" >&2
        echo "  -o, --output file  - 指定输出文件路径（静默模式）" >&2
        echo "  -q, --quiet        - 静默模式（不显示提示信息）" >&2
        exit 1
        ;;
esac

