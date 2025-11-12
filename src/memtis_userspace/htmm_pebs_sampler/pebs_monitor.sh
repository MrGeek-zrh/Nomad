#!/bin/bash
#
# PEBS 采样器通用监控脚本
# 支持监控指定 PID 或启动测试负载
#
# 使用方法:
#   1. 监控现有进程:
#      ./pebs_monitor.sh --pid <PID> [--duration <秒数>]
#      ./pebs_monitor.sh --pid <PID>  # 监控直到进程退出
#
#   2. 启动测试负载并监控:
#      ./pebs_monitor.sh --test [<运行时间>] [<线程数>]
#      ./pebs_monitor.sh --test 10 2
#      ./pebs_monitor.sh --test      # 默认 10 秒，2 线程
#
#   3. 向后兼容（旧脚本用法）:
#      ./pebs_monitor.sh [运行时间] [线程数]  # 等同于 --test
#
# 模块参数（可通过环境变量设置）:
#   LLC_SAMPLE_PERIOD=199
#   STORE_SAMPLE_PERIOD=100003
#   ENABLE_DRAM=1
#   ENABLE_STORE=1

set -e

MODULE="htmm_pebs_sampler"
KO_FILE="${MODULE}.ko"
TEST_BIN="test_load"

# 默认参数
MONITOR_MODE=""  # "pid" 或 "test"
TARGET_PID=""
MONITOR_DURATION=""  # 如果为空，监控直到进程退出
TEST_DURATION=10
TEST_THREADS=2

# 模块参数（可通过环境变量覆盖）
LLC_SAMPLE_PERIOD=${LLC_SAMPLE_PERIOD:-199}
STORE_SAMPLE_PERIOD=${STORE_SAMPLE_PERIOD:-100003}
ENABLE_DRAM=${ENABLE_DRAM:-1}
ENABLE_STORE=${ENABLE_STORE:-1}

# 显示帮助信息
show_help() {
    cat <<EOF
PEBS 采样器监控工具

用法:
  $0 --pid <PID> [--duration <秒数>]
  $0 --test [<运行时间>] [<线程数>]
  $0 [运行时间] [线程数]              # 向后兼容

选项:
  --pid PID           监控指定的进程 PID（监控直到进程退出或达到指定时长）
  --duration SECONDS  监控时长（秒），仅用于 --pid 模式
  --test [SECONDS] [THREADS]  启动测试负载并监控

环境变量（模块参数）:
  LLC_SAMPLE_PERIOD     LLC 采样周期（默认: 199）
  STORE_SAMPLE_PERIOD   存储采样周期（默认: 100003）
  ENABLE_DRAM           启用 DRAM 采样（默认: 1）
  ENABLE_STORE          启用存储采样（默认: 1）

示例:
  # 监控 PID 1234，直到进程退出
  $0 --pid 1234

  # 监控 PID 1234，持续 30 秒
  $0 --pid 1234 --duration 30

  # 启动测试负载，运行 20 秒，4 个线程
  $0 --test 20 4

  # 使用自定义采样周期
  LLC_SAMPLE_PERIOD=100 $0 --pid 1234
EOF
}

# 解析命令行参数
parse_args() {
    # 如果第一个参数是 --help 或 -h，显示帮助
    if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
        show_help
        exit 0
    fi

    # 检查是否使用了显式选项格式
    if [ "$1" = "--pid" ]; then
        if [ -z "$2" ]; then
            echo "错误: --pid 需要指定 PID" >&2
            exit 1
        fi
        MONITOR_MODE="pid"
        TARGET_PID="$2"
        shift 2
        
        # 检查是否有 --duration
        if [ "$1" = "--duration" ]; then
            if [ -z "$2" ]; then
                echo "错误: --duration 需要指定秒数" >&2
                exit 1
            fi
            MONITOR_DURATION="$2"
            shift 2
        fi
    elif [ "$1" = "--test" ]; then
        MONITOR_MODE="test"
        shift
        
        if [ -n "$1" ] && [[ "$1" =~ ^[0-9]+$ ]]; then
            TEST_DURATION="$1"
            shift
        fi
        if [ -n "$1" ] && [[ "$1" =~ ^[0-9]+$ ]]; then
            TEST_THREADS="$1"
            shift
        fi
    elif [ -n "$1" ]; then
        # 向后兼容：如果第一个参数是数字，当作旧脚本的用法
        if [[ "$1" =~ ^[0-9]+$ ]]; then
            MONITOR_MODE="test"
            TEST_DURATION="$1"
            if [ -n "$2" ] && [[ "$2" =~ ^[0-9]+$ ]]; then
                TEST_THREADS="$2"
            fi
        else
            echo "错误: 未知参数: $1" >&2
            echo "使用 --help 查看帮助信息" >&2
            exit 1
        fi
    else
        # 没有参数，显示帮助
        show_help
        exit 1
    fi
}

# 检查进程是否存在
check_pid() {
    local pid=$1
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "错误: 进程 $pid 不存在或无法访问" >&2
        exit 1
    fi
    echo "找到进程 PID $pid: $(ps -p $pid -o comm= 2>/dev/null || echo '未知')"
}

# 等待进程退出或达到指定时长
wait_for_process() {
    local pid=$1
    local duration=$2
    
    if [ -n "$duration" ]; then
        echo "监控进程 $pid，持续 $duration 秒..."
        local elapsed=0
        while [ $elapsed -lt $duration ]; do
            if ! kill -0 "$pid" 2>/dev/null; then
                echo "进程 $pid 已退出（运行了 $elapsed 秒）"
                return 0
            fi
            sleep 1
            elapsed=$((elapsed + 1))
        done
        echo "监控时长已达到 $duration 秒"
    else
        echo "监控进程 $pid，直到进程退出..."
        while kill -0 "$pid" 2>/dev/null; do
            sleep 1
        done
        echo "进程 $pid 已退出"
    fi
}

# 检测 tracing 路径
detect_trace_root() {
    if [ -d /sys/kernel/tracing ]; then
        echo "/sys/kernel/tracing"
    elif [ -d /sys/kernel/debug/tracing ]; then
        echo "/sys/kernel/debug/tracing"
    else
        # 尝试挂载 debugfs
        if sudo mount -t debugfs none /sys/kernel/debug 2>/dev/null; then
            if [ -d /sys/kernel/debug/tracing ]; then
                echo "/sys/kernel/debug/tracing"
            fi
        fi
    fi
}

# 主函数
main() {
    parse_args "$@"
    
    # 检查模块文件
    if [ ! -f "$KO_FILE" ]; then
        echo "错误: 找不到 $KO_FILE，请先编译模块" >&2
        exit 1
    fi
    
    # 检测 tracing 路径
    TRACE_ROOT=$(detect_trace_root)
    if [ -z "$TRACE_ROOT" ]; then
        echo "错误: 找不到 tracing 目录" >&2
        echo "请确保:" >&2
        echo "  1. 内核编译时启用了 CONFIG_FTRACE" >&2
        echo "  2. 或者手动挂载: sudo mount -t debugfs none /sys/kernel/debug" >&2
        exit 1
    fi
    
    echo "使用 tracing 路径: $TRACE_ROOT"
    
    # 启用 ftrace
    echo "启用 ftrace..."
    echo nop | sudo tee "$TRACE_ROOT/current_tracer" > /dev/null
    echo 1 | sudo tee "$TRACE_ROOT/tracing_on" > /dev/null
    
    # 检查是否已加载模块
    if lsmod | grep -q "^${MODULE} "; then
        echo "模块已加载，先卸载..."
        sudo rmmod "$MODULE" || true
        sleep 1
    fi
    
    # 根据模式处理
    TEST_PID=""
    if [ "$MONITOR_MODE" = "pid" ]; then
        # 监控现有 PID 模式
        check_pid "$TARGET_PID"
        echo "加载模块（per-cpu 模式，监控 PID $TARGET_PID）..."
        sudo insmod "$KO_FILE" sample_pid="$TARGET_PID" \
            llc_sample_period="$LLC_SAMPLE_PERIOD" \
            store_sample_period="$STORE_SAMPLE_PERIOD" \
            enable_dram="$ENABLE_DRAM" \
            enable_store="$ENABLE_STORE"
        TEST_PID="$TARGET_PID"
    elif [ "$MONITOR_MODE" = "test" ]; then
        # 启动测试负载模式
        if [ ! -f "$TEST_BIN" ]; then
            echo "编译测试程序..."
            gcc -O2 -o "$TEST_BIN" test_load.c -lpthread || {
                echo "错误: 编译测试程序失败" >&2
                exit 1
            }
        fi
        
        echo "加载模块（per-cpu 模式）..."
        sudo insmod "$KO_FILE" sample_pid=-1 \
            llc_sample_period="$LLC_SAMPLE_PERIOD" \
            store_sample_period="$STORE_SAMPLE_PERIOD" \
            enable_dram="$ENABLE_DRAM" \
            enable_store="$ENABLE_STORE"
        
        echo ""
        echo "启动测试负载..."
        taskset -c 1-4 numactl --interleave=0,1 ./"$TEST_BIN" "$TEST_DURATION" "$TEST_THREADS" &
        TEST_PID=$!
        echo "测试负载 PID: $TEST_PID"
        echo "运行 $TEST_DURATION 秒..."
    fi
    
    echo ""
    echo "模块参数:"
    cat /sys/module/${MODULE}/parameters/* 2>/dev/null | grep -v "^$" || true
    
    echo ""
    echo "提示: 在另一个终端可以实时查看采样结果："
    echo "  ./view_results.sh live    # 实时监控"
    echo "  ./view_results.sh trace    # 查看 trace 文件"
    echo "  ./view_results.sh dmesg    # 查看 dmesg"
    echo "  ./view_results.sh stats    # 查看统计信息"
    
    # 清空 trace buffer
    echo 0 | sudo tee "$TRACE_ROOT/trace" > /dev/null
    
    # 等待测试完成或进程退出
    if [ "$MONITOR_MODE" = "test" ]; then
        wait $TEST_PID
    else
        wait_for_process "$TEST_PID" "$MONITOR_DURATION"
    fi
    
    # 等待一小段时间让模块处理完样本
    sleep 2
    
    # 卸载模块
    echo ""
    echo "卸载模块..."
    sudo rmmod "$MODULE" || true
    
    # 显示结果
    echo ""
    echo "=== 采样结果（来自 trace_printk）==="
    echo ""
    
    # 方法1: 从 trace 文件查看
    echo "--- 方法1: $TRACE_ROOT/trace ---"
    TRACE_COUNT=$(sudo cat "$TRACE_ROOT/trace" 2>/dev/null | grep -c "htmm_pebs_sampler" || echo "0")
    if [ "$TRACE_COUNT" -gt 0 ]; then
        echo "找到 $TRACE_COUNT 条记录："
        sudo cat "$TRACE_ROOT/trace" 2>/dev/null | grep "htmm_pebs_sampler" | head -20
    else
        echo "trace 文件中未找到记录"
    fi
    
    # 方法2: 从 dmesg 查看
    echo ""
    echo "--- 方法2: dmesg (内核日志) ---"
    DMESG_COUNT=$(sudo dmesg | grep -c "htmm_pebs_sampler" || echo "0")
    if [ "$DMESG_COUNT" -gt 0 ]; then
        echo "找到 $DMESG_COUNT 条记录："
        sudo dmesg | grep "htmm_pebs_sampler" | tail -20
    else
        echo "dmesg 中未找到记录"
    fi
    
    # 方法3: 实时查看提示
    echo ""
    echo "--- 方法3: 实时查看 (trace_pipe) ---"
    echo "提示: 如果模块仍在运行，可以使用以下命令实时查看："
    echo "  sudo cat $TRACE_ROOT/trace_pipe | grep htmm_pebs_sampler"
    
    # 方法4: 检查模块统计信息
    echo ""
    echo "=== 模块统计信息 ==="
    if [ -d /sys/module/${MODULE} ]; then
        echo "模块参数:"
        cat /sys/module/${MODULE}/parameters/* 2>/dev/null | while read line; do
            [ -n "$line" ] && echo "  $line"
        done
    fi
    
    # 显示总统计
    echo ""
    echo "=== 总统计 ==="
    echo "trace 文件记录数: $TRACE_COUNT"
    echo "dmesg 记录数: $DMESG_COUNT"
    
    if [ "$TRACE_COUNT" -eq 0 ] && [ "$DMESG_COUNT" -eq 0 ]; then
        echo ""
        echo "⚠️  警告: 未找到任何采样记录"
        echo ""
        echo "可能的原因："
        echo "  1. 事件配置不正确，未触发 PEBS 溢出"
        echo "  2. 地址无效（data->addr == 0）被过滤"
        echo "  3. get_nid_for_address 失败（nid < 0）"
        echo "  4. 采样周期太大，短时间内没有样本"
        echo ""
        echo "调试建议："
        echo "  1. 检查 dmesg: sudo dmesg | tail -50"
        echo "  2. 检查模块加载日志: sudo dmesg | grep htmm_pebs_sampler"
        echo "  3. 查看 trace 原始内容: sudo cat $TRACE_ROOT/trace | tail -100"
        echo "  4. 尝试实时监控: sudo cat $TRACE_ROOT/trace_pipe"
        echo "  5. 检查模块参数: cat /sys/module/${MODULE}/parameters/*"
    fi
    
    echo ""
    echo "监控完成！"
}

# 运行主函数
main "$@"

