#!/bin/bash
#
# 清理 htmm_pebs_sampler 模块的脚本
# 用于处理模块使用计数异常或无法正常卸载的情况

MODULE="htmm_pebs_sampler"

echo "=== 清理 $MODULE 模块 ==="
echo ""

# 检查模块是否加载
if ! lsmod | grep -q "^${MODULE} "; then
    echo "模块未加载，无需清理"
    exit 0
fi

# 显示当前状态
echo "当前模块状态:"
lsmod | grep "^${MODULE} " || true
echo ""

# 尝试正常卸载
echo "尝试正常卸载..."
if sudo rmmod "$MODULE" 2>&1; then
    echo "✓ 模块已成功卸载"
    exit 0
fi

# 如果正常卸载失败，检查是否有进程在使用
echo ""
echo "正常卸载失败，检查可能的原因..."
echo ""

# 检查是否有工作队列任务
echo "检查工作队列:"
if [ -d /sys/kernel/debug/tracing ]; then
    sudo cat /sys/kernel/debug/tracing/trace | grep -i "htmm_pebs\|pebs_sampler" | tail -5 || true
elif [ -d /sys/kernel/tracing ]; then
    sudo cat /sys/kernel/tracing/trace | grep -i "htmm_pebs\|pebs_sampler" | tail -5 || true
fi

echo ""
echo "如果模块使用计数为 -1，可能需要："
echo "  1. 等待一段时间让工作队列完成"
echo "  2. 或者重启系统"
echo ""
echo "再次尝试卸载..."
sleep 2

if sudo rmmod "$MODULE" 2>&1; then
    echo "✓ 模块已成功卸载"
    exit 0
else
    echo "✗ 仍然无法卸载"
    echo ""
    echo "建议："
    echo "  1. 检查 dmesg: sudo dmesg | tail -50"
    echo "  2. 等待更长时间后重试"
    echo "  3. 如果问题持续，考虑重启系统"
    exit 1
fi

