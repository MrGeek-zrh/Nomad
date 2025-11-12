# htmm_pebs_sampler (out-of-tree kernel module)

PEBS 驱动的内核采样模块：周期性采样用户态内存访问地址，并在内核侧解析其所在 NUMA 节点。

## 功能概述

本模块通过 Intel PEBS (Precise Event Based Sampling) 技术，在内核中周期性采样应用程序的内存访问地址，并自动解析每个地址所在的 NUMA 节点。所有结果通过 ftrace 输出，便于分析和调试。

## 前提条件

1. **内核版本**: Linux 5.x 或更高版本（需支持 `perf_event_create_kernel_counter`）
2. **硬件平台**: x86/x86_64，且 CPU 支持 PEBS
3. **编译工具**: 内核头文件（通常在 `/usr/src/linux-headers-$(uname -r)` 或 `/lib/modules/$(uname -r)/build`）
4. **权限**: 需要 root 权限加载内核模块

## 构建步骤

```bash
cd htmm_pebs_sampler
make
```

构建成功后会在当前目录生成 `htmm_pebs_sampler.ko` 文件。

## 使用方法

### 1. 加载模块

基本用法（按 CPU 采样，启用 DRAM 和 Store 事件）：

```bash
sudo insmod htmm_pebs_sampler.ko \
    sample_pid=-1 \
    enable_dram=1 \
    enable_store=1 \
    llc_sample_period=199 \
    store_sample_period=100003
```

### 2. 模块参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `sample_pid` | int | -1 | 目标进程 PID。`-1` 表示按 CPU 采样（采样当前在该 CPU 上运行的任务）；`>=0` 表示只采样指定进程（会在所有 CPU 上创建事件，采样该进程在所有 CPU 上的内存访问） |
| `enable_dram` | bool | true | 启用 DRAM LLC load miss 事件采样 |
| `enable_nvm` | bool | false | 启用 NVM LLC load miss 事件采样 |
| `enable_cxl` | bool | false | 启用 CXL (remote DRAM) LLC load miss 事件采样 |
| `enable_store` | bool | true | 启用 ALL_STORES 指令采样 |
| `llc_sample_period` | uint | 199 | LLC miss 事件的采样周期（越小采样越频繁） |
| `store_sample_period` | uint | 100003 | Store 指令的采样周期 |
| `precise_ip` | int | 1 | PEBS 精度级别（1 或 2，2 更精确但性能开销更大） |
| `fifo_order` | uint | 12 | 每 CPU 缓冲区大小（2^order 条记录，默认 4096） |

### 3. 查看输出

使用 ftrace 查看采样结果：

```bash
# 挂载 tracefs（如果未挂载）
sudo mount -t tracefs tracefs /sys/kernel/tracing || true

# 实时查看采样输出
sudo cat /sys/kernel/tracing/trace_pipe | grep htmm_pebs_sampler
```

输出格式示例：
```
htmm_pebs_sampler: pid=12345 addr=0x7fff8a1b2000 ev=0 nid=0
```

其中：
- `pid`: 进程 ID
- `addr`: 被访问的虚拟地址
- `ev`: 事件类型（0=DRAMREAD, 1=NVMREAD, 2=MEMWRITE, 3=CXLREAD）
- `nid`: NUMA 节点 ID

### 4. 使用示例

#### 示例 1: 采样特定进程的内存访问

```bash
# 假设目标进程 PID 为 12345
sudo insmod htmm_pebs_sampler.ko sample_pid=12345 enable_dram=1 enable_store=1
```

#### 示例 2: 采样所有 CPU 上的 LLC miss 事件

```bash
sudo insmod htmm_pebs_sampler.ko \
    sample_pid=-1 \
    enable_dram=1 \
    enable_nvm=1 \
    enable_cxl=1 \
    enable_store=0 \
    llc_sample_period=100
```

#### 示例 3: 高精度采样（更多样本）

```bash
sudo insmod htmm_pebs_sampler.ko \
    sample_pid=-1 \
    enable_dram=1 \
    enable_store=1 \
    llc_sample_period=50 \
    store_sample_period=50000 \
    precise_ip=2 \
    fifo_order=14
```

### 5. 卸载模块

```bash
sudo rmmod htmm_pebs_sampler
```

卸载前会自动停止所有采样事件并清理资源。

## 查看模块信息

```bash
# 查看已加载的模块参数
cat /sys/module/htmm_pebs_sampler/parameters/*

# 查看模块信息
modinfo htmm_pebs_sampler.ko
```

## 常见问题

### Q: 加载模块失败，提示 "perf_event_create_kernel_counter" 未找到？

A: 检查内核是否编译了 `CONFIG_PERF_EVENTS`，且内核版本 >= 5.x。

### Q: 没有看到采样输出？

A: 
1. 确认有用户态程序在运行并产生内存访问
2. 检查 ftrace 是否正常：`cat /sys/kernel/tracing/trace_pipe | head`
3. 尝试降低采样周期（如 `llc_sample_period=50`）增加采样频率
4. 检查 dmesg 是否有错误信息：`dmesg | tail`

### Q: 采样开销太大，影响系统性能？

A: 
1. 增加采样周期（如 `llc_sample_period=500`）
2. 只启用需要的事件类型
3. 使用 `precise_ip=1` 而不是 2
4. 减小 `fifo_order` 降低缓冲区大小

### Q: 如何知道采样了多少样本？

A: 可以通过统计 trace 输出行数：
```bash
sudo cat /sys/kernel/tracing/trace_pipe | grep htmm_pebs_sampler | wc -l
```

### Q: 当指定 `sample_pid` 时，采样如何工作？

A: 当指定了 `sample_pid` (例如 `sample_pid=12345`) 时：
- 模块会在**所有 CPU** 上都创建 perf_event 事件
- 所有事件都绑定到同一个目标进程（task）
- 这样可以利用每个 CPU 上的硬件 PMU 计数器来采样该进程在所有 CPU 上的内存访问
- 如果进程是多线程的或在多个 CPU 上运行，可以在所有 CPU 上同时采样
- 注意：这可能会占用较多的硬件计数器资源（每个 CPU 每个事件类型一个计数器）

## 技术说明

### 工作原理
- 使用 `perf_event_create_kernel_counter` 创建 PEBS 事件，在 NMI 上下文的溢出回调中快速入队样本，在工作队列中异步解析 NUMA 节点
- 当 `sample_pid=-1` 时：在每个 CPU 上创建 per-CPU 事件，采样当前在该 CPU 上运行的任务
- 当 `sample_pid>=0` 时：在每个 CPU 上都创建事件，但都绑定到指定的 task。这样可以利用每个 CPU 上的硬件计数器来采样该进程在所有 CPU 上的内存访问

### 性能考虑
- 溢出回调在硬中断上下文，仅做非阻塞的入队操作；地址解析在工作队列中执行，可以睡眠
- 当指定 `sample_pid` 时，会在所有 CPU 上创建事件，可能占用更多硬件计数器资源

### 独立性
- 本模块完全独立于 memtis/HTMM 内核代码，不修改内核源码，仅复用了 RAW 事件编码定义

## 许可证

GPL v2


