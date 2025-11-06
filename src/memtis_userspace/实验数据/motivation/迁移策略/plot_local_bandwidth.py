#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
绘制迁移策略本地带宽折线图
"""

import matplotlib.pyplot as plt
import matplotlib
import numpy as np

# 设置字体
matplotlib.rcParams['axes.unicode_minus'] = False

# 文件路径
data_dir = "/home/iscas/hmm/Nomad/src/memtis_userspace/实验数据/motivation/迁移策略/data"
files = [
    ("AutoNUMA", f"{data_dir}/ycsb-zipfian-10G-AutoNUMA-bandwidth_data.txt"),
    ("TPP", f"{data_dir}/ycsb-zipfian-10G-tpp-bandwidth_data.txt"),
    ("nomigrate", f"{data_dir}/ycsb-zipfian-nomigrate-10G-bandwidth_data.txt")
]

# 读取数据
data = {}
for name, filepath in files:
    times = []
    local_bandwidths = []
    remote_bandwidths = []
    
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        # 跳过表头
        for line in lines[1:]:
            line = line.strip()
            if not line:
                continue
            parts = line.split('\t')
            if len(parts) >= 3:
                try:
                    time = float(parts[0])
                    local_bw = float(parts[1])
                    remote_bw = float(parts[2])
                    times.append(time)
                    local_bandwidths.append(local_bw)
                    remote_bandwidths.append(remote_bw)
                except ValueError:
                    continue
    
    data[name] = {
        'times': np.array(times),
        'local_bandwidths': np.array(local_bandwidths),
        'remote_bandwidths': np.array(remote_bandwidths)
    }

# 创建图形，2行1列
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
plt.rcParams['font.size'] = 12

# 定义颜色和线型
colors = ['#1f77b4', '#ff7f0e', '#2ca02c']  # 蓝色、橙色、绿色
markers = ['o', 's', '^']
marker_sizes = [3, 3, 3]

# 绘制顺序：AutoNUMA、TPP、nomigrate
order = ['AutoNUMA', 'TPP', 'nomigrate']

# 第一个子图：本地带宽
for idx, name in enumerate(order):
    if name in data:
        ax1.plot(data[name]['times'], data[name]['local_bandwidths'], 
                label=name, color=colors[idx], linewidth=1.5, 
                marker=markers[idx], markersize=marker_sizes[idx], 
                markevery=max(1, len(data[name]['times'])//50))

ax1.set_xlabel('Time (sec)', fontsize=14)
ax1.set_ylabel('Local Bandwidth (Mbps)', fontsize=14)
ax1.set_title('Migration Strategy - Local Memory Bandwidth', fontsize=16, fontweight='bold')
ax1.legend(loc='best', fontsize=12)
ax1.grid(True, alpha=0.3, linestyle='--')

# 第二个子图：远端带宽
for idx, name in enumerate(order):
    if name in data:
        ax2.plot(data[name]['times'], data[name]['remote_bandwidths'], 
                label=name, color=colors[idx], linewidth=1.5, 
                marker=markers[idx], markersize=marker_sizes[idx], 
                markevery=max(1, len(data[name]['times'])//50))

ax2.set_xlabel('Time (sec)', fontsize=14)
ax2.set_ylabel('Remote Bandwidth (Mbps)', fontsize=14)
ax2.set_title('Migration Strategy - Remote Memory Bandwidth', fontsize=16, fontweight='bold')
ax2.legend(loc='best', fontsize=12)
ax2.grid(True, alpha=0.3, linestyle='--')

plt.tight_layout()

# 保存图片
output_dir = "/home/iscas/hmm/Nomad/src/memtis_userspace/实验数据/motivation/迁移策略"
output_path = f"{output_dir}/迁移策略-内存节点带宽折线图.png"
plt.savefig(output_path, dpi=300, bbox_inches='tight')
print(f"图片已保存至: {output_path}")

plt.close()

