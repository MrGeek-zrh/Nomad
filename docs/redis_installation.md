# Redis 编译安装指南

本文档说明如何在 NOMAD 项目中编译安装 Redis 6.2.13，以满足 `start_redis_server.sh` 脚本的要求。

## 目录

- [概述](#概述)
- [系统要求](#系统要求)
- [安装步骤](#安装步骤)
- [验证安装](#验证安装)
- [脚本要求](#脚本要求)
- [故障排除](#故障排除)

## 概述

NOMAD 项目使用 Redis 6.2.13 作为 benchmark 测试工具。Redis 需要编译安装到特定目录 `third_party/tmp/redis-6.2.13/`，以便 `start_redis_server.sh` 脚本能够正确找到并启动 Redis 服务器。

## 系统要求

### 必需工具

- `wget` 或 `curl` - 用于下载源码
- `tar` - 用于解压源码包
- `make` - 用于编译
- `gcc` - C 编译器（通常已包含在 build-essential 中）

### 检查工具

在开始之前，检查必需工具是否已安装：

```bash
which wget make gcc
```

如果缺少任何工具，在 Ubuntu/Debian 系统上安装：

```bash
sudo apt update
sudo apt install -y wget build-essential
```

## 安装步骤

### 方法一：使用自动化脚本（推荐）

项目提供了 `third_party/prepare.sh` 脚本，可以自动安装所有第三方依赖，包括 Redis：

```bash
cd /home/mrgeek/hmm/Nomad
bash third_party/prepare.sh
```

### 方法二：手动安装 Redis

如果需要单独安装 Redis，按照以下步骤操作：

#### 1. 创建目录

```bash
cd /home/mrgeek/hmm/Nomad
mkdir -p third_party/tmp
```

#### 2. 下载 Redis 源码

```bash
wget https://github.com/redis/redis/archive/refs/tags/6.2.13.tar.gz -P third_party/tmp/
```

或者使用 curl：

```bash
curl -L https://github.com/redis/redis/archive/refs/tags/6.2.13.tar.gz -o third_party/tmp/6.2.13.tar.gz
```

#### 3. 解压源码

```bash
tar -xvf third_party/tmp/6.2.13.tar.gz -C third_party/tmp
```

解压后，源码将位于 `third_party/tmp/redis-6.2.13/` 目录。

#### 4. 编译 Redis

```bash
cd third_party/tmp/redis-6.2.13
make -j$(nproc)
```

`-j$(nproc)` 选项使用所有可用的 CPU 核心并行编译，加快编译速度。

#### 5. 返回项目根目录

```bash
cd /home/mrgeek/hmm/Nomad
```

## 验证安装

### 检查文件位置

编译完成后，`redis-server` 应该位于：

```
third_party/tmp/redis-6.2.13/src/redis-server
```

验证文件是否存在：

```bash
cd /home/mrgeek/hmm/Nomad
test -f third_party/tmp/redis-6.2.13/src/redis-server && echo "✓ Redis 已成功安装" || echo "✗ Redis 未找到"
```

### 检查版本

```bash
third_party/tmp/redis-6.2.13/src/redis-server --version
```

预期输出：

```
Redis server v=6.2.13 sha=45e6741e:1 malloc=jemalloc-5.1.0 bits=64 build=...
```

### 检查其他工具

Redis 编译还会生成以下工具：

- `redis-cli` - Redis 命令行客户端
- `redis-benchmark` - Redis 性能测试工具
- `redis-check-rdb` - RDB 文件检查工具
- `redis-check-aof` - AOF 文件检查工具
- `redis-sentinel` - Redis 哨兵

验证所有工具：

```bash
ls -lh third_party/tmp/redis-6.2.13/src/redis-*
```

## 脚本要求

`start_redis_server.sh` 脚本要求 Redis 位于以下路径：

```bash
redis_dir=third_party/tmp/redis-6.2.13
```

脚本会使用以下命令启动 Redis：

```bash
${redis_dir}/src/redis-server src/testing_scripts/redis/redis.conf
```

确保：
1. Redis 源码位于 `third_party/tmp/redis-6.2.13/`
2. 编译后的 `redis-server` 位于 `third_party/tmp/redis-6.2.13/src/redis-server`
3. Redis 配置文件位于 `src/testing_scripts/redis/redis.conf`

## 故障排除

### 编译错误

#### 缺少编译工具

**错误信息：**
```
make: command not found
```

**解决方法：**
```bash
sudo apt install -y build-essential
```

#### 内存不足

**错误信息：**
```
virtual memory exhausted: Cannot allocate memory
```

**解决方法：**
- 减少并行编译任务数：`make -j2` 或 `make -j1`
- 增加系统交换空间

#### jemalloc 编译错误

如果遇到 jemalloc 相关的编译错误，可以尝试：

```bash
cd third_party/tmp/redis-6.2.13
make distclean
make -j$(nproc) MALLOC=libc
```

注意：使用系统 malloc 而不是 jemalloc 可能会影响性能。

### 路径问题

#### 脚本找不到 Redis

**错误信息：**
```
redis-server: command not found
```

**检查：**
1. 确认 Redis 安装在正确位置：
   ```bash
   ls -la third_party/tmp/redis-6.2.13/src/redis-server
   ```

2. 确认脚本中的路径变量：
   ```bash
   grep redis_dir src/testing_scripts/redis/start_redis_server.sh
   ```

3. 确认在项目根目录运行脚本：
   ```bash
   pwd  # 应该显示 /home/mrgeek/hmm/Nomad
   ```

### 权限问题

如果遇到权限问题，确保：

1. 编译后的文件有执行权限：
   ```bash
   chmod +x third_party/tmp/redis-6.2.13/src/redis-server
   ```

2. 当前用户有读取配置文件的权限：
   ```bash
   ls -l src/testing_scripts/redis/redis.conf
   ```

## 完整安装脚本

以下是一个完整的独立安装脚本：

```bash
#!/bin/bash
# install_redis.sh

set -e

echo "=== 开始安装 Redis 6.2.13 ==="

# 进入项目根目录
cd "$(dirname "$0")/.."
PROJECT_ROOT=$(pwd)

# 创建目录
echo "创建目录..."
mkdir -p third_party/tmp

# 下载源码
echo "下载 Redis 源码..."
if [ ! -f "third_party/tmp/6.2.13.tar.gz" ]; then
    wget https://github.com/redis/redis/archive/refs/tags/6.2.13.tar.gz -P third_party/tmp/
else
    echo "源码包已存在，跳过下载"
fi

# 解压
echo "解压源码..."
if [ ! -d "third_party/tmp/redis-6.2.13" ]; then
    tar -xvf third_party/tmp/6.2.13.tar.gz -C third_party/tmp
else
    echo "源码已解压，跳过"
fi

# 编译
echo "编译 Redis..."
cd third_party/tmp/redis-6.2.13
if [ ! -f "src/redis-server" ]; then
    make -j$(nproc)
else
    echo "Redis 已编译，跳过"
fi

# 验证
echo "验证安装..."
cd "$PROJECT_ROOT"
if [ -f "third_party/tmp/redis-6.2.13/src/redis-server" ]; then
    echo "✓ Redis 安装成功"
    third_party/tmp/redis-6.2.13/src/redis-server --version
else
    echo "✗ Redis 安装失败"
    exit 1
fi

echo "=== 安装完成 ==="
```

保存为 `install_redis.sh` 并运行：

```bash
chmod +x install_redis.sh
./install_redis.sh
```

## 相关文件

- `src/testing_scripts/redis/start_redis_server.sh` - Redis 服务器启动脚本
- `src/testing_scripts/redis/redis.conf` - Redis 配置文件
- `src/testing_scripts/redis/run_redis.sh` - Redis benchmark 运行脚本
- `third_party/prepare.sh` - 第三方依赖安装脚本

## 参考

- [Redis 官方文档](https://redis.io/documentation)
- [Redis GitHub 仓库](https://github.com/redis/redis)
- [Redis 6.2.13 发布说明](https://github.com/redis/redis/releases/tag/6.2.13)

