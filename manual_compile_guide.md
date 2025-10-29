# 手动编译 tpp_mem_access 指南

## 1. 系统依赖安装

### 基础编译工具
```bash
sudo apt update
sudo apt install -y gcc g++ make cmake pkg-config git
```

### 必需的依赖库
```bash
sudo apt install -y libgflags-dev libgoogle-glog-dev
```

### 可选：从源码编译依赖（推荐，确保版本兼容）
如果系统包版本不兼容，可以从源码编译：

#### 编译 gflags
```bash
git clone https://github.com/gflags/gflags.git
cd gflags
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install
cd ../..
```

#### 编译 glog
```bash
git clone https://github.com/google/glog.git
cd glog
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install
cd ../..
```

## 2. 编译所需文件

### 核心源文件
- `src/userspace_programs/tpp/memory_access.cc` - 主程序文件
- `src/userspace_programs/tpp/parse_async_prom_module.cc` - 内存统计模块
- `src/userspace_programs/tpp/parse_async_prom.h` - 头文件

### 第三方库源码（已包含在项目中）
- `src/userspace_programs/third_party/abseil-cpp/` - Abseil C++ 库
- `src/userspace_programs/third_party/glog/` - Google Log 库

### 构建脚本
- `src/userspace_programs/CMakeLists.txt` - 主 CMake 配置
- `src/userspace_programs/tpp/CMakeLists.txt` - tpp 模块配置
- `src/userspace_programs/third_party/CMakeLists.txt` - 第三方库配置

## 3. 手动编译步骤

### 方法一：使用 CMake（推荐）

```bash
# 1. 进入用户空间程序目录
cd src/userspace_programs

# 2. 创建构建目录
mkdir -p build
cd build

# 3. 配置 CMake（重要：使用 -DENABLE_SANI=ON 启用动态链接）
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DENABLE_SANI=ON

# 4. 编译 tpp_mem_access
make tpp_mem_access -j$(nproc)

# 5. 编译其他相关程序（可选）
make parse_async_prom trigger_llc_miss_access -j$(nproc)

# 6. 可执行文件位置
# build/tpp/tpp_mem_access
# build/tpp/parse_async_prom
# build/tpp/trigger_llc_miss_access
```

### 方法二：直接使用 g++（不推荐，复杂）

如果要直接使用 g++ 编译，需要手动处理所有依赖：

```bash
# 1. 首先编译 abseil-cpp 和 glog（按照上面的步骤）

# 2. 编译命令（非常复杂，不推荐）
g++ -std=c++17 \
    -I src/userspace_programs \
    -I src/userspace_programs/third_party/abseil-cpp \
    -I src/userspace_programs/third_party/glog/src \
    src/userspace_programs/tpp/memory_access.cc \
    src/userspace_programs/tpp/parse_async_prom_module.cc \
    -lgflags -lglog -labsl_strings -labsl_flags_parse -labsl_log \
    -labsl_log_initialize -labsl_log_flags -labsl_status -labsl_statusor \
    -o tpp_mem_access
```

## 4. 编译验证

编译成功后，验证程序：

```bash
# 检查可执行文件
ls -la build/tpp/tpp_mem_access

# 查看帮助信息
./build/tpp/tpp_mem_access --help

# 查看链接的库
ldd build/tpp/tpp_mem_access
```

## 5. 运行时依赖

### 测试数据文件
运行 tpp_mem_access 需要以下数据文件（通过 generate_ycsb.sh 生成）：
- `warmup_zipfan_hottest_10G.bin`
- `run_zipfan_hottest_10G.bin`
- 其他规模的测试数据文件

### 内核模块
程序运行时需要内核模块支持：
- `/dev/async_prom` 设备文件
- 相应的内核驱动加载

## 6. 完整的独立编译脚本

```bash
#!/bin/bash
# standalone_compile.sh

set -e

echo "=== 开始编译 tpp_mem_access ==="

# 安装系统依赖
echo "安装系统依赖..."
sudo apt update
sudo apt install -y gcc g++ make cmake pkg-config git libgflags-dev libgoogle-glog-dev

# 进入源码目录
cd src/userspace_programs

# 创建构建目录
echo "创建构建目录..."
mkdir -p build
cd build

# 配置 CMake
echo "配置 CMake..."
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 编译
echo "开始编译..."
make tpp_mem_access -j$(nproc)

# 验证
echo "编译完成，验证..."
if [ -f "tpp/tpp_mem_access" ]; then
    echo "✓ 编译成功: $(pwd)/tpp/tpp_mem_access"
    ./tpp/tpp_mem_access --help | head -5
else
    echo "✗ 编译失败"
    exit 1
fi

echo "=== 编译完成 ==="
```

## 7. 可能遇到的问题

### 依赖版本问题
- 确保 CMake 版本 >= 3.10
- 确保 GCC 支持 C++17
- 如果系统库版本过旧，从源码编译

### gflags 依赖问题
如果遇到 `find_package(gflags)` 错误，请安装：
```bash
sudo apt install libgflags-dev pkg-config
```

### 静态链接问题
如果遇到静态链接错误，使用动态链接：
```bash
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DENABLE_SANI=ON
```

### 链接错误
- 检查 gflags 和 glog 是否正确安装
- 确保 Abseil 库正确编译

### 权限问题
- 运行时需要访问 `/dev/async_prom`
- 可能需要 sudo 权限

## 8. 与 Docker 编译的差异

手动编译与 Docker 编译的主要差异：
- Docker 使用预配置环境，依赖版本固定
- 手动编译需要自己处理依赖兼容性
- Docker 编译结果是静态链接，手动编译通常是动态链接
- Docker 环境包含更多优化选项和特定的编译标志