# Userspace testing programs

## 构建步骤

### 安装依赖

首先需要安装 `gflags` 库。在 Ubuntu/Debian 系统上：

```bash
sudo apt-get install libgflags-dev
```

### 构建

1. 创建构建目录并配置 CMake：

```bash
mkdir -p build
cd build
cmake ..
```

2. 构建目标：

```bash
cmake --build . --target tpp_mem_access -j`nproc`
```

或者使用 make：

```bash
make tpp_mem_access -j`nproc`
```

**注意**：项目通常推荐在 Docker 环境中构建（使用 `docklf/compile_environment:glogflagtest` 镜像）。

