# HLS Servo Project

这是一个用于控制HLS伺服电机的C++项目。

## 依赖项

项目需要以下依赖项：

### 方法1：安装独立的serial库（推荐）

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install libserial-dev

# CentOS/RHEL
sudo yum install libserial-devel

# macOS (使用Homebrew)
brew install libserial
```

### 方法2：使用ROS环境

如果您使用ROS，serial库通常已经包含在ROS安装中：

```bash
# 确保ROS已正确安装并配置
source /opt/ros/noetic/setup.bash  # 对于ROS Noetic
# 或
source /opt/ros/melodic/setup.bash  # 对于ROS Melodic
```

## 编译

### 1. 创建构建目录

```bash
mkdir build
cd build
```

### 2. 配置项目

```bash
cmake ..
```

### 3. 编译

```bash
make -j$(nproc)
```

## 运行

编译成功后，运行程序：

```bash
./hls_test
```

**注意**：程序需要访问串口设备（默认`/dev/ttyUSB0`），确保：
- 设备已连接
- 用户有适当的权限（通常需要将用户添加到`dialout`组）

```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER
# 重新登录或重启系统使更改生效
```

## 跨平台支持

CMakeLists.txt已配置为自动检测serial库的位置：

1. **pkg-config方式**：优先使用系统包管理器安装的库
2. **ROS环境**：自动检测ROS安装的serial库
3. **独立安装**：查找标准系统路径中的库

## 故障排除

### 找不到serial库

如果编译时报告找不到serial库，请尝试：

```bash
# 检查serial库是否已安装
pkg-config --exists serial && echo "Serial library found via pkg-config"

# 或者检查ROS环境
ls /opt/ros/*/include/serial/serial.h 2>/dev/null || echo "Serial library not found in ROS"

# 手动安装
sudo apt-get install libserial-dev
```

### 权限问题

如果运行时出现权限错误：

```bash
# 检查设备权限
ls -la /dev/ttyUSB0

# 临时更改权限（不推荐用于生产环境）
sudo chmod 666 /dev/ttyUSB0

# 永久解决方案：添加用户到dialout组
sudo usermod -a -G dialout $USER
```

## 项目结构

```
.
├── CMakeLists.txt          # CMake配置文件
├── README.md              # 本文件
├── include/               # 头文件目录
│   └── feetech/          # Feetech伺服相关头文件
│       ├── HLS.hpp       # HLS伺服类定义
│       └── feetech.hpp   # 基础伺服类定义
└── src/                  # 源代码目录
    └── HLS/              # HLS相关源代码
        └── test.cpp      # 测试程序
``` 