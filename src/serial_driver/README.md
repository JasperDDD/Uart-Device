# Serial Driver Library

这是一个独立的串口通信库，提供外部可调用的函数接口，支持自定义数据编码和解码。**新版本支持多种数据帧格式，便于移植到不同的硬件平台。**

## 主要特性

- ✅ **多帧格式支持**：支持默认、简单、固定长度等多种帧格式
- ✅ **动态配置**：运行时可以切换不同的帧格式
- ✅ **易于移植**：通过配置结构体轻松适配不同的硬件协议
- ✅ **向后兼容**：保持与原有代码的完全兼容性
- ✅ **ROS集成**：完整的ROS话题订阅和发布功能

## 数据帧格式

### 1. 默认帧格式（保持兼容性）
```
帧头(0x7B) + 组件标识符(1字节) + 数据长度(1字节) + 数据(N字节) + 校验和(1字节)
```

### 2. 简单帧格式
```
帧头(0x7C) + 数据长度(1字节) + 数据(N字节) + 帧尾(0x7D)
```

### 3. 固定长度帧格式
```
帧头(0x7E) + 固定长度数据(32字节) + 校验和(1字节)
```

### 4. 无校验和帧格式
```
帧头(0x7F) + 组件标识符(1字节) + 数据长度(1字节) + 数据(N字节)
```

### 5. 自定义帧格式
支持任意组合的帧头、帧尾、长度字段、校验和、组件标识符等。

## 帧格式配置

### FrameConfig 结构体

```cpp
struct FrameConfig {
    uint8_t frame_header;     // 帧头
    uint8_t frame_footer;     // 帧尾（可选，设为0表示不使用）
    bool use_length_field;    // 是否使用长度字段
    bool use_checksum;        // 是否使用校验和
    bool use_component;       // 是否使用组件标识符
    size_t max_data_length;   // 最大数据长度
    std::string frame_name;   // 帧格式名称
};
```

### 预定义帧格式

```cpp
#include "serial_driver/frame_examples.h"

// 获取默认帧格式
FrameConfig default_config = FrameExamples::getDefaultFrame();

// 获取简单帧格式
FrameConfig simple_config = FrameExamples::getSimpleFrame();

// 获取固定长度帧格式
FrameConfig fixed_config = FrameExamples::getFixedLengthFrame();

// 获取无校验和帧格式
FrameConfig no_checksum_config = FrameExamples::getNoChecksumFrame();
```

### 自定义帧格式

```cpp
// 创建自定义帧格式
FrameConfig custom_config = FrameExamples::createCustomFrame(
    0xAA,   // 帧头
    0x00,   // 不使用帧尾
    true,   // 使用长度字段
    true,   // 使用校验和
    false,  // 不使用组件标识符
    128,    // 最大数据长度128字节
    "CustomFrame"
);
```

## 使用方法

### 1. 基本使用（默认帧格式）

```cpp
#include "serial_driver/serial_driver.h"

// 创建串口驱动实例（使用默认帧格式）
serial_driver::SerialDriver driver;

// 设置回调函数
driver.setReceiveCallback([](const serial_driver::SerialFrame& frame) {
    std::cout << "Frame received from " << frame.component << std::endl;
});

// 初始化串口
if (driver.initialize("/dev/ttyUSB0", 115200)) {
    // 启动通信
    driver.start();
    
    // 发送数据
    std::vector<uint8_t> data = {0x01, 0x02, 0x03};
    driver.sendData(serial_driver::SerialFrame::COMPONENT_CHASSIS, data);
    
    // 停止通信
    driver.stop();
}
```

### 2. 使用自定义帧格式

```cpp
#include "serial_driver/serial_driver.h"
#include "serial_driver/frame_examples.h"

// 创建自定义帧格式的串口驱动
serial_driver::FrameConfig config = serial_driver::FrameExamples::getSimpleFrame();
serial_driver::SerialDriver driver(config);

// 设置回调函数
driver.setReceiveCallback([](const serial_driver::SerialFrame& frame) {
    std::cout << "Simple frame received, length: " << (int)frame.data_length << std::endl;
});

// 初始化串口
if (driver.initialize("/dev/ttyUSB0", 115200)) {
    driver.start();
    
    // 发送原始数据（会自动添加帧头、长度、帧尾）
    std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04};
    driver.sendRawData(data);
    
    driver.stop();
}
```

### 3. 动态切换帧格式

```cpp
#include "serial_driver/serial_driver.h"
#include "serial_driver/frame_examples.h"

serial_driver::SerialDriver driver;

if (driver.initialize("/dev/ttyUSB0", 115200)) {
    driver.start();
    
    // 使用默认格式发送
    driver.setFrameConfig(serial_driver::FrameExamples::getDefaultFrame());
    std::vector<uint8_t> data1 = {0x01, 0x02, 0x03};
    driver.sendData(serial_driver::SerialFrame::COMPONENT_CHASSIS, data1);
    
    // 切换到简单格式发送
    driver.setFrameConfig(serial_driver::FrameExamples::getSimpleFrame());
    std::vector<uint8_t> data2 = {0x04, 0x05, 0x06};
    driver.sendRawData(data2);
    
    driver.stop();
}
```

### 4. 运行帧格式示例程序

```bash
# 编译
cd ~/catkin_ws
catkin_make

# 运行帧格式示例程序
./devel/lib/serial_driver/frame_usage_example

# 或者使用ROS运行
rosrun serial_driver frame_usage_example
```

## 移植到不同硬件平台

### 步骤1：分析目标硬件的数据帧格式

例如，如果目标硬件使用以下格式：
```
帧头(0xFF) + 命令类型(1字节) + 数据长度(2字节) + 数据(N字节) + CRC16(2字节)
```

### 步骤2：创建对应的帧配置

```cpp
serial_driver::FrameConfig hardware_config = serial_driver::FrameExamples::createCustomFrame(
    0xFF,   // 帧头
    0x00,   // 不使用帧尾
    true,   // 使用长度字段（注意：这里需要2字节长度）
    true,   // 使用校验和（CRC16）
    false,  // 不使用组件标识符
    1024,   // 最大数据长度
    "HardwareFrame"
);
```

### 步骤3：继承SerialFrame类（如果需要）

如果目标硬件的数据格式与现有格式差异很大，可以继承SerialFrame类：

```cpp
class HardwareFrame : public serial_driver::SerialFrame {
public:
    uint8_t command_type;
    uint16_t crc16;
    
    // 重写序列化方法
    std::vector<uint8_t> serialize() const override {
        std::vector<uint8_t> buffer;
        buffer.push_back(header);           // 帧头 0xFF
        buffer.push_back(command_type);     // 命令类型
        buffer.push_back((data_length >> 8) & 0xFF);  // 长度高字节
        buffer.push_back(data_length & 0xFF);         // 长度低字节
        buffer.insert(buffer.end(), data.begin(), data.end());
        buffer.push_back((crc16 >> 8) & 0xFF);       // CRC16高字节
        buffer.push_back(crc16 & 0xFF);              // CRC16低字节
        return buffer;
    }
    
    // 重写反序列化方法
    bool deserialize(const std::vector<uint8_t>& buffer) override {
        if (buffer.size() < 6) return false;
        
        size_t index = 0;
        header = buffer[index++];
        command_type = buffer[index++];
        data_length = (buffer[index] << 8) | buffer[index + 1];
        index += 2;
        
        if (buffer.size() < 6 + data_length) return false;
        
        data.clear();
        for (uint16_t i = 0; i < data_length; ++i) {
            data.push_back(buffer[index++]);
        }
        
        crc16 = (buffer[index] << 8) | buffer[index + 1];
        
        return true;
    }
};
```

### 步骤4：使用自定义帧格式

```cpp
// 创建使用自定义帧格式的串口驱动
serial_driver::SerialDriver driver(hardware_config);

// 设置接收回调
driver.setReceiveCallback([](const serial_driver::SerialFrame& frame) {
    std::cout << "Hardware frame received, length: " << (int)frame.data_length << std::endl;
});

// 初始化并开始通信
if (driver.initialize("/dev/ttyUSB0", 115200)) {
    driver.start();
    
    // 发送数据
    std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04};
    driver.sendRawData(data);
    
    driver.stop();
}
```

## 编译

```bash
cd ~/catkin_ws
catkin_make
```

## 运行测试程序

### 直接运行（非ROS环境）
```bash
# 使用默认参数
./devel/lib/serial_driver/serial_test

# 指定串口和波特率
./devel/lib/serial_driver/serial_test /dev/ttyUSB0 115200

# 运行帧格式示例程序
./devel/lib/serial_driver/frame_usage_example
```

### ROS环境运行
```bash
# 设置ROS环境
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 启动ROS Master（如果需要）
roscore

# 运行测试程序
rosrun serial_driver serial_test

# 运行帧格式示例程序
rosrun serial_driver frame_usage_example

# 或者指定参数
rosrun serial_driver serial_test _port:=/dev/ttyUSB0 _baud_rate:=115200
```

### 3. 发送移动命令

程序会订阅 `/cmd_vel` 话题，接收 `geometry_msgs/Twist` 消息，并将 x、y、z 坐标通过串口发送。

#### 使用 rostopic pub 发送命令

##### 发送单个移动命令：
```bash
# 向前移动
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# 向右移动
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 1.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# 向上移动
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 1.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# 停止移动
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

##### 发送一次并退出：
```bash
# 向前移动1秒后自动停止
rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# 向右移动1秒后自动停止
rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# 向上移动1秒后自动停止
rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

##### 持续发送命令（循环发送）：
```bash
# 持续向前移动（每秒发送一次）
rostopic pub -r 1 /cmd_vel geometry_msgs/Twist "linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# 持续向右移动
rostopic pub -r 1 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# 持续向上移动
rostopic pub -r 1 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# 持续停止
rostopic pub -r 1 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

##### 使用Python脚本发送命令：
```bash
# 给脚本添加执行权限
chmod +x send_cmd_vel.py

# 发送命令：向前移动2秒
python3 send_cmd_vel.py 1.0 0.0 0.0 2.0

# 发送命令：向右移动1秒
python3 send_cmd_vel.py 0.0 1.0 0.0 1.0

# 发送命令：向上移动3秒
python3 send_cmd_vel.py 0.0 0.0 1.0 3.0
```

#### 数据格式说明

程序会将接收到的浮点数坐标乘以1000转换为整数，然后编码为以下格式：
- 字节0-1: X坐标（16位，大端序）
- 字节2-3: Y坐标（16位，大端序）
- 字节4-5: Z坐标（16位，大端序）

例如，发送 `x=1.0, y=0.5, z=0.0` 会被编码为：
- X: 1000 (0x03E8)
- Y: 500 (0x01F4)
- Z: 0 (0x0000)

最终发送的数据：`03 E8 01 F4 00 00`

### 4. 创建ROS节点使用串口驱动

#### 创建ROS节点文件
```cpp
#include "serial_driver/serial_driver.h"
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

class SerialNode {
private:
    serial_driver::SerialDriver driver_;
    ros::NodeHandle nh_;
    ros::Subscriber send_sub_;
    ros::Publisher receive_pub_;
    
public:
    SerialNode() {
        // 订阅发送话题
        send_sub_ = nh_.subscribe("serial_send", 10, &SerialNode::sendCallback, this);
        
        // 发布接收话题
        receive_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("serial_receive", 10);
        
        // 设置串口驱动回调
        driver_.setReceiveCallback([this](const serial_driver::SerialFrame& frame) {
            std_msgs::UInt8MultiArray msg;
            msg.data = frame.serialize();
            receive_pub_.publish(msg);
        });
    }
    
    bool initialize(const std::string& port, uint32_t baud_rate) {
        return driver_.initialize(port, baud_rate);
    }
    
    void start() {
        driver_.start();
    }
    
    void stop() {
        driver_.stop();
    }
    
private:
    void sendCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
        if (msg->data.size() >= 2) {
            uint8_t component = msg->data[0];
            std::vector<uint8_t> data(msg->data.begin() + 1, msg->data.end());
            driver_.sendData(static_cast<serial_driver::SerialFrame::COMPONENT>(component), data);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    std::string port;
    int baud_rate;
    private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
    private_nh.param<int>("baud_rate", baud_rate, 115200);
    
    SerialNode node;
    if (node.initialize(port, baud_rate)) {
        node.start();
        ros::spin();
        node.stop();
    }
    
    return 0;
}
```

#### 创建launch文件
```xml
<launch>
  <node name="serial_node" pkg="serial_driver" type="serial_node" output="screen">
    <param name="port" value="/dev/ttyUSB0" />
    <param name="baud_rate" value="115200" />
  </node>
</launch>
```

#### 运行ROS节点
```bash
# 启动ROS节点
roslaunch serial_driver serial_node.launch

# 或者使用rosrun
rosrun serial_driver serial_node _port:=/dev/ttyUSB0 _baud_rate:=115200
```

#### 发送和接收数据
```bash
# 发送数据到底盘组件
rostopic pub /serial_send std_msgs/UInt8MultiArray "data: [3, 1, 100, 200, 50, 30]"

# 监听接收到的数据
rostopic echo /serial_receive

# 查看话题列表
rostopic list

# 查看话题信息
rostopic info /serial_send
```

### 4. 自定义数据编码和解码

在 `main.cpp` 中，你可以看到如何自定义数据编码和解码：

#### 编码函数示例
```cpp
// 编码移动命令
std::vector<uint8_t> encodeMoveCommand(int x, int y, int z, int speed) {
    std::vector<uint8_t> data;
    data.push_back(static_cast<uint8_t>(x & 0xFF));
    data.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
    // ... 更多编码逻辑
    return data;
}
```

#### 解码函数示例
```cpp
// 解码移动命令
void decodeMoveCommand(const std::vector<uint8_t>& data) {
    if (data.size() >= 7) {
        int x = static_cast<int>(data[0]) | (static_cast<int>(data[1]) << 8);
        int y = static_cast<int>(data[2]) | (static_cast<int>(data[3]) << 8);
        // ... 更多解码逻辑
    }
}
```

## API 参考

### SerialDriver 类

#### 构造函数
```cpp
SerialDriver();
SerialDriver(const FrameConfig& config);
```

#### 帧配置
```cpp
void setFrameConfig(const FrameConfig& config);
FrameConfig getFrameConfig() const;
```

#### 初始化
```cpp
bool initialize(const std::string& port, uint32_t baud_rate, int timeout_ms = 1000);
```

#### 启动/停止
```cpp
void start();
void stop();
```

#### 发送数据
```cpp
bool sendFrame(const SerialFrame& frame);
bool sendData(SerialFrame::COMPONENT component, const std::vector<uint8_t>& data);
bool sendRawData(const std::vector<uint8_t>& data);
```

#### 设置回调
```cpp
void setReceiveCallback(ReceiveCallback callback);
void setStatusCallback(StatusCallback callback);
void setRawDataCallback(RawDataCallback callback);
```

#### 状态查询
```cpp
bool isOpen() const;
std::string getStatus() const;
```

### SerialFrame 结构体

#### 成员变量
- `header`: 帧头
- `component`: 组件标识符
- `data_length`: 数据长度
- `data`: 数据内容
- `checksum`: 校验和
- `footer`: 帧尾

#### 成员函数
```cpp
uint8_t calculateChecksum() const;
bool verifyChecksum() const;
std::vector<uint8_t> serialize() const;
bool deserialize(const std::vector<uint8_t>& buffer);
std::vector<uint8_t> serializeWithConfig(const FrameConfig& config) const;
bool deserializeWithConfig(const std::vector<uint8_t>& buffer, const FrameConfig& config);
bool isValid(const FrameConfig& config) const;
size_t getFrameLength(const FrameConfig& config) const;
```

## 回调函数类型

```cpp
using SendCallback = std::function<void(const SerialFrame&)>;
using ReceiveCallback = std::function<void(const SerialFrame&)>;
using StatusCallback = std::function<void(const std::string&)>;
using RawDataCallback = std::function<void(const std::vector<uint8_t>&)>;
```

## 示例：发送移动命令

```cpp
// 创建移动命令数据
std::vector<uint8_t> move_data = encodeMoveCommand(100, 200, 50, 30);
move_data.insert(move_data.begin(), 0x01); // 命令类型：移动

// 发送到底盘组件
driver.sendData(serial_driver::SerialFrame::COMPONENT_CHASSIS, move_data);
```

## 注意事项

1. 确保串口设备存在且有权限访问
2. 波特率需要与设备端匹配
3. 数据长度不能超过配置的最大长度
4. 程序会根据配置自动计算和验证校验和
5. 接收线程会根据配置自动处理数据帧的解析和验证
6. 帧格式配置在初始化后可以动态修改，但建议在发送数据前确定好格式

## 故障排除

### 串口打开失败
```bash
# 检查串口设备
ls -l /dev/ttyUSB*

# 添加用户到dialout组
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 编译错误
- 确保已安装serial库：`sudo apt install ros-noetic-serial`
- 检查CMake版本：`cmake --version`

### ROS相关问题
```bash
# 检查ROS环境
echo $ROS_DISTRO
echo $ROS_PACKAGE_PATH

# 重新设置ROS环境
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 检查包是否正确安装
rospack find serial_driver

# 查看节点列表
rosnode list

# 查看话题列表
rostopic list
```

### 帧格式相关问题
```bash
# 运行帧格式示例程序查看所有可用格式
./devel/lib/serial_driver/frame_usage_example

# 检查帧配置是否正确
# 确保帧头、帧尾、长度字段等配置与硬件协议匹配
``` 