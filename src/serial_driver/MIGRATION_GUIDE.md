# Serial Driver 移植指南

本指南将帮助你快速将 `serial_driver` 库移植到不同的硬件平台和协议格式。

## 目录

1. [快速开始](#快速开始)
2. [帧格式分析](#帧格式分析)
3. [配置适配](#配置适配)
4. [高级定制](#高级定制)
5. [常见问题](#常见问题)

## 快速开始

### 步骤1：分析目标硬件协议

首先，你需要了解目标硬件使用的数据帧格式。例如：

```
目标硬件协议示例：
帧头(0xFF) + 命令类型(1字节) + 数据长度(2字节) + 数据(N字节) + CRC16(2字节)
```

### 步骤2：创建帧配置

```cpp
#include "serial_driver/frame_examples.h"

// 创建对应的帧配置
serial_driver::FrameConfig hardware_config = serial_driver::FrameExamples::createCustomFrame(
    0xFF,   // 帧头
    0x00,   // 不使用帧尾
    true,   // 使用长度字段
    true,   // 使用校验和
    false,  // 不使用组件标识符
    1024,   // 最大数据长度
    "HardwareFrame"
);
```

### 步骤3：使用配置

```cpp
#include "serial_driver/serial_driver.h"

// 创建使用自定义帧格式的串口驱动
serial_driver::SerialDriver driver(hardware_config);

// 设置接收回调
driver.setReceiveCallback([](const serial_driver::SerialFrame& frame) {
    std::cout << "收到硬件帧，长度: " << (int)frame.data_length << std::endl;
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

## 帧格式分析

### 常见帧格式类型

#### 1. 固定帧头 + 长度 + 数据
```
帧头(固定值) + 数据长度(N字节) + 数据(N字节)
```
**适用场景**：简单的数据传输协议

#### 2. 固定帧头 + 长度 + 数据 + 校验
```
帧头(固定值) + 数据长度(N字节) + 数据(N字节) + 校验和(N字节)
```
**适用场景**：需要数据完整性验证的协议

#### 3. 固定帧头 + 命令 + 长度 + 数据 + 校验
```
帧头(固定值) + 命令类型(1字节) + 数据长度(N字节) + 数据(N字节) + 校验和(N字节)
```
**适用场景**：多命令类型的协议

#### 4. 固定帧头 + 数据 + 固定帧尾
```
帧头(固定值) + 数据(N字节) + 帧尾(固定值)
```
**适用场景**：使用帧尾标识结束的协议

#### 5. 固定帧头 + 长度 + 数据 + 帧尾
```
帧头(固定值) + 数据长度(N字节) + 数据(N字节) + 帧尾(固定值)
```
**适用场景**：既有长度又有帧尾的协议

### 长度字段类型

#### 1字节长度字段
```cpp
// 最大数据长度：255字节
FrameConfig config = FrameExamples::createCustomFrame(
    0xAA,   // 帧头
    0x00,   // 不使用帧尾
    true,   // 使用长度字段
    true,   // 使用校验和
    false,  // 不使用组件标识符
    255,    // 最大数据长度
    "OneByteLength"
);
```

#### 2字节长度字段
```cpp
// 最大数据长度：65535字节
// 注意：需要自定义序列化方法
class TwoByteLengthFrame : public serial_driver::SerialFrame {
public:
    std::vector<uint8_t> serialize() const override {
        std::vector<uint8_t> buffer;
        buffer.push_back(header);
        buffer.push_back((data_length >> 8) & 0xFF);  // 长度高字节
        buffer.push_back(data_length & 0xFF);         // 长度低字节
        buffer.insert(buffer.end(), data.begin(), data.end());
        if (checksum != 0) {
            buffer.push_back(checksum);
        }
        return buffer;
    }
};
```

## 配置适配

### 基本配置参数

```cpp
struct FrameConfig {
    uint8_t frame_header;     // 帧头
    uint8_t frame_footer;     // 帧尾（可选）
    bool use_length_field;    // 是否使用长度字段
    bool use_checksum;        // 是否使用校验和
    bool use_component;       // 是否使用组件标识符
    size_t max_data_length;   // 最大数据长度
    std::string frame_name;   // 帧格式名称
};
```

### 常见配置组合

#### 配置1：简单无校验协议
```cpp
FrameConfig simple_config = {
    0x7C,   // 帧头
    0x7D,   // 帧尾
    true,   // 使用长度字段
    false,  // 不使用校验和
    false,  // 不使用组件标识符
    255,    // 最大数据长度
    "SimpleFrame"
};
```

#### 配置2：带校验的协议
```cpp
FrameConfig checksum_config = {
    0x7B,   // 帧头
    0x00,   // 不使用帧尾
    true,   // 使用长度字段
    true,   // 使用校验和
    true,   // 使用组件标识符
    255,    // 最大数据长度
    "ChecksumFrame"
};
```

#### 配置3：固定长度协议
```cpp
FrameConfig fixed_config = {
    0x7E,   // 帧头
    0x00,   // 不使用帧尾
    false,  // 不使用长度字段（固定长度）
    true,   // 使用校验和
    false,  // 不使用组件标识符
    32,     // 固定32字节数据
    "FixedLengthFrame"
};
```

### 校验和算法适配

#### 默认校验和算法
```cpp
// 当前实现：所有字节相加后取模16
uint8_t calculateChecksum() const {
    uint8_t sum = header + static_cast<uint8_t>(component) + data_length;
    for (uint8_t byte : data) {
        sum += byte;
    }
    return (sum % 16);
}
```

#### 自定义校验和算法
```cpp
// 如果需要不同的校验和算法，可以继承SerialFrame类
class CustomChecksumFrame : public serial_driver::SerialFrame {
public:
    // 重写校验和计算方法
    uint8_t calculateChecksum() const override {
        uint8_t sum = 0;
        sum ^= header;
        if (use_component) sum ^= static_cast<uint8_t>(component);
        if (use_length_field) sum ^= data_length;
        for (uint8_t byte : data) {
            sum ^= byte;
        }
        return sum;
    }
};
```

## 高级定制

### 自定义帧类

如果目标硬件的数据格式与现有格式差异很大，可以创建自定义帧类：

```cpp
#include "serial_driver/serial_driver.h"

class HardwareFrame : public serial_driver::SerialFrame {
public:
    uint8_t command_type;
    uint16_t crc16;
    
    HardwareFrame() : command_type(0), crc16(0) {}
    
    // 重写序列化方法
    std::vector<uint8_t> serialize() const override {
        std::vector<uint8_t> buffer;
        buffer.push_back(header);           // 帧头
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
    
    // 重写校验和验证方法
    bool verifyChecksum() const override {
        // 实现CRC16验证逻辑
        uint16_t calculated_crc = calculateCRC16();
        return calculated_crc == crc16;
    }
    
private:
    uint16_t calculateCRC16() const {
        // 实现CRC16计算逻辑
        // 这里使用简单的示例，实际应用中需要使用标准的CRC16算法
        uint16_t crc = 0xFFFF;
        crc ^= header;
        crc ^= command_type;
        crc ^= (data_length >> 8) & 0xFF;
        crc ^= data_length & 0xFF;
        for (uint8_t byte : data) {
            crc ^= byte;
        }
        return crc;
    }
};
```

### 自定义解析器

如果需要更复杂的帧解析逻辑，可以创建自定义解析器：

```cpp
class CustomFrameParser {
public:
    // 在缓冲区中查找完整的帧
    bool findFrame(const std::vector<uint8_t>& buffer, size_t& frame_start, size_t& frame_length) {
        // 查找帧头
        for (size_t i = 0; i < buffer.size() - 1; ++i) {
            if (buffer[i] == 0xFF) {  // 帧头
                // 检查是否有足够的数据来读取长度
                if (i + 3 < buffer.size()) {
                    uint16_t data_length = (buffer[i + 2] << 8) | buffer[i + 3];
                    size_t total_length = 6 + data_length;  // 帧头+命令+长度+数据+CRC16
                    
                    if (i + total_length <= buffer.size()) {
                        frame_start = i;
                        frame_length = total_length;
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    // 解析帧数据
    std::unique_ptr<HardwareFrame> parseFrame(const std::vector<uint8_t>& frame_data) {
        auto frame = std::make_unique<HardwareFrame>();
        if (frame->deserialize(frame_data)) {
            return frame;
        }
        return nullptr;
    }
};
```

## 常见问题

### Q1: 如何适配不同的长度字段大小？

**A**: 如果硬件使用2字节长度字段，需要自定义序列化方法：

```cpp
class TwoByteLengthFrame : public serial_driver::SerialFrame {
public:
    std::vector<uint8_t> serialize() const override {
        std::vector<uint8_t> buffer;
        buffer.push_back(header);
        buffer.push_back((data_length >> 8) & 0xFF);  // 长度高字节
        buffer.push_back(data_length & 0xFF);         // 长度低字节
        buffer.insert(buffer.end(), data.begin(), data.end());
        if (checksum != 0) {
            buffer.push_back(checksum);
        }
        return buffer;
    }
};
```

### Q2: 如何适配不同的校验和算法？

**A**: 重写 `calculateChecksum()` 方法：

```cpp
class CustomChecksumFrame : public serial_driver::SerialFrame {
public:
    uint8_t calculateChecksum() const override {
        // 实现你的校验和算法
        uint8_t sum = 0;
        for (uint8_t byte : data) {
            sum += byte;
        }
        return (sum & 0xFF);  // 取低8位
    }
};
```

### Q3: 如何处理多帧尾的情况？

**A**: 创建自定义解析逻辑：

```cpp
bool findFrameWithMultipleFooters(const std::vector<uint8_t>& buffer, size_t& frame_start, size_t& frame_length) {
    for (size_t i = 0; i < buffer.size() - 2; ++i) {
        if (buffer[i] == 0x80) {  // 帧头
            // 查找第一个帧尾
            for (size_t j = i + 1; j < buffer.size() - 1; ++j) {
                if (buffer[j] == 0x81) {  // 第一个帧尾
                    // 查找第二个帧尾
                    for (size_t k = j + 1; k < buffer.size(); ++k) {
                        if (buffer[k] == 0x82) {  // 第二个帧尾
                            frame_start = i;
                            frame_length = k - i + 1;
                            return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}
```

### Q4: 如何调试帧解析问题？

**A**: 使用以下调试方法：

```cpp
// 1. 启用详细日志
driver.setStatusCallback([](const std::string& status) {
    std::cout << "[DEBUG] " << status << std::endl;
});

// 2. 打印接收到的原始数据
driver.setRawDataCallback([](const std::vector<uint8_t>& data) {
    std::cout << "Raw data: ";
    for (uint8_t byte : data) {
        printf("%02X ", byte);
    }
    std::cout << std::endl;
});

// 3. 打印帧配置信息
FrameFactory::printFrameConfig(driver.getFrameConfig());
```

### Q5: 如何优化性能？

**A**: 使用以下优化方法：

```cpp
// 1. 预分配缓冲区大小
std::vector<uint8_t> data;
data.reserve(1024);  // 预分配1KB

// 2. 使用引用避免拷贝
void processFrame(const serial_driver::SerialFrame& frame) {
    // 使用引用，避免拷贝
    const auto& data = frame.data;
    // 处理数据...
}

// 3. 批量处理数据
std::vector<serial_driver::SerialFrame> frames;
// 收集多个帧后批量处理
```

## 总结

通过使用 `FrameConfig` 结构体和预定义的帧格式，你可以快速适配大多数硬件协议。对于复杂的协议，可以通过继承 `SerialFrame` 类来实现自定义的序列化和反序列化逻辑。

记住：
1. 先分析目标硬件的帧格式
2. 选择合适的配置或创建自定义配置
3. 测试帧解析是否正确
4. 优化性能和稳定性

如有问题，请参考示例程序 `frame_usage_example` 或查看完整的API文档。 