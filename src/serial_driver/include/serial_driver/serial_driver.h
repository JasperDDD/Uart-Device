#ifndef SERIAL_DRIVER_H
#define SERIAL_DRIVER_H

#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>


// 前向声明
namespace serial {
    class Serial;
}

namespace serial_driver {

// 数据帧配置结构体
struct FrameConfig {
    uint8_t frame_header;     // 帧头
    uint8_t frame_footer;     // 帧尾（可选，设为0表示不使用）
    bool use_length_field;    // 是否使用长度字段
    bool use_checksum;        // 是否使用校验和
    bool use_component;       // 是否使用组件标识符
    size_t max_data_length;   // 最大数据长度
    std::string frame_name;   // 帧格式名称
    
    // 获取最小帧长度
    size_t getMinFrameLength() const {
        size_t length = 1; // 帧头
        
        if (use_component) {
            length += 1;
        }
        
        if (use_length_field) {
            length += 1;
        }
        
        if (use_checksum) {
            length += 1;
        }
        
        if (frame_footer != 0) {
            length += 1;
        }
        
        return length;
    }
    
    // 获取最大帧长度
    size_t getMaxFrameLength() const {
        return getMinFrameLength() + max_data_length;
    }
    
    // 默认配置（保持与原有格式兼容）
    static FrameConfig getDefaultConfig() {
        return {
            0x7B,   // frame_header
            0x00,   // frame_footer (不使用)
            true,   // use_length_field
            true,   // use_checksum
            true,   // use_component
            255,    // max_data_length
            "DefaultFrame"
        };
    }
    
    // 简单帧配置示例
    static FrameConfig getSimpleConfig() {
        return {
            0x7C,   // frame_header
            0x7D,   // frame_footer
            true,   // use_length_field
            false,  // use_checksum
            false,  // use_component
            255,    // max_data_length
            "SimpleFrame"
        };
    }
};

// 数据帧结构
struct SerialFrame {
    enum COMPONENT {
        COMPONENT_HEAD = 0x00,
        COMPONENT_LEFT_ARM = 0x01,
        COMPONENT_RIGHT_ARM = 0x02,
        COMPONENT_CHASSIS = 0x03,
    };

    static const std::string COMPONENT_NAME[];
    
    uint8_t header;           // 帧头
    COMPONENT component;      // 组件标识符
    uint8_t data_length;      // 数据长度
    std::vector<uint8_t> data; // 数据
    uint8_t checksum;         // 校验和
    uint8_t footer;           // 帧尾
    
    SerialFrame() : header(0x7B), component(COMPONENT_CHASSIS), data_length(0), checksum(0), footer(0) {}
    
    // 计算校验和
    uint8_t calculateChecksum() const;
    
    // 验证校验和
    bool verifyChecksum() const;
    
    // 序列化为字节数组
    std::vector<uint8_t> serialize() const;
    
    // 从字节数组反序列化
    bool deserialize(const std::vector<uint8_t>& buffer);
    
    // 使用配置进行序列化
    std::vector<uint8_t> serializeWithConfig(const FrameConfig& config) const;
    
    // 使用配置进行反序列化
    bool deserializeWithConfig(const std::vector<uint8_t>& buffer, const FrameConfig& config);
    
    // 验证帧的有效性
    bool isValid(const FrameConfig& config) const;
    
    // 获取帧长度
    size_t getFrameLength(const FrameConfig& config) const;
    
    // 获取帧类型
    std::string getFrameType() const;
};

// 回调函数类型定义
using SendCallback = std::function<void(const SerialFrame&)>;
using ReceiveCallback = std::function<void(const SerialFrame&)>;
using StatusCallback = std::function<void(const std::string&)>;
using RawDataCallback = std::function<void(const std::vector<uint8_t>&)>;

class SerialDriver {
public:
    SerialDriver();
    explicit SerialDriver(const FrameConfig& config);
    ~SerialDriver();
    
    // 设置帧配置
    void setFrameConfig(const FrameConfig& config);
    
    // 获取当前帧配置
    FrameConfig getFrameConfig() const;
    
    // 初始化串口
    bool initialize(const std::string& port, uint32_t baud_rate, int timeout_ms = 1000);
    
    // 启动串口通信线程
    void start();
    
    // 停止串口通信线程
    void stop();
    
    // 发送数据帧
    bool sendFrame(const SerialFrame& frame);
    
    // 发送自定义数据
    bool sendData(SerialFrame::COMPONENT component, const std::vector<uint8_t>& data);
    
    // 发送原始数据（使用当前帧配置）
    bool sendRawData(const std::vector<uint8_t>& data);
    
    // 设置回调函数
    void setReceiveCallback(ReceiveCallback callback);
    void setStatusCallback(StatusCallback callback);
    void setRawDataCallback(RawDataCallback callback);
    
    // 检查串口是否打开
    bool isOpen() const;
    
    // 获取串口状态
    std::string getStatus() const;

private:
    // 串口对象
    std::unique_ptr<serial::Serial> serial_;
    
    // 线程相关
    std::thread receive_thread_;
    std::atomic<bool> running_;
    std::mutex serial_mutex_;
    
    // 接收缓冲区
    std::vector<uint8_t> receive_buffer_;
    std::mutex buffer_mutex_;
    
    // 回调函数
    ReceiveCallback receive_callback_;
    StatusCallback status_callback_;
    RawDataCallback raw_data_callback_;
    
    // 配置参数
    std::string port_name_;
    uint32_t baud_rate_;
    int timeout_ms_;
    std::string status_;
    FrameConfig frame_config_;
    
    // 私有方法
    void receiveThread();
    void processReceivedData();
    bool findFrameInBuffer();
    void publishStatus(const std::string& status);
    
    // 帧解析方法
    bool findFrameInBufferWithConfig();
    std::unique_ptr<SerialFrame> parseFrameWithConfig(const std::vector<uint8_t>& frame_data);
};

} // namespace serial_driver

#endif // SERIAL_DRIVER_H 