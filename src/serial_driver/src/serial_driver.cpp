#include "serial_driver/serial_driver.h"
#include <memory>
#include <mutex>
#include <thread>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <serial/serial.h>

namespace serial_driver {

// 定义组件名称数组
const std::string SerialFrame::COMPONENT_NAME[] = {
    "HEAD",
    "LEFT_ARM", 
    "RIGHT_ARM",
    "CHASSIS"
};

// SerialFrame 方法实现
uint8_t SerialFrame::calculateChecksum() const {
    uint8_t sum = header + static_cast<uint8_t>(component) + data_length;
    for (uint8_t byte : data) {
        sum += byte;
    }
    return (sum % 16);
}

bool SerialFrame::verifyChecksum() const {
    return checksum == calculateChecksum();
}

std::vector<uint8_t> SerialFrame::serialize() const {
    std::vector<uint8_t> buffer;
    buffer.push_back(header);
    buffer.push_back(static_cast<uint8_t>(component));
    buffer.push_back(data_length);
    buffer.insert(buffer.end(), data.begin(), data.end());
    buffer.push_back(checksum);
    return buffer;
}

bool SerialFrame::deserialize(const std::vector<uint8_t>& buffer) {
    if (buffer.size() < 4) { // 最小长度：帧头+组件+长度+校验和
        return false;
    }
    
    uint8_t index = 0;
    header = buffer[index++];
    component = static_cast<COMPONENT>(buffer[index++]);
    data_length = buffer[index++];
    
    if (buffer.size() < 4 + data_length) {
        return false;
    }
    
    data.clear();
    for (uint8_t i = 0; i < data_length; ++i) {
        data.push_back(buffer[index++]);
    }
    
    checksum = buffer[index];
    
    return verifyChecksum();
}

// 新增的帧配置相关方法
std::vector<uint8_t> SerialFrame::serializeWithConfig(const FrameConfig& config) const {
    std::vector<uint8_t> buffer;
    buffer.push_back(config.frame_header);
    
    if (config.use_component) {
        buffer.push_back(static_cast<uint8_t>(component));
    }
    
    if (config.use_length_field) {
        buffer.push_back(data_length);
    }
    
    buffer.insert(buffer.end(), data.begin(), data.end());
    
    if (config.use_checksum) {
        buffer.push_back(checksum);
    }
    
    if (config.frame_footer != 0) {
        buffer.push_back(config.frame_footer);
    }
    
    return buffer;
}

bool SerialFrame::deserializeWithConfig(const std::vector<uint8_t>& buffer, const FrameConfig& config) {
    if (buffer.empty()) return false;
    
    size_t index = 0;
    
    // 检查帧头
    if (buffer[index++] != config.frame_header) {
        return false;
    }
    
    // 解析组件标识符
    if (config.use_component) {
        if (index >= buffer.size()) return false;
        component = static_cast<COMPONENT>(buffer[index++]);
    }
    
    // 解析数据长度
    if (config.use_length_field) {
        if (index >= buffer.size()) return false;
        data_length = buffer[index++];
    } else {
        // 固定长度，使用配置中的最大长度
        data_length = config.max_data_length;
    }
    
    // 解析数据
    if (index + data_length > buffer.size()) return false;
    data.clear();
    for (uint8_t i = 0; i < data_length; ++i) {
        data.push_back(buffer[index++]);
    }
    
    // 解析校验和
    if (config.use_checksum) {
        if (index >= buffer.size()) return false;
        checksum = buffer[index++];
    }
    
    // 检查帧尾
    if (config.frame_footer != 0) {
        if (index >= buffer.size() || buffer[index] != config.frame_footer) {
            return false;
        }
    }
    
    return true;
}

bool SerialFrame::isValid(const FrameConfig& config) const {
    if (data.size() > config.max_data_length) {
        return false;
    }
    
    if (config.use_checksum && !verifyChecksum()) {
        return false;
    }
    
    return true;
}

size_t SerialFrame::getFrameLength(const FrameConfig& config) const {
    size_t length = 1; // 帧头
    
    if (config.use_component) {
        length += 1;
    }
    
    if (config.use_length_field) {
        length += 1;
    }
    
    length += data.size();
    
    if (config.use_checksum) {
        length += 1;
    }
    
    if (config.frame_footer != 0) {
        length += 1;
    }
    
    return length;
}

std::string SerialFrame::getFrameType() const {
    return "SerialFrame";
}

// SerialDriver 方法实现
SerialDriver::SerialDriver() 
    : running_(false), timeout_ms_(1000), status_("Not initialized"), frame_config_(FrameConfig::getDefaultConfig()) {
}

SerialDriver::SerialDriver(const FrameConfig& config)
    : running_(false), timeout_ms_(1000), status_("Not initialized"), frame_config_(config) {
}

SerialDriver::~SerialDriver() {
    stop();
}

void SerialDriver::setFrameConfig(const FrameConfig& config) {
    frame_config_ = config;
    publishStatus("Frame config updated to: " + config.frame_name);
}

FrameConfig SerialDriver::getFrameConfig() const {
    return frame_config_;
}

bool SerialDriver::initialize(const std::string& port, uint32_t baud_rate, int timeout_ms) {
    try {
        port_name_ = port;
        baud_rate_ = baud_rate;
        timeout_ms_ = timeout_ms;
        
        serial_ = std::make_unique<serial::Serial>();
        serial_->setPort(port);
        serial_->setBaudrate(baud_rate);
        
        // 创建Timeout对象
        serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms_);
        serial_->setTimeout(timeout);
        
        serial_->open();
        
        if (serial_->isOpen()) {
            status_ = "Serial port opened successfully";
            publishStatus(status_);
            return true;
        } else {
            status_ = "Failed to open serial port";
            publishStatus(status_);
            return false;
        }
    } catch (const serial::IOException& e) {
        status_ = "Serial port error: " + std::string(e.what());
        publishStatus(status_);
        return false;
    }
}

void SerialDriver::start() {
    if (!serial_ || !serial_->isOpen()) {
        publishStatus("Cannot start: serial port not open");
        return;
    }
    
    if (running_.load()) {
        publishStatus("Already running");
        return;
    }
    
    running_.store(true);
    receive_thread_ = std::thread(&SerialDriver::receiveThread, this);
    publishStatus("Serial driver started");
}

void SerialDriver::stop() {
    if (!running_.load()) {
        return;
    }
    
    running_.store(false);
    publishStatus("Stopping serial driver...");
    
    // 等待接收线程结束，但设置更短的超时
    if (receive_thread_.joinable()) {
        // 等待最多500毫秒
        auto start_time = std::chrono::steady_clock::now();
        while (receive_thread_.joinable() && 
               std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(500)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        
        if (receive_thread_.joinable()) {
            publishStatus("Warning: Receive thread did not stop gracefully, detaching...");
            receive_thread_.detach(); // 强制分离线程，让程序能够退出
        }
    }
    
    if (serial_ && serial_->isOpen()) {
        try {
            serial_->close();
        } catch (const serial::IOException& e) {
            publishStatus("Error closing serial port: " + std::string(e.what()));
        }
    }
    
    publishStatus("Serial driver stopped");
}

bool SerialDriver::sendFrame(const SerialFrame& frame) {
    if (!serial_ || !serial_->isOpen()) {
        publishStatus("Cannot send: serial port not open");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(serial_mutex_);
    
    try {
        // 使用当前帧配置序列化
        std::vector<uint8_t> buffer = frame.serializeWithConfig(frame_config_);
        size_t written = serial_->write(buffer);
        
        if (written == buffer.size()) {
            publishStatus("Frame sent successfully using " + frame_config_.frame_name);
            return true;
        } else {
            publishStatus("Failed to send complete frame");
            return false;
        }
    } catch (const serial::IOException& e) {
        publishStatus("Send error: " + std::string(e.what()));
        return false;
    }
}

bool SerialDriver::sendData(SerialFrame::COMPONENT component, const std::vector<uint8_t>& data) {
    SerialFrame frame;
    frame.component = component;
    frame.data = data;
    frame.data_length = data.size();
    
    if (frame_config_.use_checksum) {
        frame.checksum = frame.calculateChecksum();
    }
    
    return sendFrame(frame);
}

bool SerialDriver::sendRawData(const std::vector<uint8_t>& data) {
    if (!serial_ || !serial_->isOpen()) {
        publishStatus("Cannot send: serial port not open");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(serial_mutex_);
    
    try {
        std::vector<uint8_t> buffer;
        buffer.push_back(frame_config_.frame_header);
        
        if (frame_config_.use_length_field) {
            buffer.push_back(static_cast<uint8_t>(data.size()));
        }
        
        buffer.insert(buffer.end(), data.begin(), data.end());
        
        if (frame_config_.use_checksum) {
            // 计算简单的校验和
            uint8_t checksum = 0;
            for (uint8_t byte : buffer) {
                checksum += byte;
            }
            buffer.push_back(checksum);
        }
        
        if (frame_config_.frame_footer != 0) {
            buffer.push_back(frame_config_.frame_footer);
        }
        
        size_t written = serial_->write(buffer);
        
        if (written == buffer.size()) {
            publishStatus("Raw data sent successfully using " + frame_config_.frame_name);
            return true;
        } else {
            publishStatus("Failed to send complete raw data");
            return false;
        }
    } catch (const serial::IOException& e) {
        publishStatus("Send error: " + std::string(e.what()));
        return false;
    }
}

void SerialDriver::setReceiveCallback(ReceiveCallback callback) {
    receive_callback_ = callback;
}

void SerialDriver::setStatusCallback(StatusCallback callback) {
    status_callback_ = callback;
}

void SerialDriver::setRawDataCallback(RawDataCallback callback) {
    raw_data_callback_ = callback;
}

bool SerialDriver::isOpen() const {
    return serial_ && serial_->isOpen();
}

std::string SerialDriver::getStatus() const {
    return status_;
}

void SerialDriver::receiveThread() {
    std::vector<uint8_t> buffer;
    buffer.reserve(1024);
    
    while (running_.load()) {
        try {
            if (serial_ && serial_->isOpen()) {
                // 读取可用数据
                size_t available = serial_->available();
                if (available > 0) {
                    std::string new_data_str = serial_->read(available);
                    
                    // 将string转换为vector<uint8_t>
                    std::vector<uint8_t> new_data(new_data_str.begin(), new_data_str.end());
                    
                    // 添加到缓冲区
                    {
                        std::lock_guard<std::mutex> lock(buffer_mutex_);
                        receive_buffer_.insert(receive_buffer_.end(), new_data.begin(), new_data.end());
                    }
                    
                    // 处理接收到的数据
                    processReceivedData();
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } catch (const serial::IOException& e) {
            publishStatus("Receive error: " + std::string(e.what()));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void SerialDriver::processReceivedData() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    while (findFrameInBufferWithConfig()) {
        // 帧已经在findFrameInBufferWithConfig中被处理
    }
}

bool SerialDriver::findFrameInBuffer() {
    // 使用新的配置方法
    return findFrameInBufferWithConfig();
}

bool SerialDriver::findFrameInBufferWithConfig() {
    if (receive_buffer_.size() < frame_config_.getMinFrameLength()) {
        return false;
    }
    
    // 查找帧头
    auto it = std::find(receive_buffer_.begin(), receive_buffer_.end(), frame_config_.frame_header);
    if (it == receive_buffer_.end()) {
        return false;
    }
    
    size_t frame_start = std::distance(receive_buffer_.begin(), it);
    size_t min_frame_length = frame_config_.getMinFrameLength();
    
    if (receive_buffer_.size() < frame_start + min_frame_length) {
        return false;
    }
    
    // 尝试解析帧
    std::vector<uint8_t> frame_data(receive_buffer_.begin() + frame_start, receive_buffer_.end());
    auto frame = parseFrameWithConfig(frame_data);
    
    if (frame) {
        // 找到完整帧，调用回调
        if (receive_callback_) {
            receive_callback_(*frame);
        }
        
        // 移除已处理的帧数据
        size_t frame_length = frame->getFrameLength(frame_config_);
        receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + frame_start + frame_length);
        
        return true;
    }
    
    return false;
}

std::unique_ptr<SerialFrame> SerialDriver::parseFrameWithConfig(const std::vector<uint8_t>& frame_data) {
    auto frame = std::make_unique<SerialFrame>();
    if (frame->deserializeWithConfig(frame_data, frame_config_)) {
        return frame;
    }
    return nullptr;
}

void SerialDriver::publishStatus(const std::string& status) {
    status_ = status;
    if (status_callback_) {
        status_callback_(status);
    }
}

} // namespace serial_driver 