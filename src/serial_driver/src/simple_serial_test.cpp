#include <iostream>
#include <serial/serial.h>
#include <thread>
#include <chrono>
#include <signal.h>
#include <atomic>

std::atomic<bool> running(true);

void signalHandler(int sig) {
    running = false;
    std::cout << "\nReceived signal " << sig << ", shutting down..." << std::endl;
}

int main(int argc, char** argv) {
    std::string port = "/dev/ttyCH343USB0";
    uint32_t baud_rate = 115200;
    
    if (argc >= 2) {
        port = argv[1];
    }
    if (argc >= 3) {
        baud_rate = std::stoi(argv[2]);
    }
    
    std::cout << "Simple Serial Test Program" << std::endl;
    std::cout << "Port: " << port << std::endl;
    std::cout << "Baud rate: " << baud_rate << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl;
    
    // 设置信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // 创建串口对象
        serial::Serial serial;
        serial.setPort(port);
        serial.setBaudrate(baud_rate);
        
        // 设置超时
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial.setTimeout(timeout);
        
        // 打开串口
        serial.open();
        
        if (!serial.isOpen()) {
            std::cerr << "Failed to open serial port" << std::endl;
            return -1;
        }
        
        std::cout << "Serial port opened successfully!" << std::endl;
        
        // 清空缓冲区
        serial.flushInput();
        serial.flushOutput();
        
        std::cout << "Starting to read data..." << std::endl;
        
        int data_count = 0;
        int total_bytes = 0;
        
        while (running) {
            try {
                if (serial.available()) {
                    size_t available = serial.available();
                    std::vector<uint8_t> buffer(available);
                    size_t bytes_read = serial.read(buffer.data(), available);
                    
                    if (bytes_read > 0) {
                        data_count++;
                        total_bytes += bytes_read;
                        
                        std::cout << "\n=== Data #" << data_count << " ===" << std::endl;
                        std::cout << "Bytes read: " << bytes_read << " (Total: " << total_bytes << ")" << std::endl;
                        
                        // 显示十六进制
                        std::cout << "HEX: ";
                        for (size_t i = 0; i < bytes_read; ++i) {
                            printf("%02X ", buffer[i]);
                        }
                        std::cout << std::endl;
                        
                        // 显示ASCII
                        std::cout << "ASCII: ";
                        for (size_t i = 0; i < bytes_read; ++i) {
                            uint8_t byte = buffer[i];
                            if (byte >= 32 && byte <= 126) {
                                std::cout << static_cast<char>(byte);
                            } else if (byte == 0x0A) {
                                std::cout << "\\n";
                            } else if (byte == 0x0D) {
                                std::cout << "\\r";
                            } else {
                                std::cout << ".";
                            }
                        }
                        std::cout << std::endl;
                    }
                } else {
                    // 短暂等待
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            } catch (const serial::IOException& e) {
                std::cerr << "Serial read error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        serial.close();
        std::cout << "Serial port closed." << std::endl;
        
    } catch (const serial::IOException& e) {
        std::cerr << "Serial error: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "Program finished." << std::endl;
    return 0;
} 