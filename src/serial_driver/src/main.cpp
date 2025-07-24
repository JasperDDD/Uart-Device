#include "serial_driver/serial_driver.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>
#include <atomic>
#include <csignal>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

std::atomic<bool> running(true);

void signalHandler(int sig) {
    running = false;
    std::cout << "\nReceived signal " << sig << ", shutting down..." << std::endl;
    ros::shutdown();
}

// 自定义数据编码函数
std::vector<uint8_t> encodeMoveCommand(int x, int y, int z) {
    std::vector<uint8_t> data;
    data.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>(x & 0xFF));
    data.push_back(static_cast<uint8_t>((y >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>(y & 0xFF));
    data.push_back(static_cast<uint8_t>((z >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>(z & 0xFF));
    return data;
}

// ROS话题回调函数
class SerialNode {
private:
    serial_driver::SerialDriver driver_;
    ros::NodeHandle nh_;
    ros::Subscriber twist_sub_;

public:
    SerialNode() {
        // 订阅Twist消息（包含x, y, z）
        twist_sub_ = nh_.subscribe("cmd_vel", 10, &SerialNode::twistCallback, this);
        
        std::cout << "Subscribed to topics:" << std::endl;
        std::cout << "  - /cmd_vel (geometry_msgs/Twist)" << std::endl;
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
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 将浮点数转换为整数（乘以1000以保留精度）
        int x = static_cast<int>(msg->linear.x * 1000);
        int y = static_cast<int>(msg->linear.y * 1000);
        int z = static_cast<int>(msg->linear.z * 1000);
        
        std::cout << "Received Twist: x=" << msg->linear.x 
                  << ", y=" << msg->linear.y 
                  << ", z=" << msg->linear.z << std::endl;
        
        // 编码并发送移动命令
        std::vector<uint8_t> data = encodeMoveCommand(x, y, z);
        
        if (driver_.sendData(serial_driver::SerialFrame::COMPONENT_CHASSIS, data)) {
            std::cout << "Move command sent successfully" << std::endl;
        } else {
            std::cout << "Failed to send move command" << std::endl;
        }
    }
};

int main(int argc, char** argv) {
    // 初始化ROS
    ros::init(argc, argv, "serial_test", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // 设置信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 获取参数
    std::string port = "/dev/ttyUSB0";
    uint32_t baud_rate = 115200;
    
    // 尝试从ROS参数获取
    private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
    int baud_rate_int = 115200;
    private_nh.param<int>("baud_rate", baud_rate_int, 115200);
    baud_rate = static_cast<uint32_t>(baud_rate_int);
    
    // 如果命令行参数存在，则覆盖ROS参数
    if (argc >= 2) {
        port = argv[1];
    }
    if (argc >= 3) {
        try {
            baud_rate = std::stoi(argv[2]);
        } catch (const std::exception& e) {
            std::cerr << "Error parsing baud rate: " << e.what() << std::endl;
            std::cerr << "Using default baud rate: " << baud_rate << std::endl;
        }
    }
    
    std::cout << "Serial Driver Test Program" << std::endl;
    std::cout << "Port: " << port << std::endl;
    std::cout << "Baud rate: " << baud_rate << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl;
    
    // 创建串口节点
    SerialNode serial_node;
    
    // 初始化串口
    if (!serial_node.initialize(port, baud_rate)) {
        std::cerr << "Failed to initialize serial port" << std::endl;
        std::cerr << "Please check if the port exists and you have permission to access it." << std::endl;
        std::cerr << "You can try: sudo chmod 666 " << port << std::endl;
        return -1;
    }
    
    std::cout << "Serial port initialized successfully!" << std::endl;
    
    // 启动串口通信
    serial_node.start();
    
    // 等待一下确保串口启动
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "Serial driver started. Waiting for ROS messages..." << std::endl;
    std::cout << "Send messages to /cmd_vel (geometry_msgs/Twist) and /speed (std_msgs/Float32)" << std::endl;
    
    // ROS主循环
    ros::Rate rate(50); // 10Hz
    while (running && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    // 停止串口通信
    serial_node.stop();
    
    std::cout << "Test program finished." << std::endl;
    return 0;
} 