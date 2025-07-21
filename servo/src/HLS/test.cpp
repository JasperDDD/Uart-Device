#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <chrono>

#include "feetech/HLS.hpp"
#include "serial/serial.h"

void printMenu() {
    std::cout << "\n=== HLS Servo Control Menu ===" << std::endl;
    std::cout << "1. Calibrate Servos" << std::endl;
    std::cout << "2. Single Run" << std::endl;
    std::cout << "3. Continuous Loop" << std::endl;
    std::cout << "4. Display Position" << std::endl;
    std::cout << "0. Exit" << std::endl;
    std::cout << "Please select: ";
}

void displayPosition(feetech::HLS& hls_1, feetech::HLS& hls_2) {
    std::cout << "\nDisplaying position..." << std::endl;
    std::cout << "Servo 1 current position: " << hls_1.getPos() << std::endl;
    std::cout << "Servo 2 current position: " << hls_2.getPos() << std::endl;
}

void calibrateServos(feetech::HLS& hls_1, feetech::HLS& hls_2) {
    std::cout << "\nStarting servo calibration..." << std::endl;
    
    // Set servos to center position
    std::cout << "Setting servos to center position (2048)..." << std::endl;
    hls_1.setOffset(2048);
    hls_2.setOffset(2048);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Read current positions
    int16_t pos1 = hls_1.getPos();
    int16_t pos2 = hls_2.getPos();
    std::cout << "Servo 1 current position: " << pos1 << std::endl;
    std::cout << "Servo 2 current position: " << pos2 << std::endl;
    
    std::cout << "Calibration completed!" << std::endl;
}

void singleRun(feetech::HLS& hls_1, feetech::HLS& hls_2) {
    std::cout << "\nStarting single run..." << std::endl;
    
    uint32_t start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    // Execute one complete sine wave cycle
    for (int i = 0; i < 100; i++) {
        uint32_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        uint16_t pos1 = 2000 * (sin((current_time - start_time) / 5000.0 * 2 * M_PI)) + 2000;
        uint16_t pos2 = 2000 * (sin((current_time - start_time) / 5000.0 * 2 * M_PI + M_PI)) + 2000;
        
        int16_t pre_pos1 = hls_1.getPos();
        int16_t pre_pos2 = hls_2.getPos();
        
        std::cout << "pre_pos1: " << pre_pos1 << " pre_pos2: " << pre_pos2 << std::endl;
        std::cout << "tar_pos1: " << pos1 << " tar_pos2: " << pos2 << std::endl;
        //std::cout << "pre_pos1: " << pre_pos1 << " tar_pos1: " << pos1 << std::endl;
        
        hls_1.setPos(pos1);
        hls_2.setPos(pos2);
        hls_1.action();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    std::cout << "Single run completed!" << std::endl;
}

void continuousLoop(feetech::HLS& hls_1, feetech::HLS& hls_2) {
    std::cout << "\nStarting continuous loop (press 'q' to quit)..." << std::endl;
    
    uint32_t start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    while (true) {
        // Check for keyboard input
        if (std::cin.peek() == 'q' || std::cin.peek() == 'Q') {
            std::cin.get(); // Clear input buffer
            break;
        }
        
        uint32_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        uint16_t pos1 = 2000 * (sin((current_time - start_time) / 5000.0 * 2 * M_PI)) + 2000;
       // uint16_t pos2 = 2000 * (sin((current_time - start_time) / 5000.0 * 2 * M_PI + M_PI)) + 2000;
        
        int16_t pre_pos1 = hls_1.getPos();
        //int16_t pre_pos2 = hls_2.getPos();
        
        //std::cout << "pre_pos1: " << pre_pos1 << " pre_pos2: " << pre_pos2 << std::endl;
        //std::cout << "tar_pos1: " << pos1 << " tar_pos2: " << pos2 << std::endl;
        std::cout << "pre_pos1: " << pre_pos1 << " tar_pos1: " << pos1 << std::endl;
        
        hls_1.setPos(pos1);
        //hls_2.setPos(pos2);
        hls_1.action();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    std::cout << "Continuous loop stopped!" << std::endl;
}

int main() {
    serial::Serial serial("/dev/ttyACM0", 1000000, serial::Timeout::simpleTimeout(1000));
    feetech::HLS hls_1(2, 4900, 20, 100, 10, &serial);
    //feetech::HLS hls_2(2, 4900, 1650, 2200, 100, &serial);

    // Check servo connections
    if (!hls_1.ping()) {
        std::cout << "Ping " << hls_1.getID() << " failed" << std::endl;
        return 0;
    }
    // if (!hls_2.ping()) {
    //     std::cout << "Ping " << hls_2.getID() << " failed" << std::endl;
    //     return 0;
    // }
    
    std::cout << "Servos connected successfully!" << std::endl;

    uint8_t mode = hls_1.getMode();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //std::cout << "Mode is " << (int)mode << std::endl;
    hls_1.setMode(feetech::HLS::Mode::SPEED);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    mode = hls_1.getMode();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    int choice;
    while (true) {
        //printMenu();
        // int16_t pos1 = hls_1.getPos();
        // std::cout << "Servo 1 current position: " << pos1 << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // hls_1.setMode(feetech::HLS::Mode::POSITION);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        hls_1.setSpeed(50);
        hls_1.action();
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        hls_1.getSpeed();
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
        hls_1.setSpeed(200);
        hls_1.action();
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        hls_1.getSpeed();
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
        //std::cout << "Mode is " << (int)mode << std::endl;
        // switch (choice) {
        //     case 0:
        //         std::cout << "Exiting program..." << std::endl;
        //         serial.close();
        //         return 0;
                
        //     case 1:
        //         calibrateServos(hls_1, hls_2);
        //         break;
                
        //     case 2:
        //         singleRun(hls_1, hls_2);
        //         break;
                
        //     case 3:
        //         continuousLoop(hls_1, hls_2);
        //         break;

        //     case 4:
        //         displayPosition(hls_1, hls_2);
        //         break;
                
        //     default:
        //         std::cout << "Invalid choice, please try again!" << std::endl;
        //         break;
        // }
    }

    serial.close();
    return 0;
}