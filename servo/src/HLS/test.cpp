#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <chrono>

#include "feetech/HLS.hpp"
#include "serial/serial.h"

int main() {
    serial::Serial serial("/dev/ttyCH343USB0", 1000000, serial::Timeout::simpleTimeout(1000));
    feetech::HLS hls_1(1, 4900, 20, 1000, 100, &serial);
    feetech::HLS hls_2(2, 4900, 20, 1000, 100, &serial);

    if (!hls_1.ping()) {
        std::cout << "Ping " << hls_1.getID() << " failed" << std::endl;
        return 0;
    }
    if (!hls_2.ping()) {
        std::cout << "Ping " << hls_2.getID() << " failed" << std::endl;
        return 0;
    }

    uint32_t start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    // while(1) {
    //     uint32_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    //     uint16_t pos1 = 2000*(sin((current_time - start_time)/5000.0*2*M_PI))+ 2000;
    //     uint16_t pos2 = 2000*(sin((current_time - start_time)/5000.0*2*M_PI+ M_PI))+ 2000;
    //     uint16_t pre_pos1 = hls_1.getPos();
    //     uint16_t pre_pos2 = hls_2.getPos();
    //     std::cout << "pre_pos1: " << (int)pre_pos1 << " pre_pos2: " << (int)pre_pos2 << std::endl;
    //     std::cout << "tar_pos1: " << (int)pos1 << " tar_pos2: " << (int)pos2 << std::endl;
    //     hls_1.setPos(pos1);
    //     hls_2.setPos(pos2);
    //     hls_2.action();
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }

    hls_1.setId(0x01);

    serial.close();
    return 0;
}