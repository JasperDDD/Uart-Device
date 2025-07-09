#include "feetech/HLS.hpp"
#include <iostream>
#include <thread>
#include <chrono>

#define DEBUG 0

namespace feetech {

HLS::HLS(uint8_t id,int16_t max_pos,int16_t min_pos,int16_t speed,int16_t acc,serial::Serial *serial) : 
Servo(id),serial(serial) 
{
    this->id = id;
    this->max_pos = max_pos;
    this->min_pos = min_pos;
    this->speed = speed;
    this->acc = acc;
    
    try {
        if (serial->isOpen()) {
            std::cout << "Serial port opened successfully" << std::endl;
        } else {
            std::cout << "Failed to open serial port" << std::endl;
        }
    } catch (serial::IOException &e) {
        std::cerr << "Unable to open port: " << e.what() << std::endl;
    }
}

HLS::~HLS() {}

std::vector<uint8_t> HLS::serialize(Packet packet) const {
    std::vector<uint8_t> data_;
    data_.push_back(packet.header1);
    data_.push_back(packet.header2);
    data_.push_back(packet.id);
    data_.push_back(packet.length);
    data_.push_back(packet.command);
    data_.insert(data_.end(), packet.data.begin(), packet.data.end());
    data_.push_back(packet.checksum);
    return data_;
}

Packet HLS::deserialize(std::vector<uint8_t> data_) const {
    Packet packet_;
    packet_.header1 = data_[0];
    packet_.header2 = data_[1];
    packet_.id = data_[2];
    packet_.length = data_[3];
    packet_.command = data_[4];
    if (packet_.command != 0) {
        std::cout << "Error is " << (int)packet_.command << std::endl;
        return packet_;
    }
    packet_.data = std::vector<uint8_t>(data_.begin() + 5, data_.end() - 1);
    packet_.checksum = data_.back();
    if(packet_.checksum != data_.back()) {
        std::cout << "Checksum is not correct" << std::endl;
        return packet_;
    }

    return packet_;
}

uint8_t HLS::getCheckSum(Packet packet) const {
    uint8_t checksum_ = 0;
    checksum_ += packet.id;
    checksum_ += packet.length;
    checksum_ += packet.command;
    if (packet.data.size() > 0) {
        for (int i = 0; i < packet.data.size(); i++) {
            checksum_ += packet.data[i];
        }
    }
    checksum_ = ~checksum_ & 0xFF;
    packet.checksum = checksum_;
    if(DEBUG) {
        std::cout << "Checksum is " << (int)packet.checksum << std::endl;
    }
    return checksum_;
}


void HLS::write(uint8_t id,uint8_t length,std::vector<uint8_t> data_) {
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.length = length + 2;
    packet_.command = static_cast<uint8_t>(feetech::Command::REGWRITE_DATA);
    packet_.data = data_;
    packet_.checksum = getCheckSum(packet_);

    if(DEBUG) {
    std::cout << "Sending data: ";
    for(auto byte : serialize(packet_)) {
        printf("%02X ", byte);
    }
    std::cout << std::endl;
    }

    if(serial->isOpen()) {
        serial->write(serialize(packet_));
    }
}

bool HLS::read() {
    size_t timeout_ms = 50;
    size_t elapsed = 0;
    std::vector<uint8_t> data_;

    while (serial->available() < 1 && elapsed < timeout_ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        elapsed++;
    }

    if (serial->available() > 0) {
        std::string data = serial->read(serial->available());
        data_.assign(data.begin(), data.end());
        serial->flush();
        if(DEBUG) {
            std::cout << "Received data: ";
            for(auto byte : data_) {
                printf("%02X ", byte);
            }
            std::cout << std::endl;
        }
    }
    if (elapsed >= timeout_ms) {
        std::cout << "Timeout waiting for data" << std::endl;
        return false;
    }
    receive_packet = deserialize(data_);
    if (receive_packet.header1 != PROTOCOL_HEADER1 || receive_packet.header2 != PROTOCOL_HEADER2) {
        std::cout << "Error is " << (int)receive_packet.command << std::endl;
        return false;
    }
    return true;
}

void HLS::setId(uint8_t id_) {
    // 掉电保存
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.length = 0x04;
    packet_.command = static_cast<uint8_t>(feetech::Command::WRITE);
    packet_.data.push_back(0x37);
    packet_.data.push_back(0x00);
    packet_.checksum = getCheckSum(packet_);
    
    std::vector<uint8_t> data = serialize(packet_);

    if(serial->isOpen()) {
        serial->write(data);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    packet_.data.clear();

    packet_.data.push_back(0x05);
    packet_.data.push_back(id_);
    packet_.checksum = getCheckSum(packet_);
    data = serialize(packet_);

    if(serial->isOpen()) {
        serial->write(data);
    }

}

bool HLS::ping() {
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.length = 0x02;
    packet_.command = static_cast<uint8_t>(feetech::Command::PING);
    packet_.checksum = getCheckSum(packet_);
    
    std::cout << "Sending ping to servo ID: " << (int)id << std::endl;
    
    if(serial->isOpen()) {
        std::vector<uint8_t> data = serialize(packet_);
        if(DEBUG) {
            std::cout << "Sending data: ";
            for(auto byte : data) {
                printf("%02X ", byte);
            }
            std::cout << std::endl;
        }
        serial->write(data);
    } else {
        std::cout << "Serial port is not open!" << std::endl;
        return false;
    }

    if (!read()) {
        std::cout << "Ping failed - no response from servo" << std::endl;
        return false;
    }
    
    std::cout << "Ping successful!" << std::endl;
    return true;
}

void HLS::setPos(int16_t pos) {
    if (pos > max_pos) {
        pos = max_pos;
    }
    if (pos < min_pos) {
        pos = min_pos;
    }

    // pos = scs_tohost(pos,15);

    this->pos = pos;

    Packet packet_;
    std::vector<uint8_t> data_;
    data_.push_back(0x29);
    data_.push_back(this->acc);
    data_.push_back(this->pos & 0xFF);
    data_.push_back(this->pos >> 8 & 0xFF);
    data_.push_back(this->torque & 0xFF);
    data_.push_back(this->torque >> 8 & 0xFF);
    data_.push_back(this->speed & 0xFF);
    data_.push_back(this->speed >> 8 & 0xFF);
    if(serial->isOpen()) {
        write(id,data_.size(),data_);
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if(!read()) {
        std::cout << "Write failed" << std::endl;
        return;
    }

    if(DEBUG) {
        std::cout << "Write successful" << std::endl;
    }

}

void HLS::setOffset(uint16_t offset) {
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.length = 4;
    packet_.command = static_cast<uint8_t>(feetech::Command::RESET);
    packet_.data.push_back(offset & 0xFF);
    packet_.data.push_back(offset >> 8);
    packet_.checksum = getCheckSum(packet_);
    if(serial->isOpen()) {
        serial->write(serialize(packet_));
    }
}

int16_t HLS::getPos() {
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.length = 0x04;
    packet_.command = static_cast<uint8_t>(feetech::Command::READ);
    packet_.data.push_back(0x38);
    packet_.data.push_back(0x02);
    packet_.checksum = getCheckSum(packet_);
    
    if(serial->isOpen()) {
        serial->write(serialize(packet_));
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if (!read()) {
        std::cout << "Failed to read position" << std::endl;
        return 0;
    }
    int16_t pos = receive_packet.data[0] | (receive_packet.data[1] << 8);
    receive_packet.data.clear();
    // pos = scs_tohost(pos,15);
    return pos;
}

void HLS::action() {
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = 0xFE;
    packet_.length = 0x02;
    packet_.command = static_cast<uint8_t>(feetech::Command::ACTION);
    packet_.checksum = getCheckSum(packet_);

    if(DEBUG) {
        std::cout << "Sending data: ";
        for(auto byte : serialize(packet_)) {
            printf("%02X ", byte);
        }
    std::cout << std::endl;
    }
    if(serial->isOpen()) {
        serial->write(serialize(packet_));
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

}