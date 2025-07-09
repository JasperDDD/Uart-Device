#include "feetech.hpp"
#include  <serial/serial.h>


namespace feetech {

class HLS : public Servo {
public:
    serial::Serial *serial;

    HLS(uint8_t id,int16_t max_pos,int16_t min_pos,int16_t speed,int16_t acc,serial::Serial *serial);
    ~HLS();

    void write(uint8_t id,uint8_t length,std::vector<uint8_t> data);
    bool read();

    bool ping() override;
    void setPos(int16_t pos) override;
    void setOffset(uint16_t offset) override;
    int16_t getPos() override;
    uint8_t getCheckSum(Packet packet) const override;
    void setId(uint8_t id) override;
    void action();

    std::vector<uint8_t> serialize(Packet packet) const;
    Packet deserialize(std::vector<uint8_t> data) const;
    uint8_t getID() {return id;};

private:
    int16_t scs_tohost(int16_t pos,int n)
    {
        if(pos & (1 << n))
            return (pos & ~(1 << n)) * -1;
        else
            return pos;
    }
    mutable Packet receive_packet;
    int16_t torque = 200;
};
}