#include "feetech.hpp"
#include <string>
#if ROS_VERSION == 1
    #include  <serial/serial.h>
#elif ROS_VERSION == 2
    #include "serial_driver/serial_driver.hpp"
#endif

namespace feetech {

class HLS : public Servo {
public:
    enum class Mode {
        POSITION = 0x00,
        SPEED = 0x01,
        CURRENT = 0x02,
        PWM = 0x03,
    };

#if ROS_VERSION == 1
    serial::Serial *serial;
    HLS(uint8_t id,int16_t max_pos,int16_t min_pos,int16_t speed,int16_t acc,serial::Serial *serial);
#elif ROS_VERSION == 2
    drivers::serial_driver::SerialDriver *serial;
    HLS(uint8_t id,int16_t max_pos,int16_t min_pos,int16_t speed,int16_t acc,int16_t torque, drivers::serial_driver::SerialDriver *serial);
#endif
    ~HLS();

    void write(uint8_t id,std::vector<uint8_t> data);
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

    void setMode(bool isStore, Mode mode);
    uint8_t getMode();
    void setSpeed(int16_t speed);
    int16_t getSpeed();
    void disable();

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
    int16_t max_speed = 150;
    int16_t min_speed = -150;

    std::string servo_name;
};
}