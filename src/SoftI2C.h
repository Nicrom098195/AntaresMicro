#ifndef SoftI2C_h
#define SoftI2C_h

#include <Arduino.h>

class SoftI2C {
public:
    SoftI2C(uint8_t sdaPin, uint8_t sclPin);

    void begin();
    void setClockDelay(uint16_t delay_us);

    void start();
    void stop();

    bool writeByte(uint8_t data);
    uint8_t readByte(bool ack);

private:
    uint8_t _sdaPin;
    uint8_t _sclPin;
    uint16_t _delay_us;

    void sdaHigh();
    void sdaLow();
    void sclHigh();
    void sclLow();
    bool sdaRead();

    void i2cDelay();
};

#endif
