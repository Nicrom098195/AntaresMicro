#ifndef BMP280_h
#define BMP280_h

#include <Arduino.h>
#include "SoftI2C.h"

class BMP280 {
public:
    BMP280(SoftI2C &i2c, uint8_t address = 0x76);

    bool begin();
    float readTemperature();
    float readPressure();

private:
    SoftI2C &_i2c;
    uint8_t _address;

    // Calibrazione
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    int32_t t_fine;

    bool readCoefficients();
    uint8_t read8(uint8_t reg);
    uint16_t read16(uint8_t reg);
    int16_t readS16(uint8_t reg);
    uint32_t read24(uint8_t reg);
    void write8(uint8_t reg, uint8_t value);

    int32_t compensateTemperature(int32_t adc_T);
    uint32_t compensatePressure(int32_t adc_P);
};

#endif
