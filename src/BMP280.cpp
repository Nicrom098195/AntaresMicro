#include "BMP280.h"

// Register BMP280
#define BMP280_REG_CALIB_00 0x88
#define BMP280_REG_ID       0xD0
#define BMP280_REG_RESET    0xE0
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG   0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_TEMP_MSB  0xFA

BMP280::BMP280(SoftI2C &i2c, uint8_t address) : _i2c(i2c), _address(address) {
}

bool BMP280::begin() {
    // Controllo ID sensore
    if (read8(BMP280_REG_ID) != 0x58) return false;

    // Reset sensore
    write8(BMP280_REG_RESET, 0xB6);
    delay(100);

    if (!readCoefficients()) return false;

    // Configurazione sensore (normal mode, oversampling x1)
    write8(BMP280_REG_CTRL_MEAS, 0x27);  // temp and pressure oversampling x1, normal mode
    write8(BMP280_REG_CONFIG, 0x00);     // t_standby = 0.5 ms, filter off

    return true;
}

bool BMP280::readCoefficients() {
    dig_T1 = read16(BMP280_REG_CALIB_00);
    dig_T2 = readS16(BMP280_REG_CALIB_00 + 2);
    dig_T3 = readS16(BMP280_REG_CALIB_00 + 4);

    dig_P1 = read16(BMP280_REG_CALIB_00 + 6);
    dig_P2 = readS16(BMP280_REG_CALIB_00 + 8);
    dig_P3 = readS16(BMP280_REG_CALIB_00 + 10);
    dig_P4 = readS16(BMP280_REG_CALIB_00 + 12);
    dig_P5 = readS16(BMP280_REG_CALIB_00 + 14);
    dig_P6 = readS16(BMP280_REG_CALIB_00 + 16);
    dig_P7 = readS16(BMP280_REG_CALIB_00 + 18);
    dig_P8 = readS16(BMP280_REG_CALIB_00 + 20);
    dig_P9 = readS16(BMP280_REG_CALIB_00 + 22);

    return true;
}

uint8_t BMP280::read8(uint8_t reg) {
    _i2c.start();
    if (!_i2c.writeByte((_address << 1) | 0)) { _i2c.stop(); return 0; }
    if (!_i2c.writeByte(reg)) { _i2c.stop(); return 0; }
    _i2c.start();
    if (!_i2c.writeByte((_address << 1) | 1)) { _i2c.stop(); return 0; }
    uint8_t val = _i2c.readByte(false);
    _i2c.stop();
    return val;
}

uint16_t BMP280::read16(uint8_t reg) {
    uint8_t lsb, msb;
    _i2c.start();
    if (!_i2c.writeByte((_address << 1) | 0)) { _i2c.stop(); return 0; }
    if (!_i2c.writeByte(reg)) { _i2c.stop(); return 0; }
    _i2c.start();
    if (!_i2c.writeByte((_address << 1) | 1)) { _i2c.stop(); return 0; }
    lsb = _i2c.readByte(true);
    msb = _i2c.readByte(false);
    _i2c.stop();
    return (msb << 8) | lsb;
}


int16_t BMP280::readS16(uint8_t reg) {
    return (int16_t)read16(reg);
}

uint32_t BMP280::read24(uint8_t reg) {
    uint32_t msb, lsb, xlsb;
    _i2c.start();
    if (!_i2c.writeByte((_address << 1) | 0)) { _i2c.stop(); return 0; }
    if (!_i2c.writeByte(reg)) { _i2c.stop(); return 0; }
    _i2c.start();
    if (!_i2c.writeByte((_address << 1) | 1)) { _i2c.stop(); return 0; }
    msb = _i2c.readByte(true);
    lsb = _i2c.readByte(true);
    xlsb = _i2c.readByte(false);
    _i2c.stop();
    return (msb << 16) | (lsb << 8) | xlsb;
}

void BMP280::write8(uint8_t reg, uint8_t value) {
    _i2c.start();
    _i2c.writeByte((_address << 1) | 0);
    _i2c.writeByte(reg);
    _i2c.writeByte(value);
    _i2c.stop();
}

float BMP280::readTemperature() {
    int32_t adc_T = read24(BMP280_REG_TEMP_MSB);
    adc_T >>= 4;

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 <<1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    float T = (t_fine * 5 + 128) >> 8;
    return T / 100.0;
}

float BMP280::readPressure() {
    int32_t adc_P = read24(BMP280_REG_PRESS_MSB);
    adc_P >>= 4;

    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) return 0; // avoid division by zero

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (float)p / 256.0f / 100.0f;  // hPa
}
