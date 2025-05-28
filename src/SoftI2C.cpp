#include "SoftI2C.h"

SoftI2C::SoftI2C(uint8_t sdaPin, uint8_t sclPin) {
    _sdaPin = sdaPin;
    _sclPin = sclPin;
    _delay_us = 5; // ~100kHz clock -> 5us delay per half cycle
}

void SoftI2C::begin() {
    pinMode(_sdaPin, INPUT_PULLUP);
    pinMode(_sclPin, INPUT_PULLUP);
}

void SoftI2C::setClockDelay(uint16_t delay_us) {
    _delay_us = delay_us;
}

void SoftI2C::sdaHigh() {
    pinMode(_sdaPin, INPUT_PULLUP); // input pull-up = line high
}

void SoftI2C::sdaLow() {
    pinMode(_sdaPin, OUTPUT);
    digitalWrite(_sdaPin, LOW);
}

void SoftI2C::sclHigh() {
    pinMode(_sclPin, INPUT_PULLUP);
    // Attendi che la linea sia effettivamente alta (clock stretching)
    unsigned long start = micros();
    while (digitalRead(_sclPin) == LOW) {
        if (micros() - start > 1000) break; // timeout 1ms
    }
}

void SoftI2C::sclLow() {
    pinMode(_sclPin, OUTPUT);
    digitalWrite(_sclPin, LOW);
}

bool SoftI2C::sdaRead() {
    pinMode(_sdaPin, INPUT_PULLUP);
    return digitalRead(_sdaPin);
}

void SoftI2C::i2cDelay() {
    delayMicroseconds(_delay_us);
}

void SoftI2C::start() {
    sdaHigh();
    sclHigh();
    i2cDelay();
    sdaLow();
    i2cDelay();
    sclLow();
    i2cDelay();
}

void SoftI2C::stop() {
    sdaLow();
    sclHigh();
    i2cDelay();
    sdaHigh();
    i2cDelay();
}

bool SoftI2C::writeByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) sdaHigh();
        else sdaLow();
        i2cDelay();
        sclHigh();
        i2cDelay();
        sclLow();
        i2cDelay();
        data <<= 1;
    }
    // Lettura ACK
    sdaHigh();  // rilascio SDA
    i2cDelay();
    sclHigh();
    i2cDelay();
    bool ack = !sdaRead();  // ACK è 0, NACK è 1
    sclLow();
    i2cDelay();
    return ack;
}

uint8_t SoftI2C::readByte(bool ack) {
    uint8_t data = 0;
    sdaHigh();  // rilascio SDA per input
    for (int i = 0; i < 8; i++) {
        data <<= 1;
        sclHigh();
        i2cDelay();
        if (sdaRead()) data |= 1;
        sclLow();
        i2cDelay();
    }
    // Invio ACK/NACK
    if (ack) sdaLow();
    else sdaHigh();
    i2cDelay();
    sclHigh();
    i2cDelay();
    sclLow();
    sdaHigh(); // rilascio SDA
    i2cDelay();
    return data;
}
