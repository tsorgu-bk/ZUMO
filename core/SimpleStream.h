#pragma once

#include <cstdint>

class SimpleStream {
public:
    virtual uint16_t available() = 0;
    virtual void write(uint8_t* data, uint16_t length) = 0;
    virtual uint8_t read() = 0;
    virtual void flush() = 0;
    virtual void init() { }
//    virtual uint8_t read() = 0;

};