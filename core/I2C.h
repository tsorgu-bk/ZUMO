//
// Created by Andrzej on 10.05.2021.
//

#pragma once
#include <cstdint>


class I2C{

public:
    virtual void write(uint8_t address, uint8_t register_address) = 0;
    virtual void write(uint8_t address, uint8_t register_address, uint8_t data) = 0;
    virtual void write(uint8_t address, uint8_t register_address, uint16_t data) = 0;
    virtual uint8_t read(uint8_t address, uint8_t register_address) = 0;
    virtual uint8_t* read(uint8_t address, uint8_t register_address, uint32_t size) = 0;
    virtual void read(uint8_t address, uint8_t register_address, uint32_t size, uint8_t* buff) = 0;
};
