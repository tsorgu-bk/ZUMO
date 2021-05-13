#pragma once
#include "Stream.h"
#include "main.h"

class UART_STM32F4x: public Stream{


public:
    UART_STM32F4x(){}

     int available() override{
         return 0;
    }
     uint8_t read() override{
         return 0;
    }
    int write(uint8_t data) override{
        (void) data;
        return 0;
    }
    int write(uint8_t* data, uint16_t len) override{
        (void) data;
        (void) len;
        return 0;
    }
    void init() override{
     }
};