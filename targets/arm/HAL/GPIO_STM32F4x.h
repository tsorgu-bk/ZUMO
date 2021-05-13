#pragma once
#include "GPIO.h"
#include "main.h"

class GPIO_STM32F4x: public GPIO {
    GPIO_TypeDef* port;
    uint16_t pin;

public:

    GPIO_STM32F4x(GPIO_TypeDef* port, uint16_t pin): port(port), pin(pin){}
    void set() override{
        HAL_GPIO_WritePin(port,pin,GPIO_PIN_SET);
    }
    void reset() override{
        HAL_GPIO_WritePin(port,pin,GPIO_PIN_RESET);
    }
    bool get() override{
        return HAL_GPIO_ReadPin(port,pin) != 0;
    }
    void toggle() override{
        HAL_GPIO_TogglePin(port,pin);
    }
};
