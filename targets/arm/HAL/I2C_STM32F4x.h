//
// Created by Andrzej on 10.05.2021.
//

#pragma once
#include "I2C.h"
#include "main.h"


class I2C_STM32F4x: public I2C{
    I2C_HandleTypeDef& hi2c; //http://www.disca.upv.es/aperles/arm_cortex_m3/llibre/st/STM32F439xx_User_Manual/structi2c__handletypedef.html
    uint8_t data_buffer[10]={0};

public:
    I2C_STM32F4x(I2C_HandleTypeDef& hi2c): hi2c(hi2c){}
    void write(uint8_t address, uint8_t register_address) override{
        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*) &hi2c, address, (uint8_t*) &register_address,1,10);
    }
    void write(uint8_t address, uint8_t register_address, uint8_t data) override {
        uint8_t adr_buff[2] = {uint8_t (register_address), data};
        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*) &hi2c, address,adr_buff,2,10);
    }
    void write(uint8_t address, uint8_t register_address, uint16_t data) override {
        uint8_t adr_buff[3] = {uint8_t (register_address), uint8_t(data >>8), uint8_t (data)};
        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*) &hi2c, address, (uint8_t*) &adr_buff,3,10);
    }
    uint8_t read(uint8_t address, uint8_t register_address) override {
        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*) &hi2c, address, (uint8_t*) &register_address, 1,10);
        HAL_I2C_Master_Receive((I2C_HandleTypeDef*) &hi2c, address,data_buffer,1,10);
        return data_buffer[0];
    }
    uint8_t* read(uint8_t address, uint8_t register_address, uint32_t size) override {
        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*) &hi2c, address, (uint8_t*) &register_address, 1,10);
        HAL_I2C_Master_Receive((I2C_HandleTypeDef*) &hi2c, address,data_buffer,size,10);
        return data_buffer;
    }
    virtual void read(uint8_t address, uint8_t register_address, uint32_t size, uint8_t* data) override {
        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*) &hi2c, address, (uint8_t*) &register_address, 1,10);
        HAL_I2C_Master_Receive((I2C_HandleTypeDef*) &hi2c, address,data,size,10);
    }
};
