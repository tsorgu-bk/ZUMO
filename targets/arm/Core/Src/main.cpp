//
// Created by Andrzej on 10.05.2021.
//
#include "main.h"
#include "GPIO_STM32F4x.h"
//#include "UART_STM32F4x.h"
#include "I2C_STM32F4x.h"
#include "BME280.h"
#include "stdio.h"



GPIO_STM32F4x LED1(LED1_GPIO_Port,LED1_Pin);
GPIO_STM32F4x LED2(LED2_GPIO_Port,LED2_Pin);


extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
I2C_STM32F4x i2c(hi2c1);
BME280 bme280( i2c, 0b1110110);

extern UART_HandleTypeDef huart1;

uint8_t data_buffer[40]= {0};

//UART_STM32F4x usart;
extern "C"
int Main(){
    // usart.init();
    if(bme280.init()){
        auto length = sprintf(reinterpret_cast<char *>(data_buffer), "bme280 OK \n");
        HAL_UART_Transmit(&huart1, data_buffer, length,10);
    } else{
        auto length = sprintf(reinterpret_cast<char *>(data_buffer), "bme280 NOT OK \n");
        HAL_UART_Transmit(&huart1, data_buffer, length,10);
    }
    bme280.set_enable(true);
    while(true){
        LED1.toggle();
        HAL_Delay(100);
        LED2.toggle();
        HAL_Delay(100);
        bme280.run_measurements();
        auto length = sprintf(reinterpret_cast<char *>(data_buffer), "Temperature: %f , Pressure: %lu \n ", bme280.get_last_temperature(), bme280.get_last_pressure());
        HAL_UART_Transmit(&huart1, data_buffer, length,10);
     }
    return 0;
}