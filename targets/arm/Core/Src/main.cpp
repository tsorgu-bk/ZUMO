//
// Created by Andrzej on 10.05.2021.
//
#include "main.h"
#include "GPIO_STM32F4x.h"
//#include "UART_STM32F4x.h"
#include "I2C_STM32F4x.h"
#include "BME280.h"
#include "stdio.h"
#include <inttypes.h>
#include "command_manager.h"
#include "Command.h"
#include "UART_STM32F4x.h"

GPIO_STM32F4x LED1(LED1_GPIO_Port,LED1_Pin);
GPIO_STM32F4x LED2(LED2_GPIO_Port,LED2_Pin);

void enable_interrupts() { __enable_irq(); }
void disable_interrupts() { __disable_irq(); }

UART_STM32F4x commandUart={USART1,115200};


CommandManager<10> commandManager = {commandUart, enable_interrupts, disable_interrupts};

void idn_handler() {
    commandManager.print("ZUMO V3");
}

void test_handler() {
    commandManager.print("TEST <3 ");
}

Command_Void idn_command = {"*IDN?", idn_handler, true};
Command_Void test_command = {"test", test_handler, true};

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
I2C_STM32F4x i2c(hi2c1);
BME280 bme280( i2c, 0b1110110);
//extern UART_HandleTypeDef huart1;

uint8_t data_buffer[100]= {0};

//UART_STM32F4x usart;
extern "C"
int Main(){
    commandUart.init();
    commandManager.init();
    commandManager.addCommand(reinterpret_cast<Command *>(&idn_command));
    commandManager.addCommand(reinterpret_cast<Command *>(&test_command));

    if(bme280.init()){
        auto length = sprintf(reinterpret_cast<char *>(data_buffer), "bme280 OK \n");
        commandManager.print(reinterpret_cast<const char *>(data_buffer));
    } else{
        auto length = sprintf(reinterpret_cast<char *>(data_buffer), "bme280 NOT OK \n");
        commandManager.print(reinterpret_cast<const char *>(data_buffer));
    }
    bme280.set_enable(true);
    while(true){
        commandManager.run();
        LED1.toggle();
        HAL_Delay(100);
        LED2.toggle();
        bme280.run_measurements();
        auto length = sprintf(reinterpret_cast<char *>(data_buffer),"%" PRIu16 " %lu %f \n ", bme280.get_last_temperature_multiplied(), bme280.get_last_pressure(), bme280.read_humidity());
        commandManager.print(reinterpret_cast<const char *>(data_buffer));
        HAL_Delay(1000);
     }
    return 0;
}