//
// Created by Andrzej on 10.05.2021.
//
#include "main.h"
#include "GPIO_STM32F4x.h"
//#include "UART_STM32F4x.h"
#include "I2C_STM32F4x.h"
#include "BME280.h"
#include "bhy.h"
#include "stdio.h"
#include <ICM20948.h>

GPIO_STM32F4x LED1(LED1_GPIO_Port,LED1_Pin);
GPIO_STM32F4x LED2(LED2_GPIO_Port,LED2_Pin);


extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
I2C_STM32F4x i2c(hi2c1);
BME280 bme280( i2c, 0b1110110);
BHYSensor bhi160(i2c);
ICM20948 myIMU(i2c);
extern UART_HandleTypeDef huart1;

uint8_t data_buffer[40]= {0};
//UART_STM32F4x usart;
extern "C"
//


// test


// test
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

    // icm20948
    // Reset ICM20948
    myIMU.writeByte(ICM20948_ADDRESS, PWR_MGMT_1, READ_FLAG);
    HAL_Delay(100);
    myIMU.writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
    HAL_Delay(100);


    // Read the WHO_AM_I register, this is a good test of communication
    auto ICM = myIMU.readByte(ICM20948_ADDRESS, WHO_AM_I_ICM20948);
    auto length2 = sprintf(reinterpret_cast<char *>(data_buffer), "ICM id : %02X \n ", ICM);
    HAL_UART_Transmit(&huart1, data_buffer, length2,10);
    //auto some_buff = i2c.read((uint8_t )(((uint8_t)ICM20948_ADDRESS<<1) | (uint8_t )1), WHO_AM_I_ICM20948);


    // Start by performing self test and reporting values
    myIMU.ICM20948SelfTest(myIMU.selfTest);
    auto print = sprintf(reinterpret_cast<char *>(data_buffer), "x-axis self test: acc trim within : ");
    HAL_UART_Transmit(&huart1, data_buffer, print,10);
    print =sprintf(reinterpret_cast<char *>(data_buffer), "%02f of factory value\n",(myIMU.selfTest[0]));
    HAL_UART_Transmit(&huart1, data_buffer, print,10);
    print = sprintf(reinterpret_cast<char *>(data_buffer), "y-axis self test: acc trim within : ");
    HAL_UART_Transmit(&huart1, data_buffer, print,10);
    print = sprintf(reinterpret_cast<char *>(data_buffer), "%02f of factory value\n",(myIMU.selfTest[1]));
    HAL_UART_Transmit(&huart1, data_buffer, print,10);
    print =  sprintf(reinterpret_cast<char *>(data_buffer), "z-axis self test: acc trim within : ");
    HAL_UART_Transmit(&huart1, data_buffer, print,10);
    print = sprintf(reinterpret_cast<char *>(data_buffer), "%02f of factory value\n",(myIMU.selfTest[2]));
    HAL_UART_Transmit(&huart1, data_buffer, print,10);

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateICM20948(myIMU.gyroBias, myIMU.accelBias);
    myIMU.initICM20948();


    // end icm20948
    // ak09916

    auto AK009916 = myIMU.readByte(AK09916_ADDRESS, WHO_AM_I_AK09916);
    auto lengthAK = sprintf(reinterpret_cast<char *>(data_buffer), "AK id : %02X \n ", AK009916);
    HAL_UART_Transmit(&huart1, data_buffer, lengthAK,10);


    //end ak09916
    while(true){
        LED1.toggle();
        HAL_Delay(100);
        LED2.toggle();
        HAL_Delay(100);
        bme280.run_measurements();
        auto length = sprintf(reinterpret_cast<char *>(data_buffer), "Temperature: %.2f , Pressure: %lu \n ", bme280.get_last_temperature(), bme280.get_last_pressure());
        HAL_UART_Transmit(&huart1, data_buffer, length,10);

        //icm measurments


        myIMU.readAccelData(myIMU.accelCount);
        myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
        myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
        myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];
        auto lengthAccel = sprintf(reinterpret_cast<char *>(data_buffer), "acceleration values: \n");
        HAL_UART_Transmit(&huart1, data_buffer, lengthAccel,10);
        lengthAccel = sprintf(reinterpret_cast<char *>(data_buffer), "%f\n", myIMU.ax);
        HAL_UART_Transmit(&huart1, data_buffer, lengthAccel,10);
        lengthAccel = sprintf(reinterpret_cast<char *>(data_buffer), "%f\n", myIMU.ay);
        HAL_UART_Transmit(&huart1, data_buffer, lengthAccel,10);
        lengthAccel = sprintf(reinterpret_cast<char *>(data_buffer), "%f\n", myIMU.az);
        HAL_UART_Transmit(&huart1, data_buffer, lengthAccel,10);



        myIMU.tempCount = myIMU.readTempData();
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        print = sprintf(reinterpret_cast<char *>(data_buffer), "Temp: %.2f\n",myIMU.temperature);
        HAL_UART_Transmit(&huart1, data_buffer, print,10);

        //end icm meas


        // for BHI 160 - nvm
        /*if (bhi160.status == BHY_OK)
            return true;

        if (bhi160.status < BHY_OK) All error codes are negative
        {
            Serial.println("Error code: (" + String(bhi160.status) + "). " + bhi160.getErrorString(bhi160.status));

            return false;  Something has gone wrong
        }
        else All warning codes are positive
        {
            Serial.println("Warning code: (" + String(bhi160.status) + ").");

            return true;
        }*/
    }
    return 0;
}