#pragma once

#include <cstdint>
//#include "commands_interface.h"
#include "I2C.h"

class BME280 {
    I2C& i2c;
    uint8_t address;

    enum class CalibrationRegister : uint8_t {
        T1_LSB = 0x88, T1_MSB = 0x89, T2_LSB = 0x8A, T2_MSB = 0x8B, T3_LSB = 0x8C, T3_MSB = 0x8D,

        P1_LSB = 0x8E, P1_MSB = 0x8F, P2_LSB = 0x90, P2_MSB = 0x91, P3_LSB = 0x92, P3_MSB = 0x93, P4_LSB = 0x94, P4_MSB = 0x95, P5_LSB = 0x96,
        P5_MSB = 0x97, P6_LSB = 0x98, P6_MSB = 0x99, P7_LSB = 0x9A, P7_MSB = 0x9B, P8_LSB = 0x9C, P8_MSB = 0x9D, P9_LSB = 0x9E, P9_MSB = 0x9F,

        H1     = 0xA1, H2_LSB = 0xE1, H2_MSB = 0xE2, H3     = 0xE3,
        H4_MSB = 0xE4, H4_LSB = 0xE5, H5_MSB = 0xE6, H6     = 0xE7
    };

    struct SensorCalibration {
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;

        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;

        uint8_t dig_H1;
        int16_t dig_H2;
        uint8_t dig_H3;
        int16_t dig_H4;
        int16_t dig_H5;
        int8_t dig_H6;
    };

    SensorCalibration calibration;
    float temperature_correction = -5.67;
    uint32_t  t_fine;
    float temperature;
    float pressure;
    float humidity;
    bool enable = false;

public:
    enum class RegisterNames : uint8_t {
        hum_lsb = 0xFE,
        hum_msb = 0xFD,
        temp_xlsb = 0xFC,
        temp_lsb = 0xFB,
        temp_msb = 0xFA,
        press_xlsb = 0xF9,
        press_lsb = 0xF8,
        press_msb = 0xF7,
        config = 0xF5,
        ctrl_meas = 0xF4,
        status = 0xF3,
        reset = 0xE0,
        id = 0xD0
    };

    enum class Oversampling : uint8_t {
        X0 = 0b000,
        X1 = 0b001,
        X2 = 0b010,
        X4 = 0b011,
        X8 = 0b100,
        X16 = 0b101,
    };

    enum class Mode : uint8_t {
        Sleep = 0b00,
        Forced_1 = 0b01,
        Forced_2 = 0b10,
        Normal = 11
    };

    BME280(I2C& i2c, uint8_t address) : i2c(i2c), address(address << 1u) {

    }

    bool init() {
        uint8_t id = i2c.read(address, uint8_t(RegisterNames::id));

        if (id != 0x60) {
            return false;
        }

        //Reading all compensation data, range 0x88:A1, 0xE1:E7
        calibration.dig_T1 = ((uint16_t)((readRegister(CalibrationRegister::T1_MSB) << 8) + readRegister(CalibrationRegister::T1_LSB)));
        calibration.dig_T2 = ((int16_t)((readRegister(CalibrationRegister::T2_MSB) << 8) + readRegister(CalibrationRegister::T2_LSB)));
        calibration.dig_T3 = ((int16_t)((readRegister(CalibrationRegister::T3_MSB) << 8) + readRegister(CalibrationRegister::T3_LSB)));

        calibration.dig_P1 = ((uint16_t)((readRegister(CalibrationRegister::P1_MSB) << 8) + readRegister(CalibrationRegister::P1_LSB)));
        calibration.dig_P2 = ((int16_t)((readRegister(CalibrationRegister::P2_MSB) << 8) + readRegister(CalibrationRegister::P2_LSB)));
        calibration.dig_P3 = ((int16_t)((readRegister(CalibrationRegister::P3_MSB) << 8) + readRegister(CalibrationRegister::P3_LSB)));
        calibration.dig_P4 = ((int16_t)((readRegister(CalibrationRegister::P4_MSB) << 8) + readRegister(CalibrationRegister::P4_LSB)));
        calibration.dig_P5 = ((int16_t)((readRegister(CalibrationRegister::P5_MSB) << 8) + readRegister(CalibrationRegister::P5_LSB)));
        calibration.dig_P6 = ((int16_t)((readRegister(CalibrationRegister::P6_MSB) << 8) + readRegister(CalibrationRegister::P6_LSB)));
        calibration.dig_P7 = ((int16_t)((readRegister(CalibrationRegister::P7_MSB) << 8) + readRegister(CalibrationRegister::P7_LSB)));
        calibration.dig_P8 = ((int16_t)((readRegister(CalibrationRegister::P8_MSB) << 8) + readRegister(CalibrationRegister::P8_LSB)));
        calibration.dig_P9 = ((int16_t)((readRegister(CalibrationRegister::P9_MSB) << 8) + readRegister(CalibrationRegister::P9_LSB)));

        calibration.dig_H1 = ((uint8_t)(readRegister(CalibrationRegister::H1)));
        calibration.dig_H2 = ((int16_t)((readRegister(CalibrationRegister::H2_MSB) << 8) + readRegister(CalibrationRegister::H2_LSB)));
        calibration.dig_H3 = ((uint8_t)(readRegister(CalibrationRegister::H3)));
        calibration.dig_H4 = ((int16_t)((readRegister(CalibrationRegister::H4_MSB) << 4) + (readRegister(CalibrationRegister::H4_LSB) & 0x0F)));
        calibration.dig_H5 = ((int16_t)((readRegister(CalibrationRegister::H5_MSB) << 4) + ((readRegister(CalibrationRegister::H4_LSB) >> 4) & 0x0F)));
        calibration.dig_H6 = ((int8_t)readRegister(CalibrationRegister::H6));

        set_control_register(Oversampling::X16, Oversampling::X16, Mode::Normal);
        return true;
    }

    uint8_t readRegister(CalibrationRegister register_name) {
        return i2c.read(address, uint8_t (register_name));
    }

    void set_temperature_oversampling(Oversampling oversampling) {
        if (enable) {
            uint8_t control = i2c.read(address, uint8_t(RegisterNames::ctrl_meas));
            control &= ~ uint8_t(0b111u << 5u);
            control |= uint8_t(oversampling) << 5u;
            i2c.write(address, uint8_t(RegisterNames::ctrl_meas), control);
        }
    }

    void set_pressure_oversampling(Oversampling oversampling) {
        if (enable) {
            uint8_t control = i2c.read(address, uint8_t(RegisterNames::ctrl_meas));
            control &= ~ uint8_t(0b111u << 2u);
            control |= uint8_t(oversampling) << 2u;
            i2c.write(address, uint8_t(RegisterNames::ctrl_meas), control);
        }
    }

    void set_mode(Mode mode) {
        if (enable) {
            uint8_t control = i2c.read(address, uint8_t(RegisterNames::ctrl_meas));
            control &= ~ uint8_t(0b11u);
            control |= uint8_t(mode);
            i2c.write(address, uint8_t(RegisterNames::ctrl_meas), control);
        }
    }

    uint8_t read_control_register() {
        if (enable) {
            return i2c.read(address, uint8_t(RegisterNames::ctrl_meas));
        } else {
            return 0;
        }
    }

    void set_control_register(Oversampling temperature_oversampling, Oversampling pressure_oversampling, Mode mode) {
        i2c.write(address, uint8_t(RegisterNames::ctrl_meas), uint8_t(uint8_t(temperature_oversampling) << 5u | uint8_t(pressure_oversampling) << 2u | uint8_t(mode)));
    }

    float read_temperature() {
        if (enable) {
            uint8_t *data = i2c.read(address, uint8_t(RegisterNames::temp_msb), 3);
            uint32_t adc_T = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((data[2] >> 4) & 0x0F);

            int64_t var1, var2;

            var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
            var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
                    ((int32_t)calibration.dig_T3)) >> 14;
            t_fine = var1 + var2;
            float output = (t_fine * 5 + 128) >> 8;

            temperature = output / 100.0f + temperature_correction;
        }
        return temperature;
    }

    float get_last_temperature() {
        return temperature;
    }

    // Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
    // Output value of “47445” represents 47445/1024 = 46. 333 %RH
    float read_humidity() {
        if (enable) {
            uint8_t *data = i2c.read(address, uint8_t(RegisterNames::hum_msb), 2);

            int32_t adc_H = ((uint32_t)data[0] << 8) | ((uint32_t)data[1]);

            int32_t var1;
            var1 = (t_fine - ((int32_t)76800));
            var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
                      ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                                                   ((int32_t)calibration.dig_H2) + 8192) >> 14));
            var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
            var1 = (var1 < 0 ? 0 : var1);
            var1 = (var1 > 419430400 ? 419430400 : var1);

            humidity = (float)(var1>>12) / 1024.0;

        } else {
            humidity = 0.0f;
        }
        return humidity;
    }

    // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
    // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    float read_pressure() {
        if (enable) {
            uint8_t *data = i2c.read(address, uint8_t(RegisterNames::press_msb), 3);

            int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((data[2] >> 4) & 0x0F);

            int64_t var1, var2, p_acc;
            var1 = ((int64_t)t_fine) - 128000;
            var2 = var1 * var1 * (int64_t)calibration.dig_P6;
            var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17);
            var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
            var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
            var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
            if (var1 == 0) {
                return 0; // avoid exception caused by division by zero
            }
            p_acc = 1048576 - adc_P;
            p_acc = (((p_acc<<31) - var2)*3125)/var1;
            var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
            var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
            p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);
            pressure = (float)p_acc / 256.0;
        }
        return pressure;
    }

    void run_measurements() {
        read_temperature();
        read_humidity();
        read_pressure();
    }

    uint16_t get_last_temperature_multiplied() {
        return uint16_t (temperature * 100.0f);
    }

    uint16_t get_last_humidity() {
        return uint16_t (humidity);
    }

    uint32_t get_last_pressure() {
        return uint32_t (pressure);
    }

    void set_enable(bool enable_) {
        this->enable = enable_;
    }

};
