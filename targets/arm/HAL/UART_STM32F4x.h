#pragma once

#include "SimpleStream.h"
#include "GPIO_STM32F4x.h"
#include "ring_buffer.h"
#include "stm32f4xx.h"
#include <cstdint>

class UART_STM32F4x : public SimpleStream {
    USART_TypeDef* uart;
    RingBuffer rxRingBuffer{};
    RingBuffer txRingBuffer{};

private:
    uint32_t baudrate;

    constexpr static auto txBufferSize = 2048;
    constexpr static auto rxBufferSize = 1024;

    uint8_t txBuffer[txBufferSize] = {0};
    uint8_t rxBuffer[rxBufferSize] = {0};

    bool transmitting = false;

    enum class InterruptType : uint8_t {
        TX_EMPTY = USART_CR1_TXEIE ,
        TX_COMPLETE = USART_CR1_TCIE,
        RX_FULL = USART_CR1_RXNEIE
    };
public:
    UART_STM32F4x(USART_TypeDef* uart, uint32_t baudrate);

    void init() override ;

    uint16_t available() override {
        return RingBuffer_GetLen(&rxRingBuffer);
    }

    void enableInterrupt(InterruptType interrupt);

    void disableInterrupt(InterruptType interrupt);

    int write(uint8_t data);
    void write(uint8_t* data, uint16_t length) override;

    uint8_t read() override ;

    void read(uint8_t* buffer, uint16_t length);

    void flush() override {
        RingBuffer_Clear(&rxRingBuffer);
        RingBuffer_Clear(&txRingBuffer);
    }

    int interrupt();

    virtual void beforeTx() {}
    virtual void afterTx() {}
};