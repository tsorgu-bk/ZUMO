
#include "UART_STM32F4x.h"
#include <stm32f4xx.h>

#define UART_BRR_MIN    0x10U        /* UART BRR minimum authorized value */
#define UART_BRR_MAX    0x0000FFFFU  /* UART BRR maximum authorized value */

UART_STM32F4x* uartHandlers[6];

UART_STM32F4x::UART_STM32F4x(USART_TypeDef* uart, uint32_t baudrate) :
        uart(uart), baudrate(baudrate) {
    if(USART1 == uart){
        uartHandlers[0] = this;
    } else if(USART2 == uart){
        uartHandlers[1] = this;
    } else if(USART3 == uart){
        uartHandlers[2] = this;
    }
    RingBuffer_Init(&rxRingBuffer, rxBuffer, rxBufferSize);
    RingBuffer_Init(&txRingBuffer, txBuffer, txBufferSize);
}

void UART_STM32F4x::init(){
    // disable
    uart->CR1 &= ~ USART_CR1_UE;
    if(uart == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
//        RCC->CFGR3 &= ~(RCC_CFGR3_USART1SW_Msk);
//        RCC->CFGR3 |= RCC_CFGR3_USART1SW_PCLK1;
    } else if(uart == USART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    } else if(uart == USART3) {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    } else {
        return;
    }

    /* In multiprocessor mode, the following bits must be kept cleared:
    - LINEN and CLKEN bits in the USART_CR2 register,
    - SCEN, HDSEL and IREN  bits in the USART_CR3 register. */
    CLEAR_BIT(uart->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
    CLEAR_BIT(uart->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

//    uint32_t usartdiv = 0x00000000U;
//    uint32_t pclk;
//
//    pclk = HAL_RCC_GetPCLK1Freq();
//
//    usartdiv = (uint16_t)((((pclk)) + ((baudrate)/2U)) / (baudrate));
//
//    if ((usartdiv >= UART_BRR_MIN) && (usartdiv <= UART_BRR_MAX)) {
//        uart->BRR = usartdiv;
//    }

    // enable
    uart->CR1 |= USART_CR1_UE;

    // transmit and receive
    uart->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_TCIE ;

    if(USART1 == uart){
        NVIC_ClearPendingIRQ(USART1_IRQn);
        NVIC_EnableIRQ(USART1_IRQn);
    } else if(USART2 == uart){
        NVIC_ClearPendingIRQ(USART2_IRQn);
        NVIC_EnableIRQ(USART2_IRQn);
    } else if(USART3 == uart){
        NVIC_ClearPendingIRQ(USART3_IRQn);
        NVIC_EnableIRQ(USART3_IRQn);
    }
    enableInterrupt(InterruptType::RX_FULL);
}

void UART_STM32F4x::write(uint8_t* data, uint16_t length){
    // enter critical section
    __disable_irq();
    // put data to ring buffer
    for(uint16_t i=0; i<length; i++){
        RingBuffer_PutChar(&txRingBuffer, *(reinterpret_cast<char*>(data + i)));
    }
    // exit critical section
    __enable_irq();
//    if (!DMAworking) {
    // enable TX empty interrupt
    enableInterrupt(InterruptType::TX_EMPTY);
//    }
}

int UART_STM32F4x::write(uint8_t data) {
    // enter critical section
    __disable_irq();
    // put data to ring buffer
    RingBuffer_PutChar(&txRingBuffer, data);
    // exit critical section
    __enable_irq();
//    if (!DMAworking) {
    // enable TX empty interrupt
    enableInterrupt(InterruptType::TX_EMPTY);
//    }
    return 0;
}

uint8_t UART_STM32F4x::read() {
    static uint8_t c;
    RingBuffer_GetChar(&rxRingBuffer, &c);
    return c;
}

void UART_STM32F4x::read(uint8_t* buffer, uint16_t length){
    if(buffer != nullptr){
        if(length <= RingBuffer_GetLen(&rxRingBuffer)){
            for(auto i=0; i<length; i++){
                RingBuffer_GetChar(&rxRingBuffer, &buffer[i]);
            }
        }
    }
}

void UART_STM32F4x::enableInterrupt(InterruptType interrupt){
    (void) interrupt;
    uart->CR1 |= static_cast<uint8_t >(interrupt);

    if (interrupt == InterruptType::TX_EMPTY && !transmitting) {
        transmitting = true;
        beforeTx();
    }
}

void UART_STM32F4x::disableInterrupt(InterruptType interrupt){
    (void) interrupt;
    uart->CR1 &= ~(static_cast<uint8_t>(interrupt));
}

int UART_STM32F4x::interrupt() {
    uint32_t isrflags   = uart->SR;
    uint32_t cr1its     = uart->CR1;
    uint8_t data        = uart->DR;
    uint32_t errorflags = 0x00U;

    // If no error occurs
    errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
    if (errorflags == RESET) {
        // UART in mode Receiver -------------------------------------------------
        if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET)) {
//            if (redirectHandler) {
//                redirectHandler(data);
//            } else {
            RingBuffer_PutChar(&rxRingBuffer, data);
//            }

            if (isrflags & USART_SR_RXNE) {
                uart->SR &= ~ USART_SR_RXNE;
            }

            return 1;
        }

        // UART in mode Transmitter -------------------------------------------------
        if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET)) {
            uint8_t c;
            if (RingBuffer_GetChar(&txRingBuffer, &c)) {
                uart->DR = c;
            } else {
                disableInterrupt(InterruptType::TX_EMPTY);
            }
        }
    }

    if (isrflags & USART_SR_IDLE) {
        uart->SR &= ~ USART_SR_IDLE;
    }

    if (isrflags & USART_SR_TXE) {
        uart->SR &= ~ USART_SR_TXE;
    }

    if (isrflags & USART_SR_TC) {
        uart->SR &= ~ USART_SR_TC;
        if(RingBuffer_IsEmpty(&txRingBuffer) && transmitting) {
            afterTx();
            transmitting = false;
        }
    }

    return 1;
}

void UART_IRQ(UART_STM32F4x* uartHandler) {
    uartHandler->interrupt();
}

extern "C" {
void USART1_IRQHandler() {
    UART_IRQ(uartHandlers[0]);
}

void USART2_IRQHandler() {
    UART_IRQ(uartHandlers[1]);
}

void USART3_IRQHandler() {
    UART_IRQ(uartHandlers[2]);
}

void UART4_IRQHandler() {
    UART_IRQ(uartHandlers[3]);
}

void UART5_IRQHandler() {
    UART_IRQ(uartHandlers[4]);
}

void USART6_IRQHandler() {
    UART_IRQ(uartHandlers[5]);
}
}