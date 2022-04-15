#include "uart.h"

void UART::init() {
    PWR->CR1 |= PWR_CR1_DBP_Msk;
    RCC->BDCR |= RCC_BDCR_LSEON_Msk;
    while(!(RCC->BDCR & RCC_BDCR_LSERDY_Msk));
    RCC->CCIPR |= RCC_CCIPR_LPUART1SEL_Msk;
    GPIOA->MODER &= ~GPIO_MODER_MODE2_0;
    GPIOA->MODER |= GPIO_MODER_MODE2_1;
    GPIOA->OTYPER |= GPIO_OTYPER_OT_2;
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED2_Msk;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD2_Msk;

    GPIOA->MODER &= ~GPIO_MODER_MODE3_0;
    GPIOA->MODER |= GPIO_MODER_MODE3_1;
    GPIOA->OTYPER |= GPIO_OTYPER_OT_3;
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED3_Msk;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD3_Msk;

    GPIOA->AFR[0] = GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL2_Msk | (0x0CUL << GPIO_AFRL_AFSEL2_Pos);
    GPIOA->AFR[0] = GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL3_Msk | (0x0CUL << GPIO_AFRL_AFSEL3_Pos);
    
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN_Msk;
}

void UART::read(uint8_t* buf, uint8_t len) {

}

void UART::write(uint8_t* data, uint8_t len) {
    LPUART1->BRR = 0x369;
    LPUART1->CR2 &= USART_CR2_STOP_Msk;
    LPUART1->CR1 |= USART_CR1_UE_Msk;
    LPUART1->CR1 |= USART_CR1_TE_Msk;

    for(uint8_t i = 0; i < len; i++) {
        while(!(LPUART1->ISR & USART_ISR_TXE_Msk));
        LPUART1->TDR = data[i];
    }
    while(!(LPUART1->ISR & USART_ISR_TC_Msk));

    LPUART1->CR1 &= ~USART_CR1_TE_Msk;
    LPUART1->CR1 &= ~USART_CR1_UE_Msk;
}
