#include "i2c_master.h"

void I2C_Master::init() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN_Msk;

  GPIOB->MODER &= ~GPIO_MODER_MODER8_0;
  GPIOB->MODER |= GPIO_MODER_MODER8_1;
  GPIOB->OTYPER |= GPIO_OTYPER_OT_8;
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_Msk;
  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD8_Msk;

  GPIOB->MODER &= ~GPIO_MODER_MODER9_0;
  GPIOB->MODER |= GPIO_MODER_MODER9_1;
  GPIOB->OTYPER |= GPIO_OTYPER_OT_9;
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED9_Msk;
  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD9_Msk;

  I2C1->CR1 &= ~I2C_CR1_PE_Msk;
  GPIOB->AFR[1] =
      GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL8 | (4UL << GPIO_AFRH_AFSEL8_Pos);
  GPIOB->AFR[1] =
      GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL9 | (4UL << GPIO_AFRH_AFSEL9_Pos);
  RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN_Msk;

  //	I2C1->CR2 |= I2C_CR2_RELOAD_Msk;
  I2C1->CR2 &= ~I2C_CR2_ADD10_Msk;
  I2C1->TIMINGR |=
      (0x07 << I2C_TIMINGR_PRESC_Pos) | (0x13 << I2C_TIMINGR_SCLL_Pos) |
      (0x0F << I2C_TIMINGR_SCLH_Pos) | (0x02 << I2C_TIMINGR_SDADEL_Pos) |
      (0x04 << I2C_TIMINGR_SCLDEL_Pos);

  I2C1->CR1 |= I2C_CR1_PE_Msk;
}
void I2C_Master::write(uint8_t addr,
                       uint8_t* data,
                       uint16_t len,
                       bool repeated_start) {
  I2C1->CR2 |= addr << (I2C_CR2_SADD_Pos + 1);
  I2C1->CR2 &= ~I2C_CR2_RD_WRN_Msk;
  volatile uint16_t pos = 0;
  while (true) {
    uint8_t tx_len;
    if (len > 0xFF) {
      I2C1->CR2 |= I2C_CR2_RELOAD_Msk;
      tx_len = 0xFF;
      len -= 0xFF;
    } else {
      I2C1->CR2 &= ~I2C_CR2_RELOAD_Msk;
      tx_len = len;
      len = 0;
    }

    I2C1->CR2 &= ~I2C_CR2_NBYTES_Msk;  // Reset NBYTES register
    I2C1->CR2 |= tx_len << I2C_CR2_NBYTES_Pos;
    if (pos == 0) {
      I2C1->CR2 |= I2C_CR2_START_Msk;
    }
    while (tx_len--) {
      while (!(I2C1->ISR & I2C_ISR_TXIS_Msk)) {
        if (I2C1->ISR & I2C_ISR_NACKF_Msk) {
          while (I2C1->ISR & I2C_ISR_BUSY_Msk)
            ;
          return;
        }
      }
      I2C1->TXDR = data[pos++] << I2C_TXDR_TXDATA_Pos;
    }
    if (!(I2C1->CR2 & I2C_CR2_RELOAD_Msk)) {
      uint16_t timeout = 0;
      if (!repeated_start) {
        while (!(I2C1->ISR & I2C_ISR_TC_Msk)) {
          if (((I2C1->ISR >> I2C_ISR_TXE_Pos) &
               (I2C1->ISR >> I2C_ISR_TXE_Pos)) &&
              (timeout++) > 1000) {
            I2C1->CR1 &= ~I2C_CR1_PE_Msk;
            I2C1->CR1 |= I2C_CR1_PE_Msk;
          }
          return;
        }

        I2C1->CR2 |= I2C_CR2_STOP_Msk;
        while (I2C1->ISR & I2C_ISR_BUSY_Msk)
          ;
        I2C1->ICR |= I2C_ICR_STOPCF_Msk;
        return;
      }
    } else if (I2C1->CR2 & I2C_CR2_RELOAD_Msk) {
      while (!(I2C1->ISR & I2C_ISR_TCR_Msk))
        ;
    }
  }
}
void I2C_Master::read(uint8_t addr,
                      uint8_t* data,
                      uint16_t len,
                      bool stop_cond) {
  uint16_t counter = 0;
  I2C1->CR2 |= I2C_CR2_RD_WRN_Msk;
  while (len > 0) {
    uint8_t tx_len;

    if (len > 0xFF) {
      I2C1->CR2 |= I2C_CR2_RELOAD_Msk;
      tx_len = 0xFF;
      len -= 0xFF;
    } else {
      I2C1->CR2 &= ~I2C_CR2_RELOAD_Msk;
      tx_len = len;
      len = 0;
    }

    I2C1->CR2 |= tx_len << I2C_CR2_NBYTES_Pos;
    I2C1->CR2 |= I2C_CR2_START_Msk;
    for (uint8_t i = 0; i < tx_len; i++) {
      while (!(I2C1->ISR & I2C_ISR_RXNE_Msk))
        ;

      data[counter++] = I2C1->RXDR;
    }

    if ((I2C1->ISR & I2C_ISR_TC_Msk) && !stop_cond) {
      I2C1->CR2 |= I2C_CR2_STOP_Msk;
    }
  }
}
