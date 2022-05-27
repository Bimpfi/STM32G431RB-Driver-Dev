#include "spi_master.h"

void SPI_Master::init() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN_Msk;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk;
  GPIOA->MODER =
      (GPIOA->MODER & ~(GPIO_MODER_MODE4_Msk | GPIO_MODER_MODE5_Msk |
                        GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk)) |
      (0x02 << GPIO_MODER_MODE4_Pos) | (0x02 << GPIO_MODER_MODE5_Pos) |
      (0x02 << GPIO_MODER_MODE6_Pos) | (0x02 << GPIO_MODER_MODE7_Pos);

  GPIOA->OTYPER =
      (GPIOA->OTYPER & ~(GPIO_OTYPER_OT4_Msk | GPIO_OTYPER_OT5_Msk |
                         GPIO_OTYPER_OT6_Msk | GPIO_OTYPER_OT7_Msk));

  GPIOA->PUPDR =
      (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPD4_Msk | GPIO_PUPDR_PUPD5_Msk |
                        GPIO_PUPDR_PUPD6_Msk | GPIO_PUPDR_PUPD7_Msk)) |
      (0x01 << GPIO_PUPDR_PUPD4_Pos);
  GPIOA->AFR[0] =
      (GPIOA->AFR[0] & ~(GPIO_AFRL_AFSEL4_Msk | GPIO_AFRL_AFSEL5_Msk |
                         GPIO_AFRL_AFSEL6_Msk | GPIO_AFRL_AFSEL7_Msk)) |
      (0x05 << GPIO_AFRL_AFSEL4_Pos) | (0x05 << GPIO_AFRL_AFSEL5_Pos) |
      (0x05 << GPIO_AFRL_AFSEL6_Pos) | (0x05 << GPIO_AFRL_AFSEL7_Pos);
  SPI1->CR1 = (0x04 << SPI_CR1_BR_Pos) | SPI_CR1_MSTR_Msk;
  SPI1->CR2 |= SPI_CR2_SSOE_Msk | (0x7 & SPI_CR2_DS_Msk);
}

void SPI_Master::write(uint8_t* data, uint32_t len) {
  SPI1->CR1 |= SPI_CR1_SPE_Msk;
  while (len--) {
    SPI1->DR = *data;
    // while (((SPI1->SR & SPI_SR_FRLVL_Msk) >> SPI_SR_FRLVL_Pos) == 0x03)
    //   ;
    data++;
    while (!(SPI1->SR & SPI_SR_TXE_Msk) && (SPI1->SR & SPI_SR_BSY_Msk))
      ;
  }

  SPI1->CR1 &= ~SPI_CR1_SPE_Msk;
}

void SPI_Master::read(uint8_t* data, uint32_t len) {}