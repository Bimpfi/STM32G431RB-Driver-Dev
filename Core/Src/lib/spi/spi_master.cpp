#include "spi_master.h"

void SPI_Master::init() {
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk;
}

void SPI_Master::write(uint8_t* data, uint32_t len) {}

void SPI_Master::read(uint8_t* data, uint32_t len) {}