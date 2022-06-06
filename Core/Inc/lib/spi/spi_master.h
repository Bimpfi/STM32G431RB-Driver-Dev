#ifndef __SPI_MASTER_H
#define __SPI_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32g431xx.h"
#ifdef __cplusplus
}
#endif

#include "spi_interface.h"

class SPI_Master : public SPI_Interface<SPI_Master> {
 public:
  static void init();
  static void write(uint8_t* data, uint32_t len);
  static void read(uint8_t* data, uint32_t len);
};

#endif /* __SPI_MASTER_H */
