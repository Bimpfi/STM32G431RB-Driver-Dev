#ifndef __I2C_MASTER_H
#define __I2C_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32g4xx.h"
#ifdef __cplusplus
}
#endif

class I2C_Master {
 public:
  static void init();
  static void write(uint8_t addr,
                    uint8_t* data,
                    uint16_t len,
                    bool repeated_start = false);
  static void read(uint8_t addr,
                   uint8_t* data,
                   uint16_t len,
                   bool stop_cond = false);
};

#endif
