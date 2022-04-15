#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32g431xx.h"
#ifdef __cplusplus
}
#endif

class UART {
 public:
    void init();
    void write(uint8_t* data, uint8_t len);
    void read(uint8_t* buf, uint8_t len);
};

#endif /* __UART_H */
