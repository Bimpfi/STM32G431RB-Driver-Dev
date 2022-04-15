#ifndef __I2C_MASTER_H
#define __I2C_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif
	#include "stm32g431xx.h"
#ifdef __cplusplus
}
#endif

#include "i2c_interface.h"

class I2C_Master : public I2C_Interface<I2C_Master> {
	//uint8_t default_addr = 0x3C;

 public:
	static void init();
	static void write(uint8_t addr, uint8_t* data, uint16_t len, bool repeated_start = false);
	//inline uint8_t writeToDefaultAddr(uint8_t* data, uint16_t len) {this->write(this->default_addr, data, len, false); return 0;}
	//void setDefaultAddr(uint8_t addr) {this->default_addr = addr; }
	static void read(uint8_t addr, uint8_t* data, uint16_t len, bool stop_cond = false);
};

#endif
