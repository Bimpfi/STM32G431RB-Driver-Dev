#ifndef __ST7XX_H_
#define __ST7XX_H_

#include "fonts.h"
#include "spi_interface.h"
#include "st77xx_enum.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32g4xx.h"
#ifdef __cplusplus
}
#endif

typedef struct pos_t {
  uint8_t x;
  uint8_t y;
} pos_t;

template <void (*spi_write)(uint8_t* data, uint32_t len)>
class ST77XX {
 private:
  pos_t pos;
  inline void sendData(uint8_t* data, uint32_t len) {
    if (len == 0 || data == nullptr)
      return;
    GPIOB->ODR |= GPIO_ODR_OD5_Msk;
    spi_write(data, len);
  }

  inline void sendCommand(uint8_t* data, uint32_t len) {
    if (len == 0 || data == nullptr)
      return;

    GPIOB->ODR = ~(GPIO_ODR_OD5_Msk);
    spi_write(data, 1);

    if (len == 1)
      return;

    this->sendData(data + 1, len - 1);
  }

  inline void sendWriteRAMCommand() {
    uint8_t init_data[] = {0x2C};
    this->sendCommand(init_data, sizeof(init_data) / sizeof(uint8_t));
  }

 public:
  ST77XX() : pos{0, 0} {}
  void init() {
    uint8_t init_data[] = {SWRESET, SLPOUT, COLMOD, 0x05,
                           NORON,   DISPON, MADCTL, 0x60};
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN_Msk;
    GPIOB->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER5_0;
    GPIOB->MODER &= ~(GPIO_MODER_MODER0_1 | GPIO_MODER_MODER5_1);
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT5);

    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR5_0);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR5_1);

    GPIOB->ODR = ~(GPIO_ODR_OD5_Msk);

    GPIOB->ODR |= GPIO_ODR_OD0_Msk;
    LL_mDelay(200);
    GPIOB->ODR &= ~GPIO_ODR_OD0_Msk;
    LL_mDelay(200);
    GPIOB->ODR |= GPIO_ODR_OD0_Msk;
    LL_mDelay(200);

    this->sendCommand(&init_data[0], 1);
    LL_mDelay(150);
    this->sendCommand(&init_data[1], 1);
    LL_mDelay(500);
    this->sendCommand(&init_data[2], 2);
    LL_mDelay(150);
    this->sendCommand(&init_data[4], 1);
    LL_mDelay(150);
    this->sendCommand(&init_data[5], 1);
    LL_mDelay(150);
    this->sendCommand(&init_data[6], 2);
    LL_mDelay(150);
  }

  void setPixel(uint8_t x, uint8_t y, uint16_t color = 0x1234) {
    uint8_t init_data[] = {0x2C};
    this->setDrawArea(x, y, x, y);
    this->sendCommand(init_data, sizeof(init_data) / sizeof(uint8_t));
    this->sendData(reinterpret_cast<uint8_t*>(&color), 2);
  }

  void writeCharacter(uint8_t x,
                      uint8_t y,
                      char ch,
                      FontDef font,
                      uint16_t color,
                      uint16_t bgcolor) {
    uint32_t i, b, j;
    this->setDrawArea(x, y, x + font.width - 1, y + font.height - 1);
    this->sendWriteRAMCommand();

    for (i = 0; i < font.height; i++) {
      b = font.data[(ch - 32) * font.height + i];
      for (j = 0; j < font.width; j++) {
        if ((b << j) & 0x8000) {
          uint8_t data[] = {color >> 8, color & 0xFF};
          this->sendData(data, sizeof(data));
        } else {
          uint8_t data[] = {bgcolor >> 8, bgcolor & 0xFF};
          this->sendData(data, sizeof(data));
        }
      }
    }
  }

  void setDrawArea(uint8_t x_start = 0x00,
                   uint8_t y_start = 0x00,
                   uint8_t x_end = 0xA0,
                   uint8_t y_end = 0x7F) {
    uint8_t init_row[] = {0x2A, 0x00, x_start, 0x00, x_end};
    uint8_t init_col[] = {0x2B, 0x00, y_start, 0x00, y_end};
    this->sendCommand(init_row, sizeof(init_row) / sizeof(uint8_t));
    this->sendCommand(init_col, sizeof(init_col) / sizeof(uint8_t));
  }

  void fillScreen(uint16_t color = 0x0000) {
    this->setDrawArea();
    this->sendWriteRAMCommand();
    for (uint32_t i = 0; i < 20480; i++) {
      this->sendData(reinterpret_cast<uint8_t*>(&color), 2);
    }
  }

  void showTestImage() {
    uint8_t color[] = {0x00, 0x00};

    static uint16_t pos = 0;
    static uint8_t i = 0;

    this->setDrawArea();
    this->sendWriteRAMCommand();

    this->writeCharacter(
        (pos * (Font_7x10.width + 1)) % 160,
        (((pos * (Font_7x10.width + 1)) / 160) * (Font_7x10.height + 2) + 4) %
            120,
        48 + (i % 36), Font_7x10, 0xffff, 0x0000);
    pos++;
    i++;
    LL_mDelay(25);
  }
};

#endif /* __ST7XX_H_ */
