#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <cstring>
#include <queue>
#include "i2c_master.h"
#include "ssd1306_alphabet.h"
#include "ssd1306_enum.h"

static const uint8_t init_cmds[] = {
    COMMANDS::SET_DISPLAY_OFF,
    COMMANDS::SET_DISPLAY_CLK_DIVIDER_CONTROL,
    COMMANDS::SET_DISPLAY_CLK_DIVIDER_128,
    COMMANDS::SET_MULTIPLEX_RATIO_CONTROL,
    COMMANDS::SET_MULTIPLEX_RATIO_64,
    COMMANDS::SET_DISPLAY_OFFSET_CONTROL,
    0x00,  // no offset
    COMMANDS::SET_START_LINE | 0x00,
    COMMANDS::SET_CHARGE_BUMP_CONTROL,
    COMMANDS::SET_CHARGE_BUMP_ON,
    COMMANDS::SET_SEG_REMAP_ON,
    COMMANDS::SET_COM_SCAN_DECREMENT,
    COMMANDS::SET_COM_PINS_CONFIG_CONTROL,
    COMMANDS::SET_COM_PINS_ALT_NO_REMAP,
    COMMANDS::SET_CONTRAST_CONTROL,
    COMMANDS::SET_CONTRAST_207,
    COMMANDS::SET_PRECHARGE_CONTROL,
    COMMANDS::SET_PRECHARGE_PHASE1_1_PHASE2_15,
    COMMANDS::SET_VCOM_DESELECT_LEVEL_CONTROL,
    COMMANDS::SET_VCOM_DESELECT_LEVEL_07VCC,
    COMMANDS::SET_MEMORY_ADDRESSING_MODE_CONTROL,
    COMMANDS::SET_HORIZONTAL_MEMORY_ADDRESSING_MODE,
    COMMANDS::SET_PAGE_ADDR_CONTROL,
    0x00,  // page start address = 0
    0x07,  // page end address = 7;
    COMMANDS::SET_COLUMN_ADDR,
    0x00,  // column start address = 0
    0x7F,  // column end address = 127
    COMMANDS::SET_DISPLAY_BY_RAM,
    COMMANDS::SET_NORMAL_DISPLAY,
    COMMANDS::SET_DEACTIVATE_SCROLL,
    COMMANDS::SET_DISPLAY_ON,
};

template <uint8_t WIDTH,
          uint8_t HEIGHT,
          uint8_t PPB,
          uint8_t ADDR,
          void (*i2c_write)(uint8_t addr,
                            uint8_t* sendData,
                            uint16_t len,
                            bool repeated_start)>
class SSD1306 {
 private:
  uint8_t framebuffer[(WIDTH * HEIGHT) / PPB];
  uint8_t current_page, current_column;
  std::queue<char>
      stringBuf;  // Saves String which will be printed slowly step by step

  uint8_t sendCommand(const uint8_t command) {
    unsigned char buf[2] = {CONTROL_BYTE_COMMANDS::SEND_SINGLE_CMD, command};
    i2c_write(ADDR, buf, 2, false);
    return 0;
  }

  uint8_t lengthOf(const uint8_t* bytes) {
    uint8_t result = 0;
    for (auto i = 0; bytes[i] != 0x00; ++i)
      ++result;
    return result;
  }

  uint8_t commandLine(const uint8_t* command_buf, uint8_t len) {
    unsigned char buf[len] = {CONTROL_BYTE_COMMANDS::SEND_MULTIPLE_CMDS};
    memcpy(buf + 1, command_buf, len);
    i2c_write(ADDR, buf, len + 1, false);
    return 0;
  }

  uint8_t sendData(const uint8_t data) {
    unsigned char buf[2] = {CONTROL_BYTE_COMMANDS::SEND_SINGLE_DATA, data};
    i2c_write(ADDR, buf, 2, false);
    return 0;
  }

  uint8_t sendMultipleData(const uint8_t* data_buf, uint8_t len) {
    unsigned char buf[len] = {CONTROL_BYTE_COMMANDS::SEND_MULTIPLE_DATA};
    memcpy(buf + 1, data_buf, len);
    i2c_write(ADDR, buf, len + 1, false);
    return 0;
  }

  /**
   * Sets position to the beginning of the next line
   */
  void nextPage() {
    ++this->current_page;
    if (this->current_page > 7) {
      this->current_page = 0;
      clearDisplay();
    }
    this->current_column = 0;
    setPosition(this->current_page, this->current_column);
  }

 public:
  SSD1306() : framebuffer{0}, current_page(0), current_column(0) {}

  void init() {
    this->commandLine(init_cmds, sizeof(init_cmds));
    this->setPosition(0, 0);
    RCC->AHB1ENR |= 0x10;
    TIM6->PSC = 16000 - 1;
    TIM6->ARR = 49;
    TIM6->CR1 = 1;
    NVIC->ICPR[1] = (1 << 22);
    NVIC->ISER[1] = (1 << 22);
    TIM6->DIER = 1;
  }

  void update() {
    if (TIM6->SR == 1) {
      this->showNextInStringBuffer();
      TIM6->SR = 0;
    }
  }

  const bool isBufferEmpty() { return stringBuf.empty(); }

  /**
   * Prints ASCII Symbol
   */
  void printSymbol(char symbol) {
    if (symbol == '\n')
      nextPage();
    else if (symbol == ' ') {
      this->current_column += 2;  // two steps, because one is not enough
      if (this->current_column > 127)
        nextPage();
    }
    if (symbol >= 33 && symbol <= 126) {
      const uint8_t* byte = ascii[symbol - 33];
      setAndShowBytesFromCurrentPosition(byte, lengthOf(byte) + 1);
    }
    setPosition(current_page, current_column);
  }

  void setTextSpeed(uint32_t ms) {
    TIM6->ARR = ms - 1;
    TIM6->CNT = 0;
  }

  void setAndShowByteAtCurrentPosition(const uint8_t data) {
    if (this->sendData(data) != 0)
      return;

    this->framebuffer[this->current_column + (this->current_page * WIDTH)] =
        data;

    this->current_column = (++this->current_column) & (WIDTH - 1);
    this->current_page = (this->current_column != 0)
                             ? this->current_page
                             : (this->current_page + 1) & ((HEIGHT / PPB) - 1);
  }

  void setAndShowByteAtAnotherPosition(const uint8_t data,
                                       uint8_t page,
                                       uint8_t column) {
    uint8_t savePosPage = this->current_page;
    uint8_t savePosColumn = this->current_column;

    setAndShowByteAtCurrentPosition(data);

    setPosition(savePosPage, savePosColumn);
  }

  void setAndShowBytesFromCurrentPosition(const uint8_t* data_buf,
                                          uint8_t len) {
    if (this->current_column + len > 127)
      nextPage();

    if (this->sendMultipleData(data_buf, len) != 0)
      return;

    memcpy((this->framebuffer + this->current_column +
            (this->current_page * WIDTH)),
           data_buf, len);

    this->current_page =
        (this->current_page + (this->current_column + len) / WIDTH) &
        ((HEIGHT / PPB) - 1);
    this->current_column = (this->current_column + len) & (WIDTH - 1);
  }

  /**
   * Sets and shows byte on requested position without loosing previous position
   */
  void setAndShowBytesFromAnotherPosition(const uint8_t* data_buf,
                                          uint8_t len,
                                          uint8_t page,
                                          uint8_t column) {
    uint8_t savePosPage = this->current_page;
    uint8_t savePosColumn = this->current_column;

    setPosition(page, column);

    setAndShowBytesFromCurrentPosition(data_buf, len);
  }

  void setStringToPutSlowly(const char* str) {
    if (stringBuf.size() < 200)
      for (auto i = 0; str[i] != '\0'; ++i)
        stringBuf.push(str[i]);
  }

  void showNextInStringBuffer() {
    if (!stringBuf.empty()) {
      char next = stringBuf.front();
      stringBuf.pop();
      printSymbol(next);
    }
  }

  void setAndShowStringFromCurrentPosition(const char* str) {
    for (auto i = 0; str[i] != '\0'; ++i)
      printSymbol(str[i]);
  }

  void setAndShowStringFromAnotherPosition(const char* str,
                                           uint8_t page,
                                           uint8_t column) {
    uint8_t savePosPage = this->current_page;
    uint8_t savePosColumn = this->current_column;
    setPosition(page, column);

    setAndShowStringFromCurrentPosition(str);

    setPosition(savePosPage, savePosColumn);
  }

  void setPixel(uint8_t pos_x, uint8_t pos_y, uint8_t value) {
    if (pos_x >= WIDTH)
      return;
    if (pos_y >= HEIGHT)
      return;

    this->framebuffer[pos_x + (pos_y / PPB) * WIDTH] =
        (this->framebuffer[pos_x + (pos_y / PPB) * WIDTH] &
         ~(1 << pos_y % PPB)) |
        (value << pos_y % PPB);
  }

  void showDisplay() {
    // unsigned char buf_out[(WIDTH * HEIGHT) / PPB + 1] = {
    //     CONTROL_BYTE_COMMANDS::SEND_MULTIPLE_DATA};

    // if (this->current_column != 0 || this->current_page != 0) {
    //   this->setPosition(0, 0);
    // }

    // memcpy(buf_out + 1, this->framebuffer, (WIDTH * HEIGHT) / PPB);
    // i2c_write(ADDR, buf_out, sizeof(buf_out), false);

    // if (this->current_column != 0 || this->current_page != 0) {
    //   this->setPosition(this->current_page, this->current_column);
    // }
    this->showDisplayInMultipleTransmissions();
  }

  void showDisplayInMultipleTransmissions() {
    unsigned char buf_out[(WIDTH * HEIGHT) / PPB + 1] = {
        CONTROL_BYTE_COMMANDS::SEND_MULTIPLE_DATA};

    if (this->current_column != 0 || this->current_page != 0) {
      this->setPosition(0, 0);
    }

    for (uint8_t i = 0; i < 4; i++) {
      memcpy(buf_out + 1, this->framebuffer + (i * 254), 254);
      i2c_write(ADDR, buf_out, 255, false);
    }

    memcpy(buf_out + 1, this->framebuffer + (4 * 254), 8);
    i2c_write(ADDR, buf_out, 9, false);

    if (this->current_column != 0 || this->current_page != 0) {
      this->setPosition(this->current_page, this->current_column);
    }
  }

  void setPosition(uint8_t page, uint8_t column) {
    const uint8_t cmds[] = {
        COMMANDS::SET_PAGE_ADDR_CONTROL,
        (uint8_t)(page & ((HEIGHT / PPB) -
                          1)),  // page start address = page % (height/ppb)
        0x07,                   // page end address = 7;
        COMMANDS::SET_COLUMN_ADDR,
        (uint8_t)(column &
                  (WIDTH - 1)),  // column start address = column % width
        0x7F,                    // column end address = 127
    };

    if (this->commandLine(cmds, sizeof(cmds)) == 0) {
      this->current_page = cmds[1];
      this->current_column = cmds[4];
    }
  }

  void clearStringBuffer() {
    while (!stringBuf.empty())
      stringBuf.pop();
  }

  void clearDisplay() {
    for (uint16_t i = 0; i < sizeof(framebuffer); i++) {
      this->framebuffer[i] = 0;
    }
    this->showDisplay();
  }

  void showTestGrid() {
    for (uint16_t i = 0; i < sizeof(this->framebuffer) * PPB; i++) {
      this->framebuffer[(i / PPB)] =
          (i % 2 && (i / PPB) % 2)
              ? this->framebuffer[i / PPB] & ~(1 << (i % PPB))
              : this->framebuffer[i / PPB] | 1 << (i % PPB);
    }
    this->showDisplay();
  }
};
#endif /* SSD1306_H */
