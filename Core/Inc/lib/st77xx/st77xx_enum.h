#ifndef __ST77XX_ENUM_
#define __ST77XX_ENUM_

/**
 * @brief Register addresses of the ST77XX display
 *
 */
typedef enum ST77XX_SYSTEM_REGISTER {
  NOP = 0x00,
  SWRESET = 0x01,
  RDDID = 0x04,
  RDDST = 0x09,
  RDDPM = 0x0A,
  RDDMADCTL = 0x0B,
  RDDCOLMOD = 0x0C,
  RDDIM = 0x0D,
  RDDSM = 0x0E,
  SLPIN = 0x10,
  SLPOUT = 0x11,
  PTLON = 0x12,
  NORON = 0x13,
  INVOFF = 0x20,
  INVON = 0x21,
  GAMSET = 0x26,
  DISPOFF = 0x28,
  DISPON = 0x29,
  CASET = 0x2A,
  RASET = 0x2B,
  RAMWR = 0x2C,
  RAMRD = 0x2E,
  PTLAR = 0x30,
  TEOFF = 0x34,
  TEON = 0x35,
  MADCTL = 0x36,
  IDMOFF = 0x38,
  IDMON = 0x39,
  COLMOD = 0x3A,
  RDID1 = 0xDA,
  RDID2 = 0xDB,
  RDID3 = 0xDC
} ST77XX_REGISTER;

/**
 * @brief Parameters to configure specific settings of the ST77XX display
 *
 */
typedef enum ST7XX_PARAM {

} ST7XX_PARAM;

/**
 * @brief Resolutions of the ST77XX display
 *
 */
typedef enum ST7XX_RESOLUTION {

} ST7XX_RESOLUTION;

#endif /* __ST77XX_ENUM_ */
