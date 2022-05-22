/*
 * Neopixel.h
 *
 *  Created on: Dec 19, 2019
 *      Author: marku
 */

#ifndef NEOPIXEL_H_
#define NEOPIXEL_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32g4xx.h"
#ifdef __cplusplus
}
#endif

#include <cstdint>

namespace neoColor {
enum COLOR {
  RED = 0x0000FF00,
  ORANGE = 0x0000FF5A,
  YELLOW = 0x0000FF7E,
  GREEN = 0x000000FF,
  BLUE = 0x00FF00FB,
  DARKBLUE = 0x00DC0000,
  PURPLE = 0x00FF5003,
  PINK = 0x0052FF8E,
  BLACK = 0x00000000,
};
}  // namespace neoColor

class Neopixel {
 private:
  static constexpr uint8_t NUM_OF_LEDS = 8;
  static constexpr uint8_t NUM_OF_ALLOWED_LEDS = 4;
  uint8_t ledsSet;
  uint32_t leds[NUM_OF_LEDS];
  uint32_t brightness[NUM_OF_LEDS];

 public:
  explicit Neopixel();

  /** Initializes LED */
  void init();

  /**
   * sets one LED to a specific color (only 4 LED at the same time)
   * @param color which color in hex BBRRGG (from left to right)
   * @param pos which LED (goes from 1 to 8)
   */
  void set(uint32_t color, uint8_t pos);

  /**
   * sets brightness of one LED
   * @param brightness brightness in hex, darkest is 1 and brightest is 8
   * @param pos which LED (goes from 1 to 8)
   */
  void setBrightness(uint8_t brightness, uint8_t pos);

  /**
   * sets brightness of every LED
   * @param brightness brightness in hex, darkest is 1 and brightest is 8
   */
  void setBrightnessOfAll(uint8_t brightness);

  /**
   * shows LEDs which are set
   */
  void __attribute__((optimize("Ofast"))) show();

  /**
   * clears all LEDs to 0 (only color, not brightness)
   */
  void clear();
};

#endif /* NEOPIXEL_H_ */