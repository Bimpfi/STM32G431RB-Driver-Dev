/*
 * Neopixel.cpp
 *
 *  Created on: Dec 19, 2019
 *      Author: marku
 */

#include "neopixel.h"

Neopixel::Neopixel() : ledsSet{0} {
  for (int i = 0; i < Neopixel::NUM_OF_LEDS; ++i) {
    brightness[i] = 0xFFFFFF;
    leds[i] = 0;
  }
}

void Neopixel::init() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN_Msk;  // Port A aktivieren
  // Mode Register auf 01 (Output) setzen
  GPIOA->MODER |= GPIO_MODER_MODE0_0;     // Mode-Bit-0 auf 1 setzen
  GPIOA->MODER &= ~(GPIO_MODER_MODE0_1);  // Mode-Bit-1 auf 0 setzen
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT_0;  // Output type auf 0 (Push-Pull) setzen
  // Pull-Up/Down Register auf 00 (No Pull) setzen
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0_0);  // Pull-Bit-0 auf 0 setzen
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0_1);  // Pull-Bit-1 auf 0 setzen
  GPIOA->ODR &= ~GPIO_ODR_OD0_Msk;
  this->clear();
}

void Neopixel::setBrightness(uint8_t brightness, uint8_t pos) {
  if (pos > Neopixel::NUM_OF_LEDS || pos < 1) {
    return;
  }
  // brightness is 0x808080 shifted to the right and 0x808080 added
  uint32_t calcBright = 0x00;
  for (int i = 0; i < brightness && calcBright < 0xFFFFFF; ++i) {
    calcBright >>= 1;
    calcBright += 0x808080;
  }
  this->brightness[pos - 1] = calcBright;  // set brightness
}

void Neopixel::setBrightnessOfAll(uint8_t brightness) {
  uint32_t calcBright = 0x00;
  // brightness is 0x808080 shifted to the right and 0x808080 added
  for (int i = 0; i < brightness && calcBright < 0xFFFFFF; ++i) {
    calcBright >>= 1;
    calcBright += 0x808080;
  }
  for (int i = 0; i < Neopixel::NUM_OF_LEDS; ++i)
    this->brightness[i] = calcBright;  // set brightness of all
}
void Neopixel::set(uint32_t color, uint8_t pos) {
  if (pos > Neopixel::NUM_OF_LEDS || pos < 1) {
    return;
  }

  if (color != 0 && leds[pos - 1] == 0)  // when setting a color, check if LED
                                         // was set before, if not then count it
    ++ledsSet;
  else if (color == 0 &&
           leds[pos - 1] != 0)  // when clearing a color, check if LED was set
                                // before, if yes decrease counter
    --ledsSet;

  if (ledsSet > Neopixel::NUM_OF_ALLOWED_LEDS)  // if too many LEDs are active,
                                                // deactivate all
    clear();

  this->leds[pos - 1] = color;  // set color
}

void Neopixel::show() {
  __asm__ __volatile__("PUSH {R9}");
  __asm__ __volatile__("PUSH {R10}");
  __asm__ __volatile__("PUSH {R11}");
  __asm__ __volatile__("PUSH {R12}");
  __asm__ __volatile__("MOVW R10, #0x01");
  __asm__ __volatile__("MOVW R9, #0xFE");
  __asm__ __volatile__("LDR R11, =0x48000014");
  __asm__ __volatile__("LDR R12, [R11]");
  for (int i = 0; i < Neopixel::NUM_OF_LEDS; i++) {
    uint32_t tmp = this->leds[i] & this->brightness[i];

    for (int j = 0; j < 24; j++) {
      __asm__ __volatile__("ORR R12, R12, #0x01");
      __asm__ __volatile__("STR R12, [R11]");
      bool isOne = 1 & tmp;
      tmp >>= 1;
      //__asm__ __volatile__ ("nop");
      if (isOne) {
        __asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
        // __asm__ __volatile__("nop");
      }
      __asm__ __volatile__("BIC R12, R12, #0x01");
      __asm__ __volatile__("STR R12, [R11]");
      if (!isOne) {
        __asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
        // __asm__ __volatile__("nop");
        // __asm__ __volatile__("nop");
      }
    }
  }
  __asm__ __volatile__("BIC R12, R12, #0x01");
  __asm__ __volatile__("STR R12, [R11]");
  for (int k = 0; k < 150; k++) {
    __asm__ __volatile__("nop");
  }
  __asm__ __volatile__("POP {R12}");
  __asm__ __volatile__("POP {R11}");
  __asm__ __volatile__("POP {R10}");
  __asm__ __volatile__("POP {R9}");
}

void Neopixel::clear() {
  for (uint8_t i = 1; i <= Neopixel::NUM_OF_LEDS; i++) {
    if (leds[i - 1] != 0)  // clear LEDs if there are not cleared anyways
      this->set(0x00, i);
  }
}