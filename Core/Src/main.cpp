/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.cpp
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <cstring>
#include "i2c_master.h"
#include "mpu_6050.h"
#include "neopixel.h"
#include "spi_master.h"
#include "ssd1306.h"
#include "uart.h"

void SystemClock_Config(void);
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  // I2C_Master i2c_master;
  SSD1306<128, 64, 8, 0x3D, I2C_Master::write> display;
  UART uart;
  MPU_6050<MPU_6050_ADDR::ADD0_LOW, I2C_Master> mpu_6050;
  Neopixel neopixel;
  SPI_Master spi;
  /* Reset of all peripherals, Initializes the Flash interface and the
   * Systick.
   */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  LL_PWR_DisableUCPDDeadBattery();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN_Msk;
  GPIOB->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER5_0;
  GPIOB->MODER &= ~(GPIO_MODER_MODER0_1 | GPIO_MODER_MODER5_1);
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT5);

  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR5_0);
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR5_1);

  GPIOB->ODR |= GPIO_ODR_OD0_Msk;
  LL_mDelay(200);
  GPIOB->ODR &= ~GPIO_ODR_OD0_Msk;
  LL_mDelay(200);
  GPIOB->ODR |= GPIO_ODR_OD0_Msk;
  LL_mDelay(200);

  // i2c_master.init();
  // uart.init();
  // I2C_Master::init();
  // mpu_6050.init();
  // display.init();
  spi.init();
  neopixel.init();
  neopixel.clear();

  float accX = 0.0f;
  float accY = 0.0f;
  float accZ = 0.0f;

  uint16_t pixel = 0;
  GPIOB->ODR = ~(GPIO_ODR_OD5_Msk);
  uint8_t init_data[] = {0x01, 0x11, 0x3A, 0x05, 0x13, 0x29, 0x36, 0x60};
  spi.write(init_data, 1);
  LL_mDelay(150);
  spi.write(&init_data[1], 1);
  LL_mDelay(500);
  spi.write(&init_data[2], 1);
  GPIOB->ODR |= GPIO_ODR_OD5_Msk;
  spi.write(&init_data[3], 1);
  GPIOB->ODR = ~(GPIO_ODR_OD5_Msk);
  LL_mDelay(150);
  spi.write(&init_data[4], 1);
  LL_mDelay(150);
  spi.write(&init_data[5], 1);
  LL_mDelay(150);
  spi.write(&init_data[6], 1);
  GPIOB->ODR |= GPIO_ODR_OD5_Msk;
  spi.write(&init_data[7], 1);

  uint8_t red = 0xff;
  uint8_t green = 0x00;
  uint8_t blue = 0x00;

  while (1) {
    uint8_t init_row[] = {0x2A, 0x00, 0x00, 0x00, 0xA0};
    uint8_t init_col[] = {0x2B, 0x00, 0x00, 0x00, 0x7F};
    uint8_t init_data[] = {0x2C};
    GPIOB->ODR = ~(GPIO_ODR_OD5_Msk);
    spi.write(init_row, 1);
    GPIOB->ODR |= GPIO_ODR_OD5_Msk;
    spi.write(&init_row[1], 4);
    GPIOB->ODR = ~(GPIO_ODR_OD5_Msk);
    spi.write(init_col, 1);
    GPIOB->ODR |= GPIO_ODR_OD5_Msk;
    spi.write(&init_col[1], 4);
    GPIOB->ODR = ~(GPIO_ODR_OD5_Msk);
    spi.write(&init_data[0], 1);

    volatile uint8_t data[2] = {0x00};
    GPIOB->ODR |= GPIO_ODR_OD5_Msk;
    for (uint32_t i = 0; i < 20480; i++) {
      // red += 4;
      // if (red == 0x00) {
      //   blue += 4;
      // }

      // if (red == 0x00 && blue == 0) {
      //   green += 4;
      // }

      data[0] = ((red << 6) & 0xFF) | (green & 0x3F);
      data[1] = ((blue << 2) & 0xFC) | (red & 0x03);
      spi.write(const_cast<uint8_t*>(data), sizeof(data) / sizeof(uint8_t));
    }
    if (red == 0xff) {
      green ^= 0xff;
      red ^= 0xff;
    } else if (green == 0xff) {
      green ^= 0xff;
      blue ^= 0xff;
    } else {
      blue ^= 0xff;
      red ^= 0xff;
    }

    LL_mDelay(1);
    // neopixel.clear();
    // neopixel.set(neoColor::BLUE, pixel++ % 9);
    // neopixel.show();
    // LL_mDelay(100);
    // display.setPixel(pixel % 128, (pixel / 128) % 64, 1);
    // // display.printSymbol('a');
    // display.showDisplay();
    // LL_mDelay(1);
    // pixel++;
    // // display.clearDisplay();
    // // display.showDisplay();
    // //  display.clearDisplay();
    // //  display.showDisplay();
    // uint8_t ret[14] = {0x00};
    // ret[12] = 0x0D;
    // ret[13] = 0x0A;
    // uint8_t ret_zero[2] = {0x0D, 0x0A};
    // mpu_6050.measure();
    // for (uint8_t i = 0; i < 15; i++) {
    //   MPU_6050_Measurement vals = mpu_6050.getMeasurement();
    //   accX = accX * 0.2f + vals.acc_data.calculated.x * 0.8f;
    //   accY = accY * 0.2f + vals.acc_data.calculated.y * 0.8f;
    //   accZ = accZ * 0.2f + vals.acc_data.calculated.z * 0.8f;
    // }

    // if (accX > 1.6f) {
    //   GPIOA->ODR ^= LL_GPIO_PIN_5;
    //   LL_mDelay(1000);
    //   GPIOA->ODR ^= LL_GPIO_PIN_5;
    //   LL_mDelay(1000);
    // }
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0) {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();
  /* Wait till HSI is ready */
  while (LL_RCC_HSI_IsReady() != 1) {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(16000000);

  LL_SetSystemCoreClock(16000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
