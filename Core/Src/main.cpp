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
#include "st77xx.h"
#include "uart.h"

void SystemClock_Config(void);
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  // I2C_Master i2c_master;
  SSD1306<128, 64, 8, 0x3D, I2C_Master::write> display;
  ST77XX<SPI_Master::write> st77xx;
  UART uart;
  MPU_6050<MPU_6050_ADDR::ADD0_LOW, I2C_Master> mpu_6050;
  Neopixel neopixel;
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

  // i2c_master.init();
  // uart.init();
  // I2C_Master::init();
  // mpu_6050.init();
  // display.init();
  SPI_Master::init();
  neopixel.init();
  neopixel.clear();

  float accX = 0.0f;
  float accY = 0.0f;
  float accZ = 0.0f;

  uint16_t pixel = 0;

  st77xx.init();
  st77xx.fillScreen();
  while (1) {
    st77xx.showTestImage();
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
