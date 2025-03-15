/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
// #include "ble_commands.h"
#include "ble.h"

#include <stdlib.h>

#include <stdint.h>
#include <stdio.h>
#include "math.h"

/* Include memory map of our MCU */
#include <stm32l475xx.h>

/* Include LED driver */
#include "leds.h"

#include "timer.h"
#include "i2c.h"
#include "lsm6dsl.h"
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// 10011001
#define PREAMBLE_STUDENT_ID 0b100110010001000010001000
#define SCALE 0.488
#define TAGNAME "Tracker"
#define BLE_MAX_PAYLOAD 20
volatile uint8_t student_id_bit_index = 0;
volatile uint8_t led_pair = 0;

volatile uint8_t update_leds = 0;
volatile uint32_t still_count = 0; // count number of timer interrupts called to calculate time
volatile int16_t prev_x = 0, prev_y = 0, prev_z = 0;
volatile uint8_t still = 0;
volatile uint32_t lastBleMsgTime = 0;
volatile uint8_t nonDiscoverable = 0;
volatile uint8_t check_lost = 0;
uint32_t prevTime = 0;

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
void SetSystemClock(uint8_t speed);

void detectLost()
{
  int16_t x, y, z;
  lsm6dsl_read_xyz(&x, &y, &z);
  x = (int16_t)(x * SCALE);
  y = (int16_t)(y * SCALE);
  z = (int16_t)(z * SCALE);

  printf("X: %d, Y: %d, Z: %d\n", x, y, z);

  if ((abs(x - prev_x) < 2000) &&
      (abs(y - prev_y) < 2000) &&
      (abs(z - prev_z) < 2000))
  {
    still = 1;
  }
  else
  {
    still = 0;
    still_count = 0; // reset still counter moving now
    lastBleMsgTime = 0;
    student_id_bit_index = 0;
    disconnectBLE();
    setDiscoverability(0);
    nonDiscoverable = 1;
    SetSystemClock(2);
    timer_set_ms(TIM2, 500);
  }

  prev_x = x;
  prev_y = y;
  prev_z = z;
}

void handle_lost_mode_leds()
{
  if (still)
  {
    // if it is still increment number of ticks it has been still

    // if no movement for 1 minute enter lost mode
    if (still_count >= 120) // 1 tick = 500 ms and 60000 ms = 1m so 60000/50 = 1200 ticks (50 ticks for testing purposes)
    {
      SetSystemClock(8);
      timer_set_ms(TIM2, 500);
      if (nonDiscoverable == 1) {
    	  setDiscoverability(nonDiscoverable);
    	  nonDiscoverable = 0;
      }
      if (((still_count * 500) / 1000 - lastBleMsgTime) >= 10)
      {
        lastBleMsgTime = (still_count * 500) / 1000;

        uint32_t lostSeconds = ((still_count - 120)* 500) / 1000; // each count is 50 ms so ticks * 50 ms to get ms -> divide by 1000 ms to get seconds
        char msg[20]; // char buffer for the output string
        sprintf(msg, "%s: %lu secs", TAGNAME, lostSeconds); // populate string with lost seconds
        updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(msg), msg);
      }
    }
    // then means movement within 1 minute
    else
    {
      // not considered lost if less than 1 min
      disconnectBLE();
      setDiscoverability(0);
      nonDiscoverable = 1;
      SetSystemClock(2);
      timer_set_ms(TIM2, 500);
    }
  }
}

void TIM2_IRQHandler(void)
{
  if (TIM2->SR & TIM_SR_UIF)
  {
    TIM2->SR &= ~TIM_SR_UIF;

    check_lost = 1;
    if (still) {
    	still_count++;
    }
  }
}

int _write(int file, char *ptr, int len)
{
  int i = 0;
  for (i = 0; i < len; i++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();

  // RESET BLE MODULE
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  ble_init();
  // ensure that ble is disconnected and it is not discoverable
  disconnectBLE();
  setDiscoverability(nonDiscoverable);
  nonDiscoverable = 1;

  HAL_Delay(10);

  // accelerometer and timer inits
  timer_init(TIM2);
  i2c_init();
  lsm6dsl_init();
  timer_set_ms(TIM2, 500);
  timer_reset(TIM2);

  int16_t x, y, z;
  lsm6dsl_read_xyz(&x, &y, &z);
  printf("X: %d, Y: %d, Z: %d", x, y, z);
  x = (int16_t)(x * SCALE);
  y = (int16_t)(y * SCALE);
  z = (int16_t)(z * SCALE);

  prev_x = x;
  prev_y = y;
  prev_z = z;

  while (1)
  {
    if (!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port, BLE_INT_Pin))
    {
      catchBLE();
    }
    
    detectLost();
    if (check_lost == 1)
    { // Each tick is 50ms
      // before handling lost mode, set lastBleMsgTime to current time
      handle_lost_mode_leds();
      check_lost = 0;
    }

//    leds_set(0b10);
//    __HAL_RCC_PWR_CLK_ENABLE();
//
//    PWR->CR1 &= ~PWR_CR1_LPMS;
//    PWR->CR1 |= PWR_CR1_LPMS_STOP1;
//    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
//    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    PWR->CR1 |= PWR_CR1_LPR;
    // Wait for interrupt, only uncomment if low power is needed
//    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    HAL_SuspendTick();
//    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    __WFI();
    HAL_ResumeTick();
//    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
//    leds_set(0b01);

    // optionally reconfigure clocks
    SystemClock_Config();
  }
}

/**
 * @brief System Clock Configuration
 * @attention This changes the System clock frequency, make sure you reflect that change in your timer
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  // This lines changes system clock frequency
  RCC_OscInitStruct.MSIClockRange = RCC_CR_MSIRANGE_7; // 1Mhz
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin | BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

void SetSystemClock(uint8_t speed)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

   /** Initializes the RCC Oscillators according to the specified parameters
 * in the RCC_OscInitTypeDef structure.
 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;

  if (speed == 8) {
    RCC_OscInitStruct.MSIClockRange = RCC_CR_MSIRANGE_7; // 8 MHz
  } else {
    RCC_OscInitStruct.MSIClockRange = RCC_CR_MSIRANGE_2; // 131 KHz
  }

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
	Error_Handler();
  }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
