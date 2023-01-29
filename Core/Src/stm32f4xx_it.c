/* USER CODE BEGIN Header */

/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "measuring_angle.h"
#include "PIDRegulating.h"
#include "uart_tx_rx.h"
#include "harmonic_signal.h"
#include "PIDRegulatingMoment.h"
#include "calculateReferenceMoment.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
double lcd_mesure_angle = 0;
double t = 0;

extern void I2C_send(uint8_t data, uint8_t flags);
extern void LCD_SendString(char *str);

extern char start_PID;
extern char start_sinus;
extern char start_Moment;

unsigned int direction = 1;

char     trans_str2[64] = { 0, };
uint32_t adc2           = 0;
double   Idv;
double   Cm                  = 0.082;
double   M                   = 0;
double   reference_M         = 0;
int32_t  previousCounter     = 0;
int32_t  currentCounter      = 0;

double   currentAngle        = 0;
double   previousAngle        = 0;
double   previousSpeedAngle  = 0;
double   currentSpeedAngle   = 0;
double   currentAcceleration = 0;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
   {
   }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */
   HAL_ADC_Start(&hadc2);
   if (start_Moment == 1)
   {
      adc2 = HAL_ADC_GetValue(&hadc2);
      M = (double)adc2/4095.0;
      setMeasureMomentUart(M);
      setMeasureMoment(M);
      PIDregulatingMoment();
   }
  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
   char        str_send_lcd_row_one[12];
   char        str_send_lcd_row_two[16];
   static char first_call = 1;

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

   if (first_call)
   {
    //  I2C_send(0b00110000, 0);
  //    I2C_send(0b00000010, 0);
    //  I2C_send(0b00001100, 0);
    //  I2C_send(0b00000001, 0);
      first_call = 0;
   }
  // I2C_send(0b10000000, 0);
   sprintf(str_send_lcd_row_one, "t = %.2f   ", t);
   LCD_SendString(str_send_lcd_row_one);
   //I2C_send(0b11000000, 0);
   sprintf(str_send_lcd_row_two, "angle = %.2f   ", lcd_mesure_angle);
   LCD_SendString(str_send_lcd_row_two);

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
   //currentCounter = TIM1->CNT;
   /*if (start_Moment == 1)
   {
      currentAngle = currentCounter * 0.0052;

      if (previousCounter > currentCounter)
      {
         currentSpeedAngle = ((previousCounter - currentCounter) * 0.0052)
                             / 0.1;
      }
      else
      {
         currentSpeedAngle = ((currentCounter - previousCounter) * 0.0052)
                             / 0.1;
      }

      if (currentSpeedAngle > previousSpeedAngle)
      {
         currentAcceleration = (currentSpeedAngle - previousSpeedAngle)
                               / 0.1;
      }
      else
      {
         currentAcceleration = (previousSpeedAngle - currentSpeedAngle)
                               / 0.1;
      }
      reference_M = calculateReferenceMoment(currentAngle, currentSpeedAngle,
                                             currentAcceleration);
      setReferenceMoment(reference_M);
      previousCounter    = currentCounter;
      previousSpeedAngle = currentSpeedAngle;
   }*/

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
   HAL_ADC_Start(&hadc1);
   if (start_PID == 1)
   {
      double measured_angle = measure_angle(HAL_ADC_GetValue(&hadc1));
      if (start_Moment == 1)
   {
      currentAngle = measured_angle*0.00425;

      if (previousAngle > currentAngle)
      {
         currentSpeedAngle = (previousAngle - currentAngle) / 0.01;
      }
      else
      {
         currentSpeedAngle = (currentAngle - previousAngle) / 0.01;
      }
      currentAcceleration = (currentSpeedAngle - previousSpeedAngle) / 0.01;
      reference_M = calculateReferenceMoment(currentAngle, currentSpeedAngle,
                                             currentAcceleration);
      setReferenceMoment(reference_M);
      previousAngle = currentAngle;
      previousSpeedAngle = currentSpeedAngle;
   }
      PIDregulating(measured_angle);
      lcd_mesure_angle = measured_angle;
      t += 0.01;
      setImpulseUartData(t, measured_angle, reference_M);
   }

   if (start_sinus == 1)
   {
      double sinus_angle    = calculateValueHarmonicSignal();
      double measured_angle = measure_angle(HAL_ADC_GetValue(&hadc1));
      setReferenceAngle(sinus_angle);
      PIDregulating(measured_angle);
      lcd_mesure_angle = measured_angle;
      setSinusUartData(t, measured_angle, sinus_angle);
      t += 0.01;
   }

   if ((start_PID != 1) && (start_sinus != 1))
   {
      set_first_mesure_adc(HAL_ADC_GetValue(&hadc1));
      t = 0;
   }
  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

