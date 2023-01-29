/*
 * PIDRegulatingMoment.c
 *
 *  Created on: Jan 26, 2022
 *      Author: ilya
 */
#include "stm32f4xx_hal.h"

extern char left_direct_test_drive;

double iteration_time_moment     = 0.001;
char   right_direct_moment_drive = 1;
double Kp_moment           = 0;
double Ki_moment           = 0;
double Kd_moment           = 0;
double reference_moment    = 0;
double measure_moment      = 0;
double function_moment     = 0;
double I_moment            = 0;
double D_moment            = 0;
double prev_measure_moment = 0;

void setPIDMomentCoefficient(double KpUser, double KiUser, double KdUser)
{
   Kp_moment           = KpUser;
   Ki_moment           = KiUser;
   Kd_moment           = KdUser;
   I_moment            = 0;
   prev_measure_moment = 0;
}


void setReferenceMoment(double moment)
{
   reference_moment = moment;
}


void setMeasureMoment(double moment)
{
   measure_moment = moment;
}


void Set_Duty_Moment(double function, double error)
{
   static char direction = 1;

   if (function < 0)
   {
      TIM4->CCR3 = (uint32_t)(-1.0 * function);
   }
   else
   {
      TIM4->CCR3 = (uint32_t)function;
   }

   if ((left_direct_test_drive == 1) && (direction == 1))
   {
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
      direction = 0;
      right_direct_moment_drive = 0;
   }
   else if ((left_direct_test_drive == 0) && (direction == 0))
   {
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
      direction = 1;
      right_direct_moment_drive = 1;
   }
}


double Get_Error_Moment()
{
   double error = reference_moment - measure_moment;

   return error;
}


void PIDregulatingMoment()
{
   double error = 0.0;

   error = Get_Error_Moment();

   if ((error >= -0.8) && (error <= 0.8))
   {
      error           = 0;
      function_moment = 0;
   }

   function_moment = Kp_moment * error;

   if (Ki_moment != 0)
   {
      I_moment        += error * iteration_time_moment;
      function_moment += Ki_moment * I_moment;
   }

   if (Kd_moment != 0)
   {
      D_moment            = -(measure_moment - prev_measure_moment) / iteration_time_moment;
      function_moment    += Kd_moment * D_moment;
      prev_measure_moment = measure_moment;
   }

   if (function_moment >= 499.0)
   {
      function_moment = 499.0;
   }
   else if (function_moment <= -499.0)
   {
      function_moment = -499.0;
   }

   Set_Duty_Moment(function_moment, error);
}
