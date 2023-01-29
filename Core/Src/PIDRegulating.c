#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

double iteration_time         = 0.01;
char   left_direct_test_drive = 1;
char   PID_state       = 1;
double Kp              = 0;
double Ki              = 0;
double Kd              = 0;
double reference_angle = 0;
double function        = 0;
double I = 0;
double D = 0;
double prev_mesure_angle = 0;

void setPIDCoefficient(double KpUser, double KiUser, double KdUser,
                       double angleUser)
{
   Kp = KpUser;
   Ki = KiUser;
   Kd = KdUser;
   reference_angle = angleUser;
   function        = 0.;
   I = 0.;
   prev_mesure_angle = 0.;
}


void setReferenceAngle(double angleUser)
{
   reference_angle = angleUser;
}


void Set_Duty(double function, double error)
{
   static char direction = 1;

   if (function < 0)
   {
      TIM4->CCR4 = (uint32_t)(-1.0 * function);
   }
   else
   {
      TIM4->CCR4 = (uint32_t)function;
   }

   if ((error < 0) && (direction == 1))
   {
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_10);
      direction = 0;
      left_direct_test_drive = 0;
   }
   else if ((error > 0) && (direction == 0))
   {
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_10);
      direction = 1;
      left_direct_test_drive = 1;
   }
}


double Get_Error(double mesure_angle)
{
   double error = reference_angle - mesure_angle;

   return error;
}


void PIDregulating(double mesure_angle)
{
   double error = 0.0;

   error = Get_Error(mesure_angle);

   if ((error >= -0.8) && (error <= 0.8))
   {
      error    = 0;
      function = 0;
   }

   function = Kp * error;

   if (Ki != 0)
   {
      I        += error * iteration_time;
      function += Ki * I;
   }

   if (Kd != 0)
   {
      D                 = -(mesure_angle - prev_mesure_angle) / iteration_time;
      function         += Kd * D;
      prev_mesure_angle = mesure_angle;
   }

   if (function >= 499.0)
   {
      function = 499.0;
   }
   else if (function <= -499.0)
   {
      function = -499.0;
   }

   Set_Duty(function, error);
}
