#include "stm32f4xx_hal.h"

extern char left_direct_test_drive;
uint32_t    first_mesure_potentiometer  = 0;
uint32_t    second_mesure_potentiometer = 0;
double      first_mesure_angle          = 0.0;
double      second_mesure_angle         = 0.0;
double      mesure_angle = 0.0;

double map(double st1, double fn1, double st2, double fn2, uint32_t value)
{
   return ((value - st1) * 1.0) / ((fn1 - st1) * 1.0) * (fn2 - st2) * 1.0 + st2;
}


void set_first_mesure_adc(uint32_t first_mesure_adc)
{
   first_mesure_potentiometer = first_mesure_adc;
   first_mesure_angle         = map(0.0, 4095.0, 0.0, 360.0,
                                    first_mesure_potentiometer);
   second_mesure_angle = first_mesure_angle;
}


double  measure_angle(uint32_t potentiometer_value)
{
   second_mesure_potentiometer = potentiometer_value;
   second_mesure_angle         = map(0.0, 4095.0, 0.0, 360.0,
                                     second_mesure_potentiometer);

   if (left_direct_test_drive == 1)
   {
      if (first_mesure_angle < second_mesure_angle)
      {
         mesure_angle = second_mesure_angle - first_mesure_angle;
      }
      else
      {
         mesure_angle = first_mesure_angle - second_mesure_angle;
      }
   }
   else
   {
      if (second_mesure_angle < first_mesure_angle)
      {
         mesure_angle = first_mesure_angle - second_mesure_angle;
      }
      else
      {
         mesure_angle = second_mesure_angle - first_mesure_angle;
      }
   }
   return mesure_angle;
}
