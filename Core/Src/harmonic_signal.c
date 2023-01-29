#include "math.h"

double sinus_angle;
double angle      = 0;
double step_angle = 0;
double sinus      = 0;
double amplitude  = 0;
double frequency  = 0;

const float  PI                      = 3.1416;
const double GEAR_RATIO              = 2.0;
const double ZERO_LEVEL_SINE_WAVE    = 50.0;
const double FREQUENCY_HANDLER_TIMER = 100.0;
const double CIRCLE_ANGLE            = 360.0;

void setSinusCoefficient(double amplitudeSinus, double frequencySinus)
{
   angle      = 0;
   amplitude  = amplitudeSinus;
   frequency  = frequencySinus;
   step_angle = CIRCLE_ANGLE / (FREQUENCY_HANDLER_TIMER / frequency);
}


double calculateValueHarmonicSignal()
{
   sinus       = sin((PI * angle) / 180.0);
   angle       = angle + step_angle;
   sinus_angle = GEAR_RATIO * (ZERO_LEVEL_SINE_WAVE + amplitude * sinus);
   if (angle > 359)
   {
      angle = 0;
   }
   return sinus_angle;
}
