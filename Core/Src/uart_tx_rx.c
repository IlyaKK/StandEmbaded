#include "PIDRegulating.h"
#include "PIDRegulatingMoment.h"
#include "calculateReferenceMoment.h"
#include "harmonic_signal.h"
#include "stm32f4xx_hal.h"
#include "cJSON.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart2;

uint8_t command[3];

cJSON *json               = NULL;
cJSON *jsonImitatorData   = NULL;
cJSON *jsonImitatorDataAdvance   = NULL;
cJSON *KpJson             = NULL;
cJSON *KiJson             = NULL;
cJSON *KdJson             = NULL;
cJSON *kpImitatorJson     = NULL;
cJSON *kiImitatorJson     = NULL;
cJSON *kdImitatorJson     = NULL;
cJSON *angleJson          = NULL;
cJSON *amplitudeSinusJson = NULL;
cJSON *frequencySinusJson = NULL;
cJSON *j_nJson            = NULL;
cJSON *k_vJson           = NULL;
cJSON *k_shJson           = NULL;
cJSON *j_dv_imJson        = NULL;
cJSON *q_imJson           = NULL;
cJSON *ce_imJson          = NULL;
cJSON *loadSelectedJson   = NULL;

uint8_t string_imitator_advance[70];
char ght[2];

uint8_t size_string_imimtator[2];
uint8_t string_imitator[70];

uint8_t size_string_pid[2];
uint8_t string_pid[70];

uint8_t size_string_sinus_pid[2];
uint8_t string_sinus_pid[70];

char start_PID    = 0;
char start_sinus  = 0;
char start_Moment = 0;

double time_Uart        = 0;
double angle_Uart       = 0;
double measure_Moment_Uart = 0;
double reference_Moment_Uart = 0;
double sinus_Angle_Uart = 0;

char stringJson[75];
char stringOfSizeStringJson[2];
int  sizeStringJson;

int sizeStringImitator = 0;
int sizeStringImitatorParams = 0;
enum UartReceiveState
{
   ReceiveCommand,
   ReceiveSizePidData,
   ReceivePidData,
   ReceiveSizeSinusPidData,
   ReceiveSinusPidData,
   ReceiveSizeImitatorData,
   ReceiveImitatorData,
   ReceiveSizeImitatorDataAdvance,
   ReceiveImitatorDataAdvance
};

enum UartTransmitState
{
   TransmitCommand, TransmitData
};

enum UartReceiveState  uart_receive_state  = ReceiveCommand;
enum UartTransmitState uart_transmit_state = TransmitCommand;

void setImpulseUartData(double time, double angle, double reference_moment)
{
   time_Uart  = time;
   angle_Uart = angle;
   reference_Moment_Uart = reference_moment;
}

void setMeasureMomentUart(double measure_moment)
{
  measure_Moment_Uart = measure_moment;
}

void setSinusUartData(double time, double mesureAngle, double sinusAngle)
{
   time_Uart        = time;
   angle_Uart       = mesureAngle;
   sinus_Angle_Uart = sinusAngle;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart == &huart2)
   {
      switch (uart_receive_state)
      {
      case ReceiveCommand:
         if (command[1] == '7')
         {
            memset(&command, 0, 3);
            start_PID    = 0;
            start_sinus  = 0;
            start_Moment = 0;
            HAL_UART_Receive_IT(&huart2, (uint8_t *)command, 2);
            break;
         }

         if (command[0] == '1')
         {
           uart_receive_state = ReceiveSizePidData;
            HAL_UART_Receive_IT(&huart2, (uint8_t *)size_string_pid, 2);
            HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Send size pid", 13);
            break;
         }

         if (command[0] == '4')
         {
            HAL_UART_Receive_IT(&huart2, (uint8_t *)size_string_sinus_pid,
                                3);
            HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Send size sinus pid",
                                 19);
            uart_receive_state = ReceiveSizeSinusPidData;
            break;
         }

         if (command[0] == '2')
         {
            memset(&stringJson, 0, 75);
            memset(&stringOfSizeStringJson, 0, 2);
            sprintf(stringJson,
            "{\"time\":%.2f,\"angle\":%.2f, \"measure_moment\":%.2f, \"reference_moment\":%.2f}",
                    time_Uart, angle_Uart, measure_Moment_Uart, reference_Moment_Uart);
            sizeStringJson = strlen(stringJson);
            sprintf(stringOfSizeStringJson, "%d", sizeStringJson);
            memset(&command, 0, 3);
            HAL_UART_Receive_IT(&huart2, (uint8_t *)command, 2);
            HAL_UART_Transmit_IT(&huart2, (uint8_t *)stringOfSizeStringJson,
                                 sizeof(stringOfSizeStringJson));
            break;
         }

         if (command[0] == '5')
         {
            memset(&stringJson, 0, 70);
            memset(&stringOfSizeStringJson, 0, 2);
            sprintf(stringJson,
                    "{\r\n  \"time\" : %.2f,\r\n  \"angle\" : %.2f,\r\n  \"sinusAngle\" : %.2f\r\n}",
                    time_Uart, angle_Uart, sinus_Angle_Uart);
            sizeStringJson = strlen(stringJson);
            sprintf(stringOfSizeStringJson, "%d", sizeStringJson);
            memset(&command, 0, 3);
            HAL_UART_Receive_IT(&huart2, (uint8_t *)command, 2);
            HAL_UART_Transmit_IT(&huart2, (uint8_t *)stringOfSizeStringJson,
                                 sizeof(stringOfSizeStringJson));
            break;
         }

         if (command[0] == '3')
         {
            memset(&command, 0, 3);
            HAL_UART_Receive_IT(&huart2, (uint8_t *)command, 2);
            HAL_UART_Transmit_IT(&huart2, (uint8_t *)stringJson,
                                 strlen(stringJson));
            break;
         }

         if (command[0] == '6')
         {
            memset(&command, 0, 3);
            HAL_UART_Receive_IT(&huart2, (uint8_t *)command, 2);
            HAL_UART_Transmit_IT(&huart2, (uint8_t *)stringJson,
                                 strlen(stringJson));
            break;
         }

      case ReceiveSizePidData:
        uart_receive_state = ReceivePidData;
         HAL_UART_Receive_IT(&huart2, (uint8_t *)string_pid,
                             atoi((char *)size_string_pid));
         HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Send pid", 8);
         break;

      case ReceiveSizeSinusPidData:
         HAL_UART_Receive_IT(&huart2, (uint8_t *)string_sinus_pid,
                             atoi((char *)size_string_sinus_pid));
         HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Send sinus pid", 14);
         uart_receive_state = ReceiveSinusPidData;
         break;

      case ReceivePidData:
         json             = cJSON_Parse((char *)string_pid);
         KpJson           = cJSON_GetObjectItem(json, "kP");
         KiJson           = cJSON_GetObjectItem(json, "kI");
         KdJson           = cJSON_GetObjectItem(json, "kD");
         angleJson        = cJSON_GetObjectItemCaseSensitive(json, "angle");
         loadSelectedJson = cJSON_GetObjectItemCaseSensitive(json,
                                                             "loadSelected");
         setPIDCoefficient(KpJson->valuedouble, KiJson->valuedouble,
                           KdJson->valuedouble, angleJson->valuedouble);
         cJSON_Delete(json);
         cJSON_Delete(KpJson);
         cJSON_Delete(KiJson);
         cJSON_Delete(KdJson);
         cJSON_Delete(angleJson);
         memset(&size_string_pid, 0, 2);
         memset(&string_pid, 0, 70);
         memset(&command, 0, 3);
         if (loadSelectedJson->valueint == 1)
         {
            uart_receive_state = ReceiveSizeImitatorData;
            HAL_UART_Receive_IT(&huart2, (uint8_t *)size_string_imimtator, 2);
            HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Send size imitator", 18);
            break;
         }
         start_PID = 1;
         HAL_UART_Receive_IT(&huart2, (uint8_t *)command, 2);
         HAL_UART_Transmit_IT(&huart2, (uint8_t *)"End Transmit", 12);
         uart_receive_state = ReceiveCommand;
         break;

      case ReceiveSinusPidData:
         json = cJSON_ParseWithLength((char *)string_sinus_pid,
                                      atoi((char *)size_string_sinus_pid));
         KpJson             = cJSON_GetObjectItemCaseSensitive(json, "kp");
         KiJson             = cJSON_GetObjectItemCaseSensitive(json, "ki");
         KdJson             = cJSON_GetObjectItemCaseSensitive(json, "kd");
         amplitudeSinusJson = cJSON_GetObjectItemCaseSensitive(json,
                                                               "amplitudeSinus");
         frequencySinusJson = cJSON_GetObjectItemCaseSensitive(json,
                                                               "frequencySinus");
         setPIDCoefficient(KpJson->valuedouble, KiJson->valuedouble,
                           KdJson->valuedouble, 0.0);
         setSinusCoefficient(amplitudeSinusJson->valuedouble,
                             frequencySinusJson->valuedouble);
         start_sinus = 1;
         cJSON_Delete(json);
         cJSON_Delete(KpJson);
         cJSON_Delete(KiJson);
         cJSON_Delete(KdJson);
         cJSON_Delete(amplitudeSinusJson);
         cJSON_Delete(frequencySinusJson);
         memset(&size_string_sinus_pid, 0, 2);
         memset(&string_sinus_pid, 0, 60);
         memset(&command, 0, 3);
         HAL_UART_Receive_IT(&huart2, (uint8_t *)command, 2);
         uart_receive_state = ReceiveCommand;
         HAL_UART_Transmit_IT(&huart2, (uint8_t *)"End Transmit sinus", 18);
         break;

      case ReceiveSizeImitatorData:
         uart_receive_state = ReceiveImitatorData;
         sizeStringImitator = atoi((char *)size_string_imimtator);
         HAL_UART_Receive_IT(&huart2, (uint8_t*)string_imitator,
                             sizeStringImitator);
         HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Send imitator", 13);
         break;

      case ReceiveImitatorData:
         uart_receive_state = ReceiveSizeImitatorDataAdvance;
         memset(&size_string_imimtator,0,2); 
         HAL_UART_Receive_IT(&huart2, (uint8_t *)ght, 2);
         HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Send size imitator advance", 26);
         break;
     
      case ReceiveSizeImitatorDataAdvance:
        uart_receive_state = ReceiveImitatorDataAdvance;
        sizeStringImitatorParams = atoi(ght);
        HAL_UART_Receive_IT(&huart2, (uint8_t*)string_imitator_advance, sizeStringImitatorParams);
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Send imitator advance", 21);
        break;
        
      case ReceiveImitatorDataAdvance:
        jsonImitatorData = cJSON_Parse((char *)string_imitator);
        kpImitatorJson   = cJSON_GetObjectItemCaseSensitive(jsonImitatorData,
                                                             "kp");
         kiImitatorJson = cJSON_GetObjectItemCaseSensitive(jsonImitatorData,
                                                           "ki");
         kdImitatorJson = cJSON_GetObjectItemCaseSensitive(jsonImitatorData,
                                                           "kd");
         j_nJson     = cJSON_GetObjectItemCaseSensitive(jsonImitatorData, "jN");
         k_vJson    = cJSON_GetObjectItem(jsonImitatorData, "kv");
         setPIDMomentCoefficient(kpImitatorJson->valuedouble,
                                 kiImitatorJson->valuedouble,
                                 kdImitatorJson->valuedouble);
         
         memset(&string_imitator, 0, 50);
         memset(&size_string_imimtator, 0, 2);
         cJSON_Delete(jsonImitatorData);
         cJSON_Delete(kpImitatorJson);
         cJSON_Delete(kiImitatorJson);
         cJSON_Delete(kdImitatorJson);
         cJSON_Delete(j_nJson);
        jsonImitatorDataAdvance = cJSON_Parse((char*)string_imitator_advance);
         k_shJson    = cJSON_GetObjectItemCaseSensitive(jsonImitatorDataAdvance, "kh");
         j_dv_imJson = cJSON_GetObjectItemCaseSensitive(jsonImitatorDataAdvance, "jDv");
         q_imJson    = cJSON_GetObjectItemCaseSensitive(jsonImitatorDataAdvance, "q");
         ce_imJson   = cJSON_GetObjectItemCaseSensitive(jsonImitatorDataAdvance, "cE");
         
         setCoefficientReferenceMoment(j_nJson->valuedouble,
                                       k_vJson->valuedouble, k_shJson->valuedouble,
                                       j_dv_imJson->valuedouble, q_imJson->valuedouble,
                                       ce_imJson->valuedouble);
         start_PID    = 1;
         start_Moment = 1;
         memset(&string_imitator_advance, 0, 60);
         cJSON_Delete(jsonImitatorDataAdvance);
         cJSON_Delete(k_shJson);
         cJSON_Delete(k_vJson);
         cJSON_Delete(j_dv_imJson);
         cJSON_Delete(q_imJson);
         cJSON_Delete(ce_imJson);
         HAL_UART_Receive_IT(&huart2, (uint8_t *)command, 2);
         HAL_UART_Transmit_IT(&huart2, (uint8_t *)"End Transmit", 12);
         uart_receive_state = ReceiveCommand;
         break;
      }
   }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart == &huart2)
   {
   }
}
