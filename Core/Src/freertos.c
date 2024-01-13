/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "main.h"
#include "usb_host.h"
#include "fatfs.h"
#include "xProFIFO.h"
#include "led.h"
#include "structs.h"
#include "can.h"
#include "can_signals.h"
#include "can_signals_db.h"
#include <stdio.h>
#include <string.h>

#define CAN_LOOPBACK() HAL_GPIO_WritePin(CAN1_LBK_GPIO_Port, CAN1_LBK_Pin, GPIO_PIN_SET)
#define CAN_NORMAL() HAL_GPIO_WritePin(CAN1_LBK_GPIO_Port, CAN1_LBK_Pin, GPIO_PIN_RESET)


extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart1;
extern IWDG_HandleTypeDef hiwdg;

extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

/* Definitions for defaultTask */
osThreadId_t h_task_time;
osThreadId_t h_task_usb;
osThreadId_t h_task_sdio;
osThreadId_t h_task_can;
osThreadId_t h_task_kline;

volatile uint64_t gTick64 = 0;

char *uint64_to_str(uint64_t n, char dest[static 21]) {
    dest += 20;
    *dest-- = 0;
    while (n) {
        *dest-- = (n % 10) + '0';
        n /= 10;
    }
    return dest + 1;
}


#define LOG10_FROM_2_TO_64_PLUS_1  21
#define UINT64_TO_STR(n)  uint64_to_str(n, (char[21]){0})

typedef struct {
    const char *name;
    uint8_t type;
    uint32_t offset;
}sParameter;

/*
 * 0 = uint64_t
 * 1 = char *
 * 2 = int32_t
 * 3 = float
 * 4 = uint8_t
 * 5 = uint16_t
 */

const sParameter gParameters[] = {
    {"AdcKnockVoltage",           3, OFFSETOF(sParameters, AdcKnockVoltage)},
    {"AdcAirTemp",                3, OFFSETOF(sParameters, AdcAirTemp)},
    {"AdcEngineTemp",             3, OFFSETOF(sParameters, AdcEngineTemp)},
    {"AdcManifoldAirPressure",    3, OFFSETOF(sParameters, AdcManifoldAirPressure)},
    {"AdcThrottlePosition",       3, OFFSETOF(sParameters, AdcThrottlePosition)},
    {"AdcReferenceVoltage",       3, OFFSETOF(sParameters, AdcReferenceVoltage)},
    {"AdcLambdaUR",               3, OFFSETOF(sParameters, AdcLambdaUR)},
    {"AdcLambdaUA",               3, OFFSETOF(sParameters, AdcLambdaUA)},
    {"AdcEtcTps1",                3, OFFSETOF(sParameters, AdcEtcTps1)},
    {"AdcEtcTps2",                3, OFFSETOF(sParameters, AdcEtcTps2)},
    {"AdcEtcPedal1",              3, OFFSETOF(sParameters, AdcEtcPedal1)},
    {"AdcEtcPedal2",              3, OFFSETOF(sParameters, AdcEtcPedal2)},
    {"AdcEtcRsvd5",               3, OFFSETOF(sParameters, AdcEtcRsvd5)},
    {"AdcEtcRsvd6",               3, OFFSETOF(sParameters, AdcEtcRsvd6)},
    {"AdcEtcReferenceVoltage",    3, OFFSETOF(sParameters, AdcEtcReferenceVoltage)},
    {"AdcEtcPowerVoltage",        3, OFFSETOF(sParameters, AdcEtcPowerVoltage)},
    {"ThrottlePosition",          3, OFFSETOF(sParameters, ThrottlePosition)},
    {"PedalPosition",             3, OFFSETOF(sParameters, PedalPosition)},
    {"ThrottleDefaultPosition",   3, OFFSETOF(sParameters, ThrottleDefaultPosition)},
    {"ThrottleTargetPosition",    3, OFFSETOF(sParameters, ThrottleTargetPosition)},
    {"WishThrottleTargetPosition",3, OFFSETOF(sParameters, WishThrottleTargetPosition)},
    {"KnockSensor",               3, OFFSETOF(sParameters, KnockSensor)},
    {"KnockSensorFiltered",       3, OFFSETOF(sParameters, KnockSensorFiltered)},
    {"KnockSensorDetonate",       3, OFFSETOF(sParameters, KnockSensorDetonate)},
    {"KnockZone",                 3, OFFSETOF(sParameters, KnockZone)},
    {"KnockAdvance",              3, OFFSETOF(sParameters, KnockAdvance)},
    {"KnockSaturation",           3, OFFSETOF(sParameters, KnockSaturation)},
    {"AirTemp",                   3, OFFSETOF(sParameters, AirTemp)},
    {"EngineTemp",                3, OFFSETOF(sParameters, EngineTemp)},
    {"CalculatedAirTemp",         3, OFFSETOF(sParameters, CalculatedAirTemp)},
    {"ManifoldAirPressure",       3, OFFSETOF(sParameters, ManifoldAirPressure)},
    {"ReferenceVoltage",          3, OFFSETOF(sParameters, ReferenceVoltage)},
    {"PowerVoltage",              3, OFFSETOF(sParameters, PowerVoltage)},
    {"FuelRatio",                 3, OFFSETOF(sParameters, FuelRatio)},
    {"FuelRatioDiff",             3, OFFSETOF(sParameters, FuelRatioDiff)},
    {"LambdaValue",               3, OFFSETOF(sParameters, LambdaValue)},
    {"LambdaTemperature",         3, OFFSETOF(sParameters, LambdaTemperature)},
    {"LambdaHeaterVoltage",       3, OFFSETOF(sParameters, LambdaHeaterVoltage)},
    {"ShortTermCorrection",       3, OFFSETOF(sParameters, ShortTermCorrection)},
    {"LongTermCorrection",        3, OFFSETOF(sParameters, LongTermCorrection)},
    {"IdleCorrection",            3, OFFSETOF(sParameters, IdleCorrection)},
    {"RPM",                       3, OFFSETOF(sParameters, RPM)},
    {"Speed",                     3, OFFSETOF(sParameters, Speed)},
    {"MassAirFlow",               3, OFFSETOF(sParameters, MassAirFlow)},
    {"CyclicAirFlow",             3, OFFSETOF(sParameters, CyclicAirFlow)},
    {"EffectiveVolume",           3, OFFSETOF(sParameters, EffectiveVolume)},
    {"EngineLoad",                3, OFFSETOF(sParameters, EngineLoad)},
    {"EstimatedPower",            3, OFFSETOF(sParameters, EstimatedPower)},
    {"EstimatedTorque",           3, OFFSETOF(sParameters, EstimatedTorque)},
    {"WishFuelRatio",             3, OFFSETOF(sParameters, WishFuelRatio)},
    {"IdleValvePosition",         3, OFFSETOF(sParameters, IdleValvePosition)},
    {"IdleRegThrRPM",             3, OFFSETOF(sParameters, IdleRegThrRPM)},
    {"WishIdleRPM",               3, OFFSETOF(sParameters, WishIdleRPM)},
    {"WishIdleMassAirFlow",       3, OFFSETOF(sParameters, WishIdleMassAirFlow)},
    {"WishIdleValvePosition",     3, OFFSETOF(sParameters, WishIdleValvePosition)},
    {"WishIdleIgnitionAdvance",   3, OFFSETOF(sParameters, WishIdleIgnitionAdvance)},
    {"IgnitionAdvance",           3, OFFSETOF(sParameters, IgnitionAdvance)},
    {"InjectionPhase",            3, OFFSETOF(sParameters, InjectionPhase)},
    {"InjectionPhaseDuration",    3, OFFSETOF(sParameters, InjectionPhaseDuration)},
    {"InjectionPulse",            3, OFFSETOF(sParameters, InjectionPulse)},
    {"InjectionDutyCycle",        3, OFFSETOF(sParameters, InjectionDutyCycle)},
    {"InjectionEnrichment",       3, OFFSETOF(sParameters, InjectionEnrichment)},
    {"InjectionLag",              3, OFFSETOF(sParameters, InjectionLag)},
    {"IgnitionPulse",             3, OFFSETOF(sParameters, IgnitionPulse)},
    {"IdleSpeedShift",            3, OFFSETOF(sParameters, IdleSpeedShift)},
    {"EnrichmentSyncAmount",      3, OFFSETOF(sParameters, EnrichmentSyncAmount)},
    {"EnrichmentAsyncAmount",     3, OFFSETOF(sParameters, EnrichmentAsyncAmount)},
    {"EnrichmentStartLoad",       3, OFFSETOF(sParameters, EnrichmentStartLoad)},
    {"EnrichmentLoadDerivative",  3, OFFSETOF(sParameters, EnrichmentLoadDerivative)},
    {"DrivenKilometers",          3, OFFSETOF(sParameters, DrivenKilometers)},
    {"FuelConsumed",              3, OFFSETOF(sParameters, FuelConsumed)},
    {"FuelConsumption",           3, OFFSETOF(sParameters, FuelConsumption)},
    {"FuelHourly",                3, OFFSETOF(sParameters, FuelHourly)},
    {"TspsRelativePosition",      3, OFFSETOF(sParameters, TspsRelativePosition)},
    {"IdleWishToRpmRelation",     3, OFFSETOF(sParameters, IdleWishToRpmRelation)},
    {"KnockCount",                5, OFFSETOF(sParameters, KnockCount)},
    {"KnockCountCy1",             5, OFFSETOF(sParameters, KnockCountCy[0])},
    {"KnockCountCy2",             5, OFFSETOF(sParameters, KnockCountCy[1])},
    {"KnockCountCy3",             5, OFFSETOF(sParameters, KnockCountCy[2])},
    {"KnockCountCy4",             5, OFFSETOF(sParameters, KnockCountCy[3])},
    {"SwitchPosition",            4, OFFSETOF(sParameters, SwitchPosition)},
    {"CurrentTable",              4, OFFSETOF(sParameters, CurrentTable)},
    {"InjectorChannel",           4, OFFSETOF(sParameters, InjectorChannel)},
    {"IdleFlag",                  4, OFFSETOF(sParameters, IdleFlag)},
    {"IdleCorrFlag",              4, OFFSETOF(sParameters, IdleCorrFlag)},
    {"IdleEconFlag",              4, OFFSETOF(sParameters, IdleEconFlag)},
    {"LambdaValid",               4, OFFSETOF(sParameters, LambdaValid)},
    {"OilSensor",                 4, OFFSETOF(sParameters, OilSensor)},
    {"FanForceSwitch",            4, OFFSETOF(sParameters, FanForceSwitch)},
    {"HandbrakeSensor",           4, OFFSETOF(sParameters, HandbrakeSensor)},
    {"ChargeSensor",              4, OFFSETOF(sParameters, ChargeSensor)},
    {"ClutchSensor",              4, OFFSETOF(sParameters, ClutchSensor)},
    {"IgnSensor",                 4, OFFSETOF(sParameters, IgnSensor)},
    {"FuelPumpRelay",             4, OFFSETOF(sParameters, FuelPumpRelay)},
    {"FanRelay",                  4, OFFSETOF(sParameters, FanRelay)},
    {"CheckEngine",               4, OFFSETOF(sParameters, CheckEngine)},
    {"StarterRelay",              4, OFFSETOF(sParameters, StarterRelay)},
    {"FanSwitch",                 4, OFFSETOF(sParameters, FanSwitch)},
    {"IgnOutput",                 4, OFFSETOF(sParameters, IgnOutput)},
    {"StartAllowed",              4, OFFSETOF(sParameters, StartAllowed)},
    {"IsRunning",                 4, OFFSETOF(sParameters, IsRunning)},
    {"IsCheckEngine",             4, OFFSETOF(sParameters, IsCheckEngine)},

    {"EtcMotorActiveFlag",        4, OFFSETOF(sParameters, EtcMotorActiveFlag)},
    {"EtcStandaloneFlag",         4, OFFSETOF(sParameters, EtcStandaloneFlag)},
    {"EtcMotorFullCloseFlag",     4, OFFSETOF(sParameters, EtcMotorFullCloseFlag)},

    {"EtcOutCruizeG",             4, OFFSETOF(sParameters, EtcOutCruizeG)},
    {"EtcOutCruizeR",             4, OFFSETOF(sParameters, EtcOutCruizeR)},
    {"EtcOutRsvd3",               4, OFFSETOF(sParameters, EtcOutRsvd3)},
    {"EtcOutRsvd4",               4, OFFSETOF(sParameters, EtcOutRsvd4)},
    {"EtcInCruizeStart",          4, OFFSETOF(sParameters, EtcInCruizeStart)},
    {"EtcInCruizeStop",           4, OFFSETOF(sParameters, EtcInCruizeStop)},
    {"EtcInBrake",                4, OFFSETOF(sParameters, EtcInBrake)},
    {"EtcInRsvd4",                4, OFFSETOF(sParameters, EtcInRsvd4)},
    {"EtcInRsvd5",                4, OFFSETOF(sParameters, EtcInRsvd5)},
    {"EtcInRsvd6",                4, OFFSETOF(sParameters, EtcInRsvd6)},

    {"CylinderIgnitionBitmask",   4, OFFSETOF(sParameters, CylinderIgnitionBitmask)},
    {"CylinderInjectionBitmask",  4, OFFSETOF(sParameters, CylinderInjectionBitmask)}

};

#define BIT_GET(array,bit) ((array)[(bit) >> 3] & (1 << ((bit) & 7)))
#define BIT_SET(array,bit)   ((array)[(bit) >> 3] |= (1 << ((bit) & 7)))
#define BIT_RESET(array,bit)   ((array)[(bit) >> 3] &= ~(1 << ((bit) & 7)))
static uint8_t gConfigBitmap[16] = {0};

const char *gFileInitialHeader = "TimePoint";

typedef struct {
    uint64_t timestamp;
    sParameters params;
}sLoggerParameters;

#define PARAMS_BUFFER_SIZE 40
static sLoggerParameters gParamsBuffer[2][PARAMS_BUFFER_SIZE];
static sProFIFO gParamsFifo[2];

const osThreadAttr_t attrs_task_time = {
  .name = "taskTime",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

const osThreadAttr_t attrs_task_usb = {
  .name = "taskUsb",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t attrs_task_sdio = {
  .name = "taskSdio",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t attrs_task_can = {
  .name = "taskCan",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t attrs_task_kline = {
  .name = "taskKline",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

void StartTimeTask(void *argument)
{
  for(;;)
  {
    HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, FORMAT_BIN);
    osDelay(100);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM7) {
    led_irq();
    gTick64++;
  }
}

struct sLogDriver {
    uint8_t initialized;
    uint8_t was_initialized;
    FRESULT fres;
    char filename[32];
    char string[128];
    uint16_t filenumber;
    UINT read;
    UINT wrote;
    sLoggerParameters parameters;
    uint64_t InitTime64;
    uint64_t diff;
    uint64_t sync_last;

    eLed led;
    FATFS *fatfs;
    FIL *file;
    const char *path;
    sProFIFO *fifo;
};

static void driver_loop(struct sLogDriver *driver)
{
  uint32_t readpos;

  led_set(driver->led, LedOff);
  led_set_post(driver->led, LedOff);

  for(;;)
  {
    if(!driver->initialized) {
      if(!driver->initialized && driver->was_initialized) {
        f_close(driver->file);
        f_mount(NULL, driver->path, 0);
        driver->was_initialized = 0;
      }

      osDelay(10);

      driver->fres = f_mount(driver->fatfs, driver->path, 1);
      if(driver->fres != FR_OK) {
        led_set(driver->led, LedOff);
        led_set_post(driver->led, LedOff);
        continue;
      }

      led_set(driver->led, LedLong);
      led_set_post(driver->led, LedOn);

      driver->was_initialized = 1;

      sprintf(driver->filename, "%s/params.txt", driver->path);
      driver->fres = f_open(driver->file, driver->filename, FA_OPEN_EXISTING | FA_READ);
      if(driver->fres == FR_OK) {
        if(f_size(driver->file) > 0) {
          memset(gConfigBitmap, 0, sizeof(gConfigBitmap));
          for(readpos = 0; readpos < sizeof(driver->string);) {
            driver->fres = f_read(driver->file, &driver->string[readpos], 1, &driver->read);
            if(driver->fres != FR_OK) { f_close(driver->file); continue; }
            if(driver->string[readpos] == ',' || driver->string[readpos] == '\r' ||
                driver->string[readpos] == '\n' || driver->string[readpos] == ' ' ||
                driver->string[readpos] == '\0') {
              if(readpos > 0) {
                driver->string[readpos] = '\0';
                for(int j = 0; j < ITEMSOF(gParameters); j++) {
                  if(strcmp(gParameters[j].name, driver->string) == 0) {
                    BIT_SET(gConfigBitmap, j);
                    break;
                  }
                }
                readpos = 0;
              }
            } else {
              readpos++;
            }
            if(f_eof(driver->file))
              break;
          }
        }
        f_close(driver->file);
      } else if(driver->fres != FR_NO_FILE) {
        continue;
      }

      sprintf(driver->filename, "%s/log_last.txt", driver->path);
      driver->fres = f_open(driver->file, driver->filename, FA_OPEN_EXISTING | FA_READ);
      if(driver->fres == FR_OK) {
        if(f_size(driver->file) >= sizeof(driver->string)) {
          driver->fres = f_close(driver->file);
          if(driver->fres != FR_OK) { continue; }
          driver->filenumber = 1;
        } else {
          driver->fres = f_read(driver->file, driver->string, f_size(driver->file), &driver->read);
          if(driver->fres != FR_OK) { f_close(driver->file); continue; }
          driver->fres = f_close(driver->file);
          if(driver->fres != FR_OK) { continue; }
          driver->string[driver->read] = 0;
          driver->fres = sscanf(driver->string, "%hu", &driver->filenumber);
          if(driver->fres == 1) {
            driver->filenumber++;
          }

        }
      } else if(driver->fres == FR_NO_FILE) {
        driver->filenumber = 1;
      } else { continue; }

      driver->fres = f_open(driver->file, driver->filename, FA_CREATE_ALWAYS | FA_WRITE);
      if(driver->fres == FR_OK) {
        sprintf(driver->filename, "%05hu", driver->filenumber);
        driver->fres = f_write(driver->file, driver->filename, strlen(driver->filename), &driver->read);
        if(driver->fres != FR_OK) { f_close(driver->file); continue; }
        driver->fres = f_sync(driver->file);
        if(driver->fres != FR_OK) { f_close(driver->file); continue; }
        driver->fres = f_close(driver->file);
        if(driver->fres != FR_OK) { continue; }
      } else {
        osDelay(100);
        continue;
      }

      sprintf(driver->filename, "%s/log_%05hu.eculog", driver->path, driver->filenumber);
      driver->fres = f_open(driver->file, driver->filename, FA_CREATE_ALWAYS | FA_WRITE);
      if(driver->fres != FR_OK) { continue; }
      driver->fres = f_sync(driver->file);
      if(driver->fres != FR_OK) { f_close(driver->file); continue; }

      strcpy(driver->string, gFileInitialHeader);
      driver->fres = f_write(driver->file, driver->string, strlen(driver->string), &driver->wrote);
      if(driver->fres != FR_OK) { f_close(driver->file); continue; }

      for(int i = 0; i < ITEMSOF(gParameters); i++) {
        if(BIT_GET(gConfigBitmap, i)) {
          strcpy(driver->string, ",");
          strcat(driver->string, gParameters[i].name);
          driver->fres = f_write(driver->file, driver->string, strlen(driver->string), &driver->wrote);
          if(driver->fres != FR_OK)
            break;
        }
      }
      if(driver->fres != FR_OK) { f_close(driver->file); continue; }

      strcpy(driver->string, "\r\n");
      driver->fres = f_write(driver->file, driver->string, strlen(driver->string), &driver->wrote);
      if(driver->fres != FR_OK) { f_close(driver->file); continue; }

      driver->fres = f_sync(driver->file);
      if(driver->fres != FR_OK) { f_close(driver->file); continue; }

      driver->initialized = 1;
      driver->InitTime64 = gTick64;
      driver->sync_last = gTick64;

      led_set(driver->led, LedOn);
      led_set_post(driver->led, LedOn);
    }

    if(protPull(driver->fifo, &driver->parameters)) {
      if(driver->parameters.timestamp < driver->InitTime64) {
        driver->InitTime64 = driver->parameters.timestamp;
      }
      driver->diff = driver->parameters.timestamp - driver->InitTime64;
      if(driver->diff > 0) {
        sprintf(driver->string, "%s000", UINT64_TO_STR(driver->diff));
      } else {
        strcpy(driver->string, "0");
      }
      driver->fres = f_write(driver->file, driver->string, strlen(driver->string), &driver->wrote);
      if(driver->fres != FR_OK) { f_close(driver->file); driver->initialized = 0; continue; }
      for(int i = 0; i < ITEMSOF(gParameters);) {
        driver->string[0] = '\0';
        if(BIT_GET(gConfigBitmap, i)) {
          const void *ptr = ((uint8_t *)&driver->parameters.params) + gParameters[i].offset;
          switch(gParameters[i].type) {
            case 1:
              sprintf(driver->string, ",%s", (const char *)ptr);
              break;
            case 2:
              sprintf(driver->string, ",%ld", *(const int32_t *)ptr);
              break;
            case 3:
              sprintf(driver->string, ",%f", *(const float *)ptr);
              break;
            case 4:
              sprintf(driver->string, ",%hu", *(const uint8_t *)ptr);
              break;
            case 5:
              sprintf(driver->string, ",%hu", *(const uint16_t *)ptr);
              break;
            default:
              break;
          }
        }

        if(++i >= ITEMSOF(gParameters)) {
          strcat(driver->string, "\r\n");
        }
        if(driver->string[0]) {
          driver->fres = f_write(driver->file, driver->string, strlen(driver->string), &driver->wrote);
          if(driver->fres != FR_OK) { f_close(driver->file); driver->initialized = 0; continue; }
        }
      }

      if(gTick64 - driver->sync_last > 1000) {
        driver->fres = f_sync(driver->file);
        if(driver->fres != FR_OK) { f_close(driver->file); driver->initialized = 0; continue; }
        driver->sync_last = gTick64;
      }

      led_set(driver->led, LedShortSingle);
    } else {
      osDelay(1);
    }

  }
}

static int8_t ecu_can_process_message(const sCanRawMessage *message, sLoggerParameters *logger_parameters)
{
  const uint32_t messages_count = 12;
  const uint32_t msgs_expected = (1 << messages_count) - 1;
  static uint32_t msgs_bitmap = 0;
  sParameters *parameters = &logger_parameters->params;

  int8_t status = 0;

  sCanMessage * can_msg = can_message_get_msg(message);

  if(can_msg != NULL) {
    switch(can_msg->Id) {
      case 0x020:
        can_signal_get_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcKnockVoltage, &parameters->AdcKnockVoltage);
        can_signal_get_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcAirTemp, &parameters->AdcAirTemp);
        can_signal_get_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcEngineTemp, &parameters->AdcEngineTemp);
        can_signal_get_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcManifoldAirPressure, &parameters->AdcManifoldAirPressure);
        can_signal_get_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcThrottlePosition, &parameters->AdcThrottlePosition);
        can_signal_get_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcReferenceVoltage, &parameters->AdcReferenceVoltage);
        can_signal_get_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcLambdaUR, &parameters->AdcLambdaUR);
        can_signal_get_float(&g_can_message_id020_ECU, &g_can_signal_id020_ECU_AdcLambdaUA, &parameters->AdcLambdaUA);
        msgs_bitmap |= 1 << (can_msg->Id - 0x020);
        break;
      case 0x021:
        can_signal_get_float(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockSensor, &parameters->KnockSensor);
        can_signal_get_float(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockSensorFiltered, &parameters->KnockSensorFiltered);
        can_signal_get_float(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockSensorDetonate, &parameters->KnockSensorDetonate);
        can_signal_get_float(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockSaturation, &parameters->KnockSaturation);
        can_signal_get_float(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockAdvance, &parameters->KnockAdvance);
        can_signal_get_uint16(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockCountCy1, &parameters->KnockCountCy[0]);
        can_signal_get_uint16(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockCountCy2, &parameters->KnockCountCy[1]);
        can_signal_get_uint16(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockCountCy3, &parameters->KnockCountCy[2]);
        can_signal_get_uint16(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockCountCy4, &parameters->KnockCountCy[3]);
        can_signal_get_uint16(&g_can_message_id021_ECU, &g_can_signal_id021_ECU_KnockCount, &parameters->KnockCount);
        msgs_bitmap |= 1 << (can_msg->Id - 0x020);
        break;
      case 0x022:
        can_signal_get_float(&g_can_message_id022_ECU, &g_can_signal_id022_ECU_AirTemp, &parameters->AirTemp);
        can_signal_get_float(&g_can_message_id022_ECU, &g_can_signal_id022_ECU_EngineTemp, &parameters->EngineTemp);
        can_signal_get_float(&g_can_message_id022_ECU, &g_can_signal_id022_ECU_CalculatedAirTemp, &parameters->CalculatedAirTemp);
        can_signal_get_float(&g_can_message_id022_ECU, &g_can_signal_id022_ECU_ManifoldAirPressure, &parameters->ManifoldAirPressure);
        can_signal_get_float(&g_can_message_id022_ECU, &g_can_signal_id022_ECU_ThrottlePosition, &parameters->ThrottlePosition);
        can_signal_get_float(&g_can_message_id022_ECU, &g_can_signal_id022_ECU_RPM, &parameters->RPM);
        msgs_bitmap |= 1 << (can_msg->Id - 0x020);
        break;
      case 0x023:
        can_signal_get_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_FuelRatio, &parameters->FuelRatio);
        can_signal_get_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_FuelRatioDiff, &parameters->FuelRatioDiff);
        can_signal_get_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_LambdaValue, &parameters->LambdaValue);
        can_signal_get_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_LambdaTemperature, &parameters->LambdaTemperature);
        can_signal_get_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_LambdaHeaterVoltage, &parameters->LambdaHeaterVoltage);
        can_signal_get_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_ShortTermCorrection, &parameters->ShortTermCorrection);
        can_signal_get_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_LongTermCorrection, &parameters->LongTermCorrection);
        can_signal_get_float(&g_can_message_id023_ECU, &g_can_signal_id023_ECU_IdleCorrection, &parameters->IdleCorrection);
        msgs_bitmap |= 1 << (can_msg->Id - 0x020);
        break;
      case 0x024:
        can_signal_get_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_Speed, &parameters->Speed);
        can_signal_get_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_MassAirFlow, &parameters->MassAirFlow);
        can_signal_get_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_CyclicAirFlow, &parameters->CyclicAirFlow);
        can_signal_get_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_EffectiveVolume, &parameters->EffectiveVolume);
        can_signal_get_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_EngineLoad, &parameters->EngineLoad);
        can_signal_get_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_EstimatedPower, &parameters->EstimatedPower);
        can_signal_get_float(&g_can_message_id024_ECU, &g_can_signal_id024_ECU_EstimatedTorque, &parameters->EstimatedTorque);
        msgs_bitmap |= 1 << (can_msg->Id - 0x020);
        break;
      case 0x025:
        can_signal_get_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_WishFuelRatio, &parameters->WishFuelRatio);
        can_signal_get_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_IdleValvePosition, &parameters->IdleValvePosition);
        can_signal_get_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_IdleRegThrRPM, &parameters->IdleRegThrRPM);
        can_signal_get_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_WishIdleRPM, &parameters->WishIdleRPM);
        can_signal_get_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_WishIdleMassAirFlow, &parameters->WishIdleMassAirFlow);
        can_signal_get_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_WishIdleValvePosition, &parameters->WishIdleValvePosition);
        can_signal_get_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_WishIdleIgnitionAdvance, &parameters->WishIdleIgnitionAdvance);
        can_signal_get_float(&g_can_message_id025_ECU, &g_can_signal_id025_ECU_IdleSpeedShift, &parameters->IdleSpeedShift);
        msgs_bitmap |= 1 << (can_msg->Id - 0x020);
        break;
      case 0x026:
        can_signal_get_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_InjectionPhase, &parameters->InjectionPhase);
        can_signal_get_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_InjectionPhaseDuration, &parameters->InjectionPhaseDuration);
        can_signal_get_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_InjectionPulse, &parameters->InjectionPulse);
        can_signal_get_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_InjectionDutyCycle, &parameters->InjectionDutyCycle);
        can_signal_get_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_IgnitionPulse, &parameters->IgnitionPulse);
        can_signal_get_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_InjectionLag, &parameters->InjectionLag);
        can_signal_get_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_TspsRelativePosition, &parameters->TspsRelativePosition);
        can_signal_get_float(&g_can_message_id026_ECU, &g_can_signal_id026_ECU_IgnitionAdvance, &parameters->IgnitionAdvance);
        msgs_bitmap |= 1 << (can_msg->Id - 0x020);
        break;
      case 0x027:
        can_signal_get_float(&g_can_message_id027_ECU, &g_can_signal_id027_ECU_EnrichmentSyncAmount, &parameters->EnrichmentSyncAmount);
        can_signal_get_float(&g_can_message_id027_ECU, &g_can_signal_id027_ECU_EnrichmentAsyncAmount, &parameters->EnrichmentAsyncAmount);
        can_signal_get_float(&g_can_message_id027_ECU, &g_can_signal_id027_ECU_EnrichmentStartLoad, &parameters->EnrichmentStartLoad);
        can_signal_get_float(&g_can_message_id027_ECU, &g_can_signal_id027_ECU_EnrichmentLoadDerivative, &parameters->EnrichmentLoadDerivative);
        can_signal_get_float(&g_can_message_id027_ECU, &g_can_signal_id027_ECU_InjectionEnrichment, &parameters->InjectionEnrichment);
        can_signal_get_float(&g_can_message_id027_ECU, &g_can_signal_id027_ECU_IdleWishToRpmRelation, &parameters->IdleWishToRpmRelation);
        msgs_bitmap |= 1 << (can_msg->Id - 0x020);
        break;
      case 0x028:
        can_signal_get_float(&g_can_message_id028_ECU, &g_can_signal_id028_ECU_DrivenKilometers, &parameters->DrivenKilometers);
        can_signal_get_float(&g_can_message_id028_ECU, &g_can_signal_id028_ECU_FuelConsumption, &parameters->FuelConsumption);
        can_signal_get_float(&g_can_message_id028_ECU, &g_can_signal_id028_ECU_FuelConsumed, &parameters->FuelConsumed);
        can_signal_get_float(&g_can_message_id028_ECU, &g_can_signal_id028_ECU_FuelHourly, &parameters->FuelHourly);
        msgs_bitmap |= 1 << (can_msg->Id - 0x020);
        break;
      case 0x029:
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_LambdaValid, &parameters->LambdaValid);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_OilSensor, &parameters->OilSensor);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_FanForceSwitch, &parameters->FanForceSwitch);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_HandbrakeSensor, &parameters->HandbrakeSensor);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_ChargeSensor, &parameters->ChargeSensor);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_ClutchSensor, &parameters->ClutchSensor);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IgnSensor, &parameters->IgnSensor);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_FuelPumpRelay, &parameters->FuelPumpRelay);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_FanRelay, &parameters->FanRelay);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_CheckEngine, &parameters->CheckEngine);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_StarterRelay, &parameters->StarterRelay);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_FanSwitch, &parameters->FanSwitch);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IgnOutput, &parameters->IgnOutput);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_StartAllowed, &parameters->StartAllowed);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IsRunning, &parameters->IsRunning);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IsCheckEngine, &parameters->IsCheckEngine);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_CylinderIgnitionBitmask, &parameters->CylinderIgnitionBitmask);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_CylinderInjectionBitmask, &parameters->CylinderInjectionBitmask);
        can_signal_get_float(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_PowerVoltage, &parameters->PowerVoltage);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IdleFlag, &parameters->IdleFlag);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IdleCorrFlag, &parameters->IdleCorrFlag);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_IdleEconFlag, &parameters->IdleEconFlag);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_SwitchPosition, &parameters->SwitchPosition);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_CurrentTable, &parameters->CurrentTable);
        can_signal_get_uint8(&g_can_message_id029_ECU, &g_can_signal_id029_ECU_InjectorChannel, &parameters->InjectorChannel);
        msgs_bitmap |= 1 << (can_msg->Id - 0x020);
        break;
      case 0x02A:
        can_signal_get_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcTps1, &parameters->AdcEtcTps1);
        can_signal_get_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcTps2, &parameters->AdcEtcTps2);
        can_signal_get_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcPedal1, &parameters->AdcEtcPedal1);
        can_signal_get_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcPedal2, &parameters->AdcEtcPedal2);
        can_signal_get_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcRsvd5, &parameters->AdcEtcRsvd5);
        can_signal_get_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcRsvd6, &parameters->AdcEtcRsvd6);
        can_signal_get_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcReferenceVoltage, &parameters->AdcEtcReferenceVoltage);
        can_signal_get_float(&g_can_message_id02A_ECU, &g_can_signal_id02A_ECU_EtcAdcPowerVoltage, &parameters->AdcEtcPowerVoltage);
        break;
      case 0x02B:
        can_signal_get_float(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcThrottleTargetPosition, &parameters->ThrottleTargetPosition);
        can_signal_get_float(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcThrottleDefaultPosition, &parameters->ThrottleDefaultPosition);
        can_signal_get_float(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcWishThrottleTargetPosition, &parameters->WishThrottleTargetPosition);
        can_signal_get_float(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcPedalPosition, &parameters->PedalPosition);
        can_signal_get_uint8(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcStandaloneFlag, &parameters->EtcStandaloneFlag);
        can_signal_get_uint8(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcMotorActive, &parameters->EtcMotorActiveFlag);
        can_signal_get_uint8(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcMotorFullCloseFlag, &parameters->EtcMotorFullCloseFlag);
        can_signal_get_uint8(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcInCruizeStart, &parameters->EtcInCruizeStart);
        can_signal_get_uint8(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcInCruizeStop, &parameters->EtcInCruizeStop);
        can_signal_get_uint8(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcInBrake, &parameters->EtcInBrake);
        can_signal_get_uint8(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcInRsvd4, &parameters->EtcInRsvd4);
        can_signal_get_uint8(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcInRsvd5, &parameters->EtcInRsvd5);
        can_signal_get_uint8(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcInRsvd6, &parameters->EtcInRsvd6);
        can_signal_get_uint8(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcOutCruizeG, &parameters->EtcOutCruizeG);
        can_signal_get_uint8(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcOutCruizeR, &parameters->EtcOutCruizeR);
        can_signal_get_uint8(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcOutRsvd3, &parameters->EtcOutRsvd3);
        can_signal_get_uint8(&g_can_message_id02B_ECU, &g_can_signal_id02B_ECU_EtcOutRsvd4, &parameters->EtcOutRsvd4);
        break;
      default:
        break;
    }

    if(msgs_bitmap >= msgs_expected) {
      led_set(LedCan, LedShort);
      logger_parameters->timestamp = gTick64;
      for(int i = 0; i < ITEMSOF(gParamsFifo); i++) {
        if(gParamsFifo[i].buffer && gParamsFifo[i].info.capacity) {
          protPush(&gParamsFifo[i], logger_parameters);
        }
      }
      msgs_bitmap = 0;
    }
  }
  return status;
}

void StartUsbTask(void *argument)
{
  struct sLogDriver driver = {0};

  MX_USB_HOST_Init();

  driver.fifo = &gParamsFifo[0];
  driver.fatfs = &USBHFatFS;
  driver.file = &USBHFile;
  driver.path = USBHPath;
  driver.led = LedUsb;
  protInit(driver.fifo, gParamsBuffer[1], sizeof(sLoggerParameters), PARAMS_BUFFER_SIZE);

  driver_loop(&driver);
}

void StartSdioTask(void *argument)
{
  struct sLogDriver driver = {0};

  driver.fifo = &gParamsFifo[1];
  driver.fatfs = &SDFatFS;
  driver.file = &SDFile;
  driver.path = SDPath;
  driver.led = LedSdio;
  protInit(driver.fifo, gParamsBuffer[0], sizeof(sLoggerParameters), PARAMS_BUFFER_SIZE);

  driver_loop(&driver);
}

void StartCanTask(void *argument)
{
  uint64_t now, last;
  sLoggerParameters parameters = {0};
  static sCanRawMessage message = {0};
  int8_t status;

  can_init(&hcan1);

  const uint16_t filter_ids[] = { 0x020, 0x028 };
  const uint16_t filter_masks[] = { 0x7F8, 0x7F8 };
  can_start(filter_ids, filter_masks, ITEMSOF(filter_ids));

  led_set(LedCan, LedOn);
  led_set_post(LedCan, LedOn);

  can_message_register_msg(&g_can_message_id020_ECU);
  can_message_register_msg(&g_can_message_id021_ECU);
  can_message_register_msg(&g_can_message_id022_ECU);
  can_message_register_msg(&g_can_message_id023_ECU);
  can_message_register_msg(&g_can_message_id024_ECU);
  can_message_register_msg(&g_can_message_id025_ECU);
  can_message_register_msg(&g_can_message_id026_ECU);
  can_message_register_msg(&g_can_message_id027_ECU);
  can_message_register_msg(&g_can_message_id028_ECU);
  can_message_register_msg(&g_can_message_id029_ECU);

  now = gTick64;
  last = now;

  while(1)
  {
    now = gTick64;

    can_loop();

    status = can_receive(&message);
    if(status > 0) {
      ecu_can_process_message(&message, &parameters);
    }

    if(now - last > 1000) {
      can_signal_append_uint(&g_can_message_id110_LOG_ECU, &g_can_signal_id110_LOG_ECU_Unique, (uint16_t)now);
      can_message_send(&g_can_message_id110_LOG_ECU);
      last = now;
    }

    osDelay(1);
  }
}

void StartKlineTask(void *argument)
{
  led_set(LedKline, LedOff);
  for(;;)
  {
    osDelay(1000);
  }
}

void tasksInit(void)
{
  memset(gConfigBitmap, 0, sizeof(gConfigBitmap));
  for(int i = 0; i < ITEMSOF(gParameters); i++) {
    BIT_SET(gConfigBitmap, i);
  }

  h_task_time = osThreadNew(StartTimeTask, NULL, &attrs_task_time);
  //h_task_usb = osThreadNew(StartUsbTask, NULL, &attrs_task_usb);
  h_task_sdio = osThreadNew(StartSdioTask, NULL, &attrs_task_sdio);
  h_task_can = osThreadNew(StartCanTask, NULL, &attrs_task_can);
  h_task_kline = osThreadNew(StartKlineTask, NULL, &attrs_task_kline);
}
