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
#include <stdio.h>
#include <string.h>

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

const char *gFileHeader = "TimePoint,CurrentTableName,SwitchPosition,CurrentTable,InjectorChannel,AdcKnockVoltage,AdcAirTemp,AdcEngineTemp,"
    "AdcManifoldAirPressure,AdcThrottlePosition,AdcPowerVoltage,AdcReferenceVoltage,AdcLambdaUR,AdcLambdaUA,KnockSensor,KnockSensorFiltered,"
    "AirTemp,EngineTemp,ManifoldAirPressure,ThrottlePosition,ReferenceVoltage,PowerVoltage,FuelRatio,LambdaValue,LambdaTemperature,"
    "LambdaHeaterVoltage,LambdaTemperatureVoltage,ShortTermCorrection,LongTermCorrection,IdleCorrection,IdleFlag,RPM,Speed,Acceleration,"
    "MassAirFlow,CyclicAirFlow,EffectiveVolume,AirDestiny,RelativeFilling,WishFuelRatio,IdleValvePosition,WishIdleRPM,WishIdleMassAirFlow,"
    "WishIdleValvePosition,WishIdleIgnitionAngle,IgnitionAngle,InjectionPhase,InjectionPhaseDuration,InjectionPulse,InjectionDutyCycle,"
    "InjectionEnrichment,InjectionLag,IgnitionPulse,IdleSpeedShift,DrivenKilometers,FuelConsumed,FuelConsumption,FuelHourly,TspsRelativePosition,"
    "LambdaValid,OilSensor,StarterSensor,HandbrakeSensor,ChargeSensor,ClutchSensor,IgnSensor,FuelPumpRelay,FanRelay,CheckEngine,StarterRelay,"
    "Rsvd1Output,IgnOutput,StartAllowed,IsRunning,IsCheckEngine\r\n";

/*
 * 0 = uint64_t
 * 1 = char *
 * 2 = int32_t
 * 3 = float
 */
const uint8_t gFileFormats[] = {
    0,1,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,2,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,2,
    2,2,2,2,2,2,2,2,2,2,2,2,2,2,2
  };

const osThreadAttr_t attrs_task_time = {
  .name = "taskTime",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t attrs_task_usb = {
  .name = "taskUsb",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t attrs_task_sdio = {
  .name = "taskSdio",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t attrs_task_can = {
  .name = "taskCan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t attrs_task_kline = {
  .name = "taskKline",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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

void StartUsbTask(void *argument)
{
  uint8_t state = 0;
  uint8_t state_prev = 0;
  uint8_t initialized = 0;
  FRESULT fres;
  char filename[32];
  char string[128];
  FATFS *fatfs = &USBHFatFS;
  FIL *file = &USBHFile;
  const char *path = USBHPath;
  uint16_t filenumber = 0;
  UINT read;
  UINT wrote;
  uint8_t recreate = 0;
  sParameters parameters = {0};
  uint64_t InitTime64 = 0;

  MX_USB_HOST_Init();

  for(;;)
  {
    state = USBH_GetAppliState() == APPLICATION_READY;
    if(state) {
      if(!initialized) {
        fres = f_mount(fatfs, path, 1);
        if(fres != FR_OK) {
          osDelay(100);
          continue;
        }

        recreate = 0;
        fres = f_open(file, "last.txt", FA_OPEN_EXISTING | FA_READ);
        if(fres == FR_OK) {
          if(f_size(file) >= sizeof(string)) {
            fres = f_close(file);
            if(fres != FR_OK) { osDelay(100); continue; }
            filenumber = 1;
            recreate = 1;
          } else {
            fres = f_read(file, string, f_size(file), &read);
            if(fres != FR_OK) { f_close(file); osDelay(100); continue; }
            fres = f_close(file);
            if(fres != FR_OK) { osDelay(100); continue; }
            string[read] = 0;
            fres = sscanf(string, "%hu", &filenumber);
            if(fres != 1) {
              recreate = 1;
            }

          }
        }

        if(recreate) {
          fres = f_open(file, "last.txt", FA_CREATE_ALWAYS | FA_WRITE);
          if(fres == FR_OK) {
            sprintf(filename, "%05hu", filenumber);
            fres = f_write(file, filename, strlen(filename), &read);
            if(fres != FR_OK) { f_close(file); osDelay(100); continue; }
            fres = f_sync(file);
            if(fres != FR_OK) { f_close(file); osDelay(100); continue; }
            fres = f_close(file);
            if(fres != FR_OK) { osDelay(100); continue; }
          } else {
            osDelay(100);
            continue;
          }
        }

        sprintf(filename, "%slog_%05hu.eculog", path, filenumber);
        fres = f_open(file, filename, FA_CREATE_ALWAYS | FA_WRITE);
        if(fres != FR_OK) { osDelay(100); continue; }
        fres = f_write(file, gFileHeader, strlen(gFileHeader), &wrote);
        if(fres != FR_OK) { f_close(file); osDelay(100); continue; }
        fres = f_sync(file);
        if(fres != FR_OK) { f_close(file); osDelay(100); continue; }

        initialized = 1;
        InitTime64 = gTick64;
      }

      //TODO: logic of I/O of parameters
      if(1) {
        sprintf(string, "%llu000,", gTick64 - InitTime64);
        fres = f_write(file, string, strlen(string), &wrote);
        if(fres != FR_OK) { f_close(file); initialized = 0; state_prev = 0; osDelay(100); continue; }
        for(int i = 0, j = 0; i < sizeof(parameters) / sizeof(uint32_t); j++) {
          string[0] = ',';
          string[1] = '\0';
          switch(gFileFormats[j]) {
            case 1:
              sprintf(string, "%s,", (const char *)(((uint32_t *)&parameters)[i]));
              break;
            case 2:
              sprintf(string, "%ld,", ((const int32_t *)&parameters)[i]);
              break;
            case 3:
              sprintf(string, "%f,", ((const float *)&parameters)[i]);
              break;
            default:
              break;
          }
          if(i == 0) {
            i += TABLE_STRING_MAX / sizeof(uint32_t);
          } else {
            i++;
          }
          if(i >= sizeof(sizeof(parameters) / sizeof(uint32_t))) {
            string[strlen(string) - 1] = '\0';
          }
          fres = f_write(file, string, strlen(string), &wrote);
          if(fres != FR_OK) { f_close(file); initialized = 0; state_prev = 0; osDelay(100); continue; }

        }
      }

      strcpy(string, "\r\n");
      fres = f_write(file, string, strlen(string), &wrote);
      if(fres != FR_OK) { f_close(file); initialized = 0; state_prev = 0; osDelay(100); continue; }
      fres = f_sync(file);
      if(fres != FR_OK) { f_close(file); initialized = 0; state_prev = 0; osDelay(100); continue; }

    } else {
      led_set(LedUsb, LedOff);
      led_set_post(LedUsb, LedOff);

      if(state_prev != state && initialized) {
        f_close(file);
        f_mount(NULL, path, 0);
        initialized = 0;
      }
    }

    state_prev = state;
    osDelay(1);
  }
}

void StartSdioTask(void *argument)
{
  uint8_t initialized = 0;
  uint8_t was_initialized = 0;
  FRESULT fres;
  FATFS *fatfs = &SDFatFS;
  FIL *file = &SDFile;
  const char *path = SDPath;

  for(;;)
  {
    if(!initialized && was_initialized) {
      f_close(file);
      f_mount(NULL, path, 0);
      was_initialized = 0;
    }

    if(!initialized) {
      fres = f_mount(fatfs, path, 1);
      if(fres != FR_OK) {
        osDelay(100);
        continue;
      }
      initialized = 1;
      was_initialized = initialized;
    }
  }
}

void StartCanTask(void *argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

void StartKlineTask(void *argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

void tasksInit(void)
{
  h_task_time = osThreadNew(StartTimeTask, NULL, &attrs_task_time);
  h_task_usb = osThreadNew(StartUsbTask, NULL, &attrs_task_usb);
  h_task_sdio = osThreadNew(StartSdioTask, NULL, &attrs_task_sdio);
  h_task_can = osThreadNew(StartCanTask, NULL, &attrs_task_can);
  h_task_kline = osThreadNew(StartKlineTask, NULL, &attrs_task_kline);
}
