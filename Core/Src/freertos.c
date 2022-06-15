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
  1,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
  3,3,3,3,3,3,3,3,3,3,2,3,3,3,3,3,3,3,3,3,
  3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,2,
  2,2,2,2,2,2,2,2,2,2,2,2,2,2,2
};

#define PARAMS_BUFFER_SIZE 32
static sParameters gParamsBuffer[2][PARAMS_BUFFER_SIZE];

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

struct sLogDriver {
    uint8_t initialized;
    uint8_t was_initialized;
    FRESULT fres;
    char filename[32];
    char string[128];
    uint16_t filenumber;
    UINT read;
    UINT wrote;
    sParameters parameters;
    uint64_t InitTime64;
    uint64_t diff;

    eLed led;
    FATFS *fatfs;
    FIL *file;
    const char *path;
    sProFIFO fifo;
};

static void driver_loop(struct sLogDriver *driver)
{
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

      driver->fres = f_mount(driver->fatfs, driver->path, 1);
      if(driver->fres != FR_OK) {
        osDelay(100);
        continue;
      }

      led_set(driver->led, LedLong);
      led_set_post(driver->led, LedOn);

      sprintf(driver->filename, "%s/last.txt", driver->path);
      driver->fres = f_open(driver->file, driver->filename, FA_OPEN_EXISTING | FA_READ);
      if(driver->fres == FR_OK) {
        if(f_size(driver->file) >= sizeof(driver->string)) {
          driver->fres = f_close(driver->file);
          if(driver->fres != FR_OK) { osDelay(100); continue; }
          driver->filenumber = 1;
        } else {
          driver->fres = f_read(driver->file, driver->string, f_size(driver->file), &driver->read);
          if(driver->fres != FR_OK) { f_close(driver->file); osDelay(100); continue; }
          driver->fres = f_close(driver->file);
          if(driver->fres != FR_OK) { osDelay(100); continue; }
          driver->string[driver->read] = 0;
          driver->fres = sscanf(driver->string, "%hu", &driver->filenumber);
          if(driver->fres == 1) {
            driver->filenumber++;
          }

        }
      } else if(driver->fres == FR_NO_FILE) {
        driver->filenumber = 1;
      } else { osDelay(100); continue; }

      driver->fres = f_open(driver->file, driver->filename, FA_CREATE_ALWAYS | FA_WRITE);
      if(driver->fres == FR_OK) {
        sprintf(driver->filename, "%05hu", driver->filenumber);
        driver->fres = f_write(driver->file, driver->filename, strlen(driver->filename), &driver->read);
        if(driver->fres != FR_OK) { f_close(driver->file); osDelay(100); continue; }
        driver->fres = f_sync(driver->file);
        if(driver->fres != FR_OK) { f_close(driver->file); osDelay(100); continue; }
        driver->fres = f_close(driver->file);
        if(driver->fres != FR_OK) { osDelay(100); continue; }
      } else {
        osDelay(100);
        continue;
      }

      sprintf(driver->filename, "%s/log_%05hu.eculog", driver->path, driver->filenumber);
      driver->fres = f_open(driver->file, driver->filename, FA_CREATE_ALWAYS | FA_WRITE);
      if(driver->fres != FR_OK) { osDelay(100); continue; }
      driver->fres = f_write(driver->file, gFileHeader, strlen(gFileHeader), &driver->wrote);
      if(driver->fres != FR_OK) { f_close(driver->file); osDelay(100); continue; }
      driver->fres = f_sync(driver->file);
      if(driver->fres != FR_OK) { f_close(driver->file); osDelay(100); continue; }

      driver->initialized = 1;
      driver->InitTime64 = gTick64;

      led_set(driver->led, LedOn);
      led_set_post(driver->led, LedOn);
    }

    if(protPull(&driver->fifo, &driver->parameters)) {
      driver->diff = gTick64 - driver->InitTime64;
      if(driver->diff > 0) {
        sprintf(driver->string, "%s000,", UINT64_TO_STR(driver->diff));
      } else {
        strcpy(driver->string, "0,");
      }
      driver->fres = f_write(driver->file, driver->string, strlen(driver->string), &driver->wrote);
      if(driver->fres != FR_OK) { f_close(driver->file); driver->initialized = 0; osDelay(100); continue; }
      for(int i = 0, j = 0; i < sizeof(driver->parameters) / sizeof(uint32_t); j++) {
        driver->string[0] = ',';
        driver->string[1] = '\0';
        switch(gFileFormats[j]) {
          case 1:
            sprintf(driver->string, "%s,", (const char *)&((uint32_t *)&driver->parameters)[i]);
            break;
          case 2:
            sprintf(driver->string, "%ld,", ((const int32_t *)&driver->parameters)[i]);
            break;
          case 3:
            sprintf(driver->string, "%f,", ((const float *)&driver->parameters)[i]);
            break;
          default:
            break;
        }
        if(i == 0) {
          i += TABLE_STRING_MAX / sizeof(uint32_t);
        } else {
          i++;
        }
        if(i >= sizeof(driver->parameters) / sizeof(uint32_t)) {
          driver->string[strlen(driver->string) - 1] = '\0';
        }
        driver->fres = f_write(driver->file, driver->string, strlen(driver->string), &driver->wrote);
        if(driver->fres != FR_OK) { f_close(driver->file); driver->initialized = 0; osDelay(100); continue; }

      }

      strcpy(driver->string, "\r\n");
      driver->fres = f_write(driver->file, driver->string, strlen(driver->string), &driver->wrote);
      if(driver->fres != FR_OK) { f_close(driver->file); driver->initialized = 0; osDelay(100); continue; }
      driver->fres = f_sync(driver->file);
      if(driver->fres != FR_OK) { f_close(driver->file); driver->initialized = 0; osDelay(100); continue; }

      led_set(driver->led, LedShortSingle);
    }

    osDelay(1);
  }
}

void StartUsbTask(void *argument)
{
  struct sLogDriver driver = {0};

  MX_USB_HOST_Init();

  driver.fatfs = &USBHFatFS;
  driver.file = &USBHFile;
  driver.path = USBHPath;
  driver.led = LedUsb;
  protInit(&driver.fifo, gParamsBuffer[0], sizeof(sParameters), PARAMS_BUFFER_SIZE);

  driver_loop(&driver);
}

void StartSdioTask(void *argument)
{
  struct sLogDriver driver = {0};

  driver.fatfs = &SDFatFS;
  driver.file = &SDFile;
  driver.path = SDPath;
  driver.led = LedSdio;
  protInit(&driver.fifo, gParamsBuffer[1], sizeof(sParameters), PARAMS_BUFFER_SIZE);

  driver_loop(&driver);
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
