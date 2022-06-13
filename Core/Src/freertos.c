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

extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

/* Definitions for defaultTask */
osThreadId_t h_task_time;
osThreadId_t h_task_usb;
osThreadId_t h_task_sdio;
osThreadId_t h_task_can;
osThreadId_t h_task_kline;

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
  }
}

void StartUsbTask(void *argument)
{
  uint8_t state = 0;
  uint8_t state_prev = 0;
  uint8_t initialized = 0;
  FRESULT fres;

  MX_USB_HOST_Init();

  for(;;)
  {
    state = USBH_GetAppliState() == APPLICATION_READY;
    if(state) {
      if(!initialized) {
        fres = f_mount(&USBHFatFS, USBHPath, 1);
        if(fres != FR_OK) {
          osDelay(100);
          continue;
        }

        initialized = 1;
      }

    } else {
      led_set(LedUsb, LedOff);
      led_set_post(LedUsb, LedOff);

      if(state_prev != state && initialized) {
        f_mount(NULL, USBHPath, 0);
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

  for(;;)
  {
    if(!initialized && was_initialized) {
      f_mount(NULL, SDPath, 0);
      was_initialized = 0;
    }

    if(!initialized) {
      fres = f_mount(&SDFatFS, SDPath, 1);
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
