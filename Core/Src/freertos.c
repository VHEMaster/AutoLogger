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

#define CAN_LOOPBACK() HAL_GPIO_WritePin(CAN1_LBK_GPIO_Port, CAN1_LBK_Pin, GPIO_PIN_SET)
#define CAN_NORMAL() HAL_GPIO_WritePin(CAN1_LBK_GPIO_Port, CAN1_LBK_Pin, GPIO_PIN_RESET)

typedef struct {
    union {
        uint8_t bytes[8];
        uint16_t words[4];
        uint32_t dwords[2];
        uint64_t qword;
    } data;
    uint32_t rtr;
    uint16_t id;
    uint8_t length;
    uint8_t pad;
}sCanMessage __attribute__((aligned(8)));

#define CAN_RX_BUFFER_COUNT (16)

static sProFIFO canrxfifo;
static sCanMessage canrxbuffer[CAN_RX_BUFFER_COUNT];


extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart1;

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
static sProFIFO gParamsFifo[2];

const osThreadAttr_t attrs_task_time = {
  .name = "taskTime",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
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
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t attrs_task_kline = {
  .name = "taskKline",
  .stack_size = 128 * 4,
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
    sParameters parameters;
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

      sprintf(driver->filename, "%s/last.txt", driver->path);
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
      driver->fres = f_write(driver->file, gFileHeader, strlen(gFileHeader), &driver->wrote);
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
      driver->diff = gTick64 - driver->InitTime64;
      if(driver->diff > 0) {
        sprintf(driver->string, "%s000,", UINT64_TO_STR(driver->diff));
      } else {
        strcpy(driver->string, "0,");
      }
      driver->fres = f_write(driver->file, driver->string, strlen(driver->string), &driver->wrote);
      if(driver->fres != FR_OK) { f_close(driver->file); driver->initialized = 0; continue; }
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
          strcat(driver->string, "\r\n");
        }
        driver->fres = f_write(driver->file, driver->string, strlen(driver->string), &driver->wrote);
        if(driver->fres != FR_OK) { f_close(driver->file); driver->initialized = 0; continue; }

      }

      if(gTick64 - driver->sync_last > 200) {
        driver->fres = f_sync(driver->file);
        if(driver->fres != FR_OK) { f_close(driver->file); driver->initialized = 0; continue; }
      }

      led_set(driver->led, LedShortSingle);
    } else {
      osDelay(5);
    }

  }
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
  protInit(driver.fifo, gParamsBuffer[0], sizeof(sParameters), PARAMS_BUFFER_SIZE);

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
  protInit(driver.fifo, gParamsBuffer[1], sizeof(sParameters), PARAMS_BUFFER_SIZE);

  driver_loop(&driver);
}

void can_rxfifopendingcallback(CAN_HandleTypeDef *_hcan, uint32_t fifo)
{
  CAN_RxHeaderTypeDef header;
  sCanMessage message = {0};
  HAL_StatusTypeDef status;

  if(_hcan == &hcan1) {
    status = HAL_CAN_GetRxMessage(_hcan, fifo, &header, message.data.bytes);
    if(status == HAL_OK) {
      message.id = header.StdId;
      message.length = header.DLC;
      message.rtr = header.RTR;
      protPush(&canrxfifo, &message);
    }
  }
}

void StartCanTask(void *argument)
{
  CAN_TxHeaderTypeDef header;
  sCanMessage message;
  HAL_StatusTypeDef status;
  CAN_FilterTypeDef can_filter;
  uint32_t free_level;
  uint32_t mailbox;
  uint8_t need_send = 0;
  uint64_t last_send = 0;
  uint64_t diff = 0;
  sParameters parameters = {0};
  uint32_t *params_ptr = (uint32_t *)&parameters;
  uint8_t msgs_bitmap[64];
  uint8_t isoktosend = 0;
  uint32_t pos_send = 0;
  uint8_t pinged = 0;

  uint16_t filter_id = 0x110;
  uint16_t filter_mask = 0x7F0;

  memset(msgs_bitmap, 0, sizeof(msgs_bitmap));

  CAN_LOOPBACK();

  led_set(LedCan, LedOn);
  led_set_post(LedCan, LedOn);

  protInit(&canrxfifo, canrxbuffer, sizeof(canrxbuffer[0]), ITEMSOF(canrxbuffer));

  can_filter.FilterActivation = CAN_FILTER_ENABLE;
  can_filter.FilterBank = 0;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterIdHigh = filter_id<<5;
  can_filter.FilterIdLow = 0;
  can_filter.FilterMaskIdHigh = filter_mask<<5;
  can_filter.FilterMaskIdLow = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.SlaveStartFilterBank = 13;

  status = HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  if(status != HAL_OK)
    goto can_error;

  status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
  if(status != HAL_OK)
    goto can_error;

  status = HAL_CAN_Start(&hcan1);
  if(status != HAL_OK)
    goto can_error;

  CAN_NORMAL();

  last_send = gTick64;

  for(;;)
  {
    if(protPull(&canrxfifo, &message)) {
      if(message.id == 0x112) {
        if(message.rtr == CAN_RTR_DATA && message.length == 8) {
          if(message.data.dwords[0] < sizeof(sParameters) / sizeof(uint32_t)) {
            led_set(LedCan, LedShort);
            params_ptr[message.data.dwords[0]] = message.data.dwords[1];
            msgs_bitmap[message.data.dwords[0] >> 3] |= (1 << (message.data.dwords[0] & 7));
            isoktosend = 1;
            pos_send = 0;
            for(int i = 0; i < sizeof(sParameters) / sizeof(uint32_t); i++) {
              if(msgs_bitmap[i >> 3] |= (1 << (i & 7)) == 0) {
                pos_send = i;
                isoktosend = 0;
                break;
              }
            }
            if(isoktosend) {
              isoktosend = 0;
              for(int i = 0; ITEMSOF(gParamsBuffer); i++) {
                protPush(&gParamsFifo[i], &parameters);
              }
              memset(msgs_bitmap, 0, sizeof(msgs_bitmap));
              pos_send = 0;
            }
            need_send = 1;
          }
          }
      } else if(message.id == 0x110) {
        if(message.rtr == CAN_RTR_DATA && message.length == 5) {
          if(message.data.bytes[0] == 0x10 &&
              message.data.bytes[1] == 0xB5 &&
              message.data.bytes[2] == 0xDA &&
              message.data.bytes[3] == 0x7F &&
              message.data.bytes[4] == 0x63) {
            led_set(LedCan, LedShort);
            pinged = 1;
          }
        }
      }
    }

    if(!need_send) {
      diff = gTick64 - last_send;
      if(diff > 50000)
        need_send = 1;
    }

    if(need_send) {
      free_level = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
      if(free_level) {

        if(pinged) {
          message.id = 0x102;
          message.length = 4;
          message.rtr = CAN_RTR_REMOTE;
          message.data.dwords[0] = pos_send;
        } else {
          message.id = 0x100;
          message.length = 5;
          message.rtr = CAN_RTR_REMOTE;
          message.data.bytes[0] = 0x10;
          message.data.bytes[1] = 0xB5;
          message.data.bytes[2] = 0xDA;
          message.data.bytes[3] = 0x7F;
          message.data.bytes[4] = 0x63;
        }

        header.IDE = CAN_ID_STD;
        header.StdId = message.id;
        header.RTR = message.rtr;
        header.DLC = message.length;

        status = HAL_CAN_AddTxMessage(&hcan1, &header, message.data.bytes, &mailbox);
        if(status == HAL_OK) {
          last_send = gTick64;
          need_send = 0;
          free_level &= ~(1 << mailbox);

          if(!pinged) {
            led_set(LedCan, LedShort);
          }
        }
      }
    }

    osDelay(1);
  }

  can_error:


  led_set(LedCan, LedLong);

  while(1)
  {
   osDelay(1000);
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
  h_task_time = osThreadNew(StartTimeTask, NULL, &attrs_task_time);
  h_task_usb = osThreadNew(StartUsbTask, NULL, &attrs_task_usb);
  h_task_sdio = osThreadNew(StartSdioTask, NULL, &attrs_task_sdio);
  h_task_can = osThreadNew(StartCanTask, NULL, &attrs_task_can);
  h_task_kline = osThreadNew(StartKlineTask, NULL, &attrs_task_kline);
}
