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
    union {
        uint32_t u;
        float f;
        uint8_t b[4];
    } value;
}sParameter;

/*
 * 0 = uint64_t
 * 1 = char *
 * 2 = int32_t
 * 3 = float
 */

sParameter gParameters[] = {
    {"SwitchPosition",            2, OFFSETOF(sParameters, SwitchPosition), {0}},
    {"CurrentTable",              2, OFFSETOF(sParameters, CurrentTable), {0}},
    {"InjectorChannel",           2, OFFSETOF(sParameters, InjectorChannel), {0}},
    {"AdcKnockVoltage",           3, OFFSETOF(sParameters, AdcKnockVoltage), {0}},
    {"AdcAirTemp",                3, OFFSETOF(sParameters, AdcAirTemp), {0}},
    {"AdcEngineTemp",             3, OFFSETOF(sParameters, AdcEngineTemp), {0}},
    {"AdcManifoldAirPressure",    3, OFFSETOF(sParameters, AdcManifoldAirPressure), {0}},
    {"AdcThrottlePosition",       3, OFFSETOF(sParameters, AdcThrottlePosition), {0}},
    {"AdcPowerVoltage",           3, OFFSETOF(sParameters, AdcPowerVoltage), {0}},
    {"AdcReferenceVoltage",       3, OFFSETOF(sParameters, AdcReferenceVoltage), {0}},
    {"AdcLambdaUR",               3, OFFSETOF(sParameters, AdcLambdaUR), {0}},
    {"AdcLambdaUA",               3, OFFSETOF(sParameters, AdcLambdaUA), {0}},
    {"KnockSensor",               3, OFFSETOF(sParameters, KnockSensor), {0}},
    {"KnockSensorFiltered",       3, OFFSETOF(sParameters, KnockSensorFiltered), {0}},
    {"KnockSensorDetonate",       3, OFFSETOF(sParameters, KnockSensorDetonate), {0}},
    {"KnockZone",                 3, OFFSETOF(sParameters, KnockZone), {0}},
    {"KnockAdvance",              3, OFFSETOF(sParameters, KnockAdvance), {0}},
    {"KnockCount",                2, OFFSETOF(sParameters, KnockCount), {0}},
    {"AirTemp",                   3, OFFSETOF(sParameters, AirTemp), {0}},
    {"EngineTemp",                3, OFFSETOF(sParameters, EngineTemp), {0}},
    {"ManifoldAirPressure",       3, OFFSETOF(sParameters, ManifoldAirPressure), {0}},
    {"ThrottlePosition",          3, OFFSETOF(sParameters, ThrottlePosition), {0}},
    {"ReferenceVoltage",          3, OFFSETOF(sParameters, ReferenceVoltage), {0}},
    {"PowerVoltage",              3, OFFSETOF(sParameters, PowerVoltage), {0}},
    {"FuelRatio",                 3, OFFSETOF(sParameters, FuelRatio), {0}},
    {"FuelRatioDiff",             3, OFFSETOF(sParameters, FuelRatioDiff), {0}},
    {"LambdaValue",               3, OFFSETOF(sParameters, LambdaValue), {0}},
    {"LambdaTemperature",         3, OFFSETOF(sParameters, LambdaTemperature), {0}},
    {"LambdaHeaterVoltage",       3, OFFSETOF(sParameters, LambdaHeaterVoltage), {0}},
    {"LambdaTemperatureVoltage",  3, OFFSETOF(sParameters, LambdaTemperatureVoltage), {0}},
    {"ShortTermCorrection",       3, OFFSETOF(sParameters, ShortTermCorrection), {0}},
    {"LongTermCorrection",        3, OFFSETOF(sParameters, LongTermCorrection), {0}},
    {"IdleCorrection",            3, OFFSETOF(sParameters, IdleCorrection), {0}},
    {"IdleFlag",                  2, OFFSETOF(sParameters, IdleFlag), {0}},
    {"IdleCorrFlag",              2, OFFSETOF(sParameters, IdleCorrFlag), {0}},
    {"IdleEconFlag",              2, OFFSETOF(sParameters, IdleEconFlag), {0}},
    {"RPM",                       3, OFFSETOF(sParameters, RPM), {0}},
    {"Speed",                     3, OFFSETOF(sParameters, Speed), {0}},
    {"Acceleration",              3, OFFSETOF(sParameters, Acceleration), {0}},
    {"MassAirFlow",               3, OFFSETOF(sParameters, MassAirFlow), {0}},
    {"CyclicAirFlow",             3, OFFSETOF(sParameters, CyclicAirFlow), {0}},
    {"EffectiveVolume",           3, OFFSETOF(sParameters, EffectiveVolume), {0}},
    {"AirDestiny",                3, OFFSETOF(sParameters, AirDestiny), {0}},
    {"RelativeFilling",           3, OFFSETOF(sParameters, RelativeFilling), {0}},
    {"WishFuelRatio",             3, OFFSETOF(sParameters, WishFuelRatio), {0}},
    {"IdleValvePosition",         3, OFFSETOF(sParameters, IdleValvePosition), {0}},
    {"IdleRegThrRPM",             3, OFFSETOF(sParameters, IdleRegThrRPM), {0}},
    {"WishIdleRPM",               3, OFFSETOF(sParameters, WishIdleRPM), {0}},
    {"WishIdleMassAirFlow",       3, OFFSETOF(sParameters, WishIdleMassAirFlow), {0}},
    {"WishIdleValvePosition",     3, OFFSETOF(sParameters, WishIdleValvePosition), {0}},
    {"WishIdleIgnitionAngle",     3, OFFSETOF(sParameters, WishIdleIgnitionAngle), {0}},
    {"IgnitionAngle",             3, OFFSETOF(sParameters, IgnitionAngle), {0}},
    {"InjectionPhase",            3, OFFSETOF(sParameters, InjectionPhase), {0}},
    {"InjectionPhaseDuration",    3, OFFSETOF(sParameters, InjectionPhaseDuration), {0}},
    {"InjectionPulse",            3, OFFSETOF(sParameters, InjectionPulse), {0}},
    {"InjectionDutyCycle",        3, OFFSETOF(sParameters, InjectionDutyCycle), {0}},
    {"InjectionEnrichment",       3, OFFSETOF(sParameters, InjectionEnrichment), {0}},
    {"InjectionLag",              3, OFFSETOF(sParameters, InjectionLag), {0}},
    {"IgnitionPulse",             3, OFFSETOF(sParameters, IgnitionPulse), {0}},
    {"IdleSpeedShift",            3, OFFSETOF(sParameters, IdleSpeedShift), {0}},
    {"DrivenKilometers",          3, OFFSETOF(sParameters, DrivenKilometers), {0}},
    {"FuelConsumed",              3, OFFSETOF(sParameters, FuelConsumed), {0}},
    {"FuelConsumption",           3, OFFSETOF(sParameters, FuelConsumption), {0}},
    {"FuelHourly",                3, OFFSETOF(sParameters, FuelHourly), {0}},
    {"TspsRelativePosition",      3, OFFSETOF(sParameters, TspsRelativePosition), {0}},
    {"LambdaValid",               2, OFFSETOF(sParameters, LambdaValid), {0}},
    {"OilSensor",                 2, OFFSETOF(sParameters, OilSensor), {0}},
    {"FanForceSwitch",            2, OFFSETOF(sParameters, FanForceSwitch), {0}},
    {"HandbrakeSensor",           2, OFFSETOF(sParameters, HandbrakeSensor), {0}},
    {"ChargeSensor",              2, OFFSETOF(sParameters, ChargeSensor), {0}},
    {"ClutchSensor",              2, OFFSETOF(sParameters, ClutchSensor), {0}},
    {"IgnSensor",                 2, OFFSETOF(sParameters, IgnSensor), {0}},
    {"FuelPumpRelay",             2, OFFSETOF(sParameters, FuelPumpRelay), {0}},
    {"FanRelay",                  2, OFFSETOF(sParameters, FanRelay), {0}},
    {"CheckEngine",               2, OFFSETOF(sParameters, CheckEngine), {0}},
    {"StarterRelay",              2, OFFSETOF(sParameters, StarterRelay), {0}},
    {"FanSwitch",                 2, OFFSETOF(sParameters, FanSwitch), {0}},
    {"IgnOutput",                 2, OFFSETOF(sParameters, IgnOutput), {0}},
    {"StartAllowed",              2, OFFSETOF(sParameters, StartAllowed), {0}},
    {"IsRunning",                 2, OFFSETOF(sParameters, IsRunning), {0}},
    {"IsCheckEngine",             2, OFFSETOF(sParameters, IsCheckEngine), {0}}

};

#define BIT_GET(array,bit) ((array)[(bit) >> 3] & (1 << ((bit) & 7)))
#define BIT_SET(array,bit)   ((array)[(bit) >> 3] |= (1 << ((bit) & 7)))
#define BIT_RESET(array,bit)   ((array)[(bit) >> 3] &= ~(1 << ((bit) & 7)))
static uint8_t gConfigBitmap[16] = {0};

const char *gFileInitialHeader = "TimePoint";

#define PARAMS_BUFFER_SIZE 52
static sParameters gParamsBuffer[2][PARAMS_BUFFER_SIZE];
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
                  if(!strnstr(gParameters[j].name, driver->string, strlen(gParameters[j].name))) {
                    BIT_SET(gConfigBitmap, j);
                    break;
                  }
                }
                readpos = 0;
              }
            } else {
              readpos++;
            }
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
      driver->diff = gTick64 - driver->InitTime64;
      if(driver->diff > 0) {
        sprintf(driver->string, "%s000,", UINT64_TO_STR(driver->diff));
      } else {
        strcpy(driver->string, "0,");
      }
      driver->fres = f_write(driver->file, driver->string, strlen(driver->string), &driver->wrote);
      if(driver->fres != FR_OK) { f_close(driver->file); driver->initialized = 0; continue; }
      for(int i = 0; i < ITEMSOF(gParameters);) {
        if(BIT_GET(gConfigBitmap, i)) {
          strcpy(driver->string, ",");
          const void *ptr = ((uint8_t *)&driver->parameters) + gParameters[i].offset;
          switch(gParameters[i].type) {
            case 1:
              sprintf(driver->string, "%s,", (const char *)ptr);
              break;
            case 2:
              sprintf(driver->string, "%ld,", *(const int32_t *)ptr);
              break;
            case 3:
              sprintf(driver->string, "%f,", *(const float *)ptr);
              break;
            default:
              break;
          }

          if(++i >= ITEMSOF(gParameters)) {
            driver->string[strlen(driver->string) - 1] = '\0';
            strcat(driver->string, "\r\n");
          }
          driver->fres = f_write(driver->file, driver->string, strlen(driver->string), &driver->wrote);
          if(driver->fres != FR_OK) { f_close(driver->file); driver->initialized = 0; continue; }
        }
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
  uint64_t last_iwdg = 0;
  uint64_t diff = 0;
  sParameters parameters = {0};
  uint32_t *params_ptr = (uint32_t *)&parameters;
  uint8_t msgs_bitmap[16];
  uint8_t isoktosend = 0;
  uint32_t pos_send = 0;
  uint8_t pinged = 0;

  uint16_t filter_id = 0x200;
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
  can_filter.SlaveStartFilterBank = 14;

  status = HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  if(status != HAL_OK)
    goto can_error;

  status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
  if(status != HAL_OK)
    goto can_error;

  status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR);
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
      if(message.id == 0x202) {
        if(message.rtr == CAN_RTR_DATA && message.length == 8) {
          if(message.data.dwords[0] < sizeof(sParameters) / sizeof(uint32_t)) {
            led_set(LedCan, LedShort);
            params_ptr[message.data.dwords[0]] = message.data.dwords[1];
            BIT_SET(msgs_bitmap, message.data.dwords[0]);
            isoktosend = 1;
            pos_send = 0;
            for(int i = 0; i < ITEMSOF(gParameters); i++) {
              if(BIT_GET(gConfigBitmap, i) && !BIT_GET(msgs_bitmap, gParameters[i].offset / sizeof(uint32_t))) {
                pos_send = i;
                isoktosend = 0;
                break;
              }
            }
            if(isoktosend) {
              isoktosend = 0;
              for(int i = 0; i < sizeof(msgs_bitmap); i++) {
                if(msgs_bitmap[i]) {
                  isoktosend = 1;
                  break;
                }
              }
              if(isoktosend) {
                for(int i = 0; i < ITEMSOF(gParamsFifo); i++) {
                  if(gParamsFifo[i].buffer && gParamsFifo[i].info.capacity) {
                    protPush(&gParamsFifo[i], &parameters);
                  }
                }
                memset(msgs_bitmap, 0, sizeof(msgs_bitmap));
                pos_send = 0;
              }
            }
            need_send = 1;
          }
          }
      } else if(message.id == 0x200) {
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
      if(diff > 50)
        need_send = 1;
    }

    if(need_send) {
      free_level = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
      if(free_level) {

        if(pinged) {
          message.id = 0x102;
          message.length = 4;
          message.rtr = CAN_RTR_DATA;
          message.data.dwords[0] = pos_send;
        } else {
          message.id = 0x100;
          message.length = 5;
          message.rtr = CAN_RTR_DATA;
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
    diff = gTick64 - last_iwdg;
    if(diff > 100) {
      HAL_IWDG_Refresh(&hiwdg);
      last_iwdg = gTick64;
    }
  }

  can_error:


  led_set(LedCan, LedLong);

  while(1)
  {
   HAL_IWDG_Refresh(&hiwdg);
   osDelay(100);
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
  h_task_usb = osThreadNew(StartUsbTask, NULL, &attrs_task_usb);
  h_task_sdio = osThreadNew(StartSdioTask, NULL, &attrs_task_sdio);
  h_task_can = osThreadNew(StartCanTask, NULL, &attrs_task_can);
  h_task_kline = osThreadNew(StartKlineTask, NULL, &attrs_task_kline);
}
