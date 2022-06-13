/*
 * led.h
 *
 *  Created on: 13 июн. 2022 г.
 *      Author: VHEMaster
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "main.h"

typedef enum {
  LedUsb,
  LedSdio,
  LedCan,
  LedKline,
  LedCount
}eLed;

typedef enum {
  LedOff = 0,
  LedOn,
  LedShortSingle,
  LedLongSingle,
  LedShort,
  LedLong
}eLedStatus;

void led_set(eLed led, eLedStatus status);
void led_set_post(eLed led, eLedStatus status_post);
void led_init(void);
void led_register(eLed led, GPIO_TypeDef *port, uint16_t pin, eLedStatus status);
void led_irq(void);

#endif /* INC_LED_H_ */
