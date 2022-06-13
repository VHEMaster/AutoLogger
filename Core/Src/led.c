/*
 * led.c
 *
 *  Created on: 13 июн. 2022 г.
 *      Author: VHEMaster
 */

#include "led.h"
#include <string.h>

#define TICK_DIFF(a,b) (((a)>(b))?((a)-(b)):((0xFFFFFFFF-(b))+(a)))

typedef struct {
   GPIO_TypeDef *port;
   uint16_t pin;
   uint8_t inverted;
   eLedStatus status;
   eLedStatus status_post;
   GPIO_PinState pinstate;

   uint32_t period;
   uint32_t last_tick;
}sLed;

static sLed gLeds[LedCount];
static volatile uint32_t gLedTick = 0;

void led_init(void)
{
  memset(gLeds, 0, sizeof(gLeds));
}

void led_register(eLed led, GPIO_TypeDef *port, uint16_t pin, eLedStatus status)
{
  sLed *pled;

  if(led < LedCount && port && pin) {
    pled = &gLeds[led];
    pled->port = port;
    pled->pin = pin;
    pled->inverted = 1;
    pled->status = status;

    if(status > 0) {
      pled->pinstate = GPIO_PIN_SET;
    }

    pled->port->BSRR = (pled->pinstate ^ pled->inverted) ? pled->pin : pled->pin << 16;
  }
}

void led_set(eLed led, eLedStatus status)
{
  if(led < LedCount && gLeds[led].port && gLeds[led].pin) {
    if(gLeds[led].status != status) {
      gLeds[led].status = status;

      if(status == LedOff) {
        gLeds[led].pinstate = GPIO_PIN_RESET;
      } else {
        gLeds[led].pinstate = GPIO_PIN_SET;
        if(status == LedShortSingle) {
          gLeds[led].period = 300;
        } else if(status == LedLongSingle) {
          gLeds[led].period = 1200;
        } else if(status == LedShort) {
          gLeds[led].period = 300;
        } else if(status == LedLong) {
          gLeds[led].period = 1200;
        }
      }

      gLeds[led].last_tick = gLedTick;
    }
  }
}

void led_set_post(eLed led, eLedStatus status_post)
{
  if(led < LedCount && gLeds[led].port && gLeds[led].pin) {
    gLeds[led].status_post = status_post;
  }
}

void led_irq(void)
{
  gLedTick++;

  for(int i = 0; i < LedCount; i++) {
    if(gLeds[i].port && gLeds[i].pin) {
      if(gLeds[i].status == LedOff) {
        gLeds[i].pinstate = GPIO_PIN_RESET;
      } else if(gLeds[i].status == LedOn) {
        gLeds[i].pinstate = GPIO_PIN_SET;
      } else {
        if(TICK_DIFF(gLedTick, gLeds[i].last_tick) < gLeds[i].period >> 1) {
          gLeds[i].pinstate = GPIO_PIN_SET;
        } else {
          gLeds[i].pinstate = GPIO_PIN_RESET;
          if(TICK_DIFF(gLedTick, gLeds[i].last_tick) >= gLeds[i].period) {
            if(gLeds[i].status == LedShortSingle || gLeds[i].status == LedLongSingle) {
              gLeds[i].status = gLeds[i].status_post;
            } else {
              gLeds[i].last_tick = gLedTick;
            }
          }
        }
      }
    }
  }

  for(int i = 0; i < LedCount; i++) {
    if(gLeds[i].port && gLeds[i].pin) {
      gLeds[i].port->BSRR = (gLeds[i].pinstate ^ gLeds[i].inverted) ? gLeds[i].pin : gLeds[i].pin << 16;
    }
  }
}
