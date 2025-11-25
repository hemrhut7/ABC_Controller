#ifndef HAL_LED_H
#define HAL_LED_H

#include <Arduino.h>


typedef enum {
  INITIALIZING,
  IMU_MEASURING,
  CONFIGURING,
} SYSTEM_STATE;

void init_led();
static void led_toggle();
void led_blink_1sec_block();
void set_led_state(SYSTEM_STATE state);
static void vLedTimerCallback(TimerHandle_t xTimerHandle);

#endif