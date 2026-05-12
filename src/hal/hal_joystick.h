#pragma once

#include "config.h"

#if HAS_BLUEPAD32
#include "app/app_mode.h"
#include <Bluepad32.h>

class HAL_Joystick {
public:
  HAL_Joystick(AppMode *app_mode, uint32_t period_ms, float deadzone = 0.08f);
  void task_loop();
  static void task_entry(void *pvParameters);

private:
  static HAL_Joystick *instance_;

  static void on_connected_callback(GamepadPtr gamepad);
  static void on_disconnected_callback(GamepadPtr gamepad);

  void on_connected(GamepadPtr gamepad);
  void on_disconnected(GamepadPtr gamepad);
  void setup_driver();
  void update_gamepad();

  AppMode *app_mode_;
  GamepadPtr gamepad_;
  uint32_t period_ms_;
  float deadzone_;
  float last_val_;
  float last_steer_;
  TickType_t last_push_tick_;
  uint32_t last_buttons_;
  bool was_connected_;
};
#endif  // HAS_BLUEPAD32
