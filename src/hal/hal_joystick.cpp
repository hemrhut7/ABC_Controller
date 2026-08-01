#include "hal_joystick.h"
#include <Arduino.h>
#include <math.h>

#if HAS_BLUEPAD32

HAL_Joystick *HAL_Joystick::instance_ = nullptr;

namespace {
float normalize_axis(int32_t raw) {
  constexpr float kAxisMax = 512.0f;
  float normalized = static_cast<float>(raw) / kAxisMax;
  if (normalized > 1.0f)
    normalized = 1.0f;
  if (normalized < -1.0f)
    normalized = -1.0f;
  return normalized;
}

float apply_deadzone(float input, float deadzone) {
  float abs_input = fabsf(input);
  if (abs_input <= deadzone)
    return 0.0f;

  float sign = (input >= 0.0f) ? 1.0f : -1.0f;
  return sign * ((abs_input - deadzone) / (1.0f - deadzone));
}
} // namespace

HAL_Joystick::HAL_Joystick(AppMode *app_mode, uint32_t period_ms, float deadzone)
    : app_mode_(app_mode), gamepad_(nullptr), period_ms_(period_ms),
      deadzone_(deadzone), last_val_(0.0f), last_steer_(0.0f), last_push_tick_(0),
      last_buttons_(0), was_connected_(false) {
  mutex_ = xSemaphoreCreateMutex();
}

void HAL_Joystick::setup_driver() {
  instance_ = this;
  BP32.setup(&HAL_Joystick::on_connected_callback,
             &HAL_Joystick::on_disconnected_callback);
}

void HAL_Joystick::on_connected_callback(GamepadPtr gamepad) {
  if (instance_ != nullptr) {
    instance_->on_connected(gamepad);
  }
}

void HAL_Joystick::on_disconnected_callback(GamepadPtr gamepad) {
  if (instance_ != nullptr) {
    instance_->on_disconnected(gamepad);
  }
}

void HAL_Joystick::on_connected(GamepadPtr gamepad) {
  if (mutex_ != nullptr && xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
    if (gamepad_ == nullptr || !gamepad_->isConnected()) {
      gamepad_ = gamepad;
      app_mode_->enqueue_target(0.0f, 0.0f);
    }
    xSemaphoreGive(mutex_);
  }
}

void HAL_Joystick::on_disconnected(GamepadPtr gamepad) {
  if (mutex_ != nullptr && xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
    if (gamepad_ == gamepad) {
      gamepad_ = nullptr;
      last_buttons_ = 0;
    }
    xSemaphoreGive(mutex_);
  }
  app_mode_->enqueue_mode(MODE_REMOTE);
  app_mode_->enqueue_target(0.0f, 0.0f);
}

void HAL_Joystick::update_gamepad() {
  constexpr float kChangeThreshold = 0.02f;
  const TickType_t kMinPushInterval = pdMS_TO_TICKS(30);

  BP32.update();

  if (mutex_ == nullptr || xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
    return;
  }

  bool connected = (gamepad_ != nullptr) && gamepad_->isConnected();
  if (!connected) {
    if (gamepad_ != nullptr) {
      gamepad_ = nullptr; // Clear stale pointer immediately to allow reconnection
    }
    if (was_connected_) {
      app_mode_->enqueue_target(0.0f, 0.0f);
      app_mode_->enqueue_mode(MODE_REMOTE);
      last_val_ = 0.0f;
      last_steer_ = 0.0f;
      last_push_tick_ = xTaskGetTickCount();
      last_buttons_ = 0;
    }
    was_connected_ = false;
    xSemaphoreGive(mutex_);
    return;
  }

  was_connected_ = true;

  uint32_t current_buttons = gamepad_->buttons();
  uint32_t pressed_buttons = current_buttons & ~last_buttons_;
  last_buttons_ = current_buttons;

  if (pressed_buttons & BUTTON_X) {
    app_mode_->enqueue_mode(MODE_FREE);
  } else if (pressed_buttons & BUTTON_B) {
    app_mode_->enqueue_mode(MODE_REMOTE);
  } else if (pressed_buttons & BUTTON_A) {
    app_mode_->enqueue_mode(MODE_ANGLE);
  } else if (pressed_buttons & BUTTON_Y) {
    app_mode_->enqueue_mode(MODE_AUTO);
  }

  float val = -apply_deadzone(normalize_axis(gamepad_->axisY()), deadzone_);
  float steer = apply_deadzone(normalize_axis(gamepad_->axisX()), deadzone_);

  TickType_t now = xTaskGetTickCount();
  bool changed = (fabsf(val - last_val_) >= kChangeThreshold) ||
                 (fabsf(steer - last_steer_) >= kChangeThreshold);
  bool period_reached = (now - last_push_tick_) >= kMinPushInterval;

  if (changed || period_reached) {
    // In MODE_AUTO, reject joystick target commands (micro-ROS has control)
    if (app_mode_->get_mode() == MODE_AUTO) {
      xSemaphoreGive(mutex_);
      return;
    }

    float scaled_val = val;
    float scaled_steer = steer;
    switch (app_mode_->get_mode())
    {
    case MODE_MOTOR:
        scaled_val *= MAX_RPM;
        break;
    case MODE_PWM:
        scaled_val *= MAX_PWM_DUTY;
        break;
    case MODE_ANGLE:
        scaled_val *= MAX_PITCH * RAD_TO_DEG;
        break;
    case MODE_VELOCITY:
    case MODE_REMOTE:
        scaled_val *= MAX_VELOCITY;
        break;
    default:
        break;
    }
    if (app_mode_->enqueue_target(scaled_val, scaled_steer)) {
      last_val_ = val;
      last_steer_ = steer;
      last_push_tick_ = now;
    }
  }

  xSemaphoreGive(mutex_);
}

void HAL_Joystick::task_loop() {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(period_ms_);

  setup_driver();
  for (;;) {
    update_gamepad();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void HAL_Joystick::task_entry(void *pvParameters) {
  HAL_Joystick *joystick = static_cast<HAL_Joystick *>(pvParameters);
  if (joystick == nullptr) {
    vTaskDelete(nullptr);
    return;
  }
  joystick->task_loop();
}
#endif  // HAS_BLUEPAD32
