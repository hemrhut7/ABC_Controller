#include "config.h"
#include "app/app_fail_safe.h"
#include "app/app_mode.h"
#include "app/app_script.h"
#include "hal/hal_led.h"
#include "hal/hal_storage.h"
#include "hal/hal_telemetry.h"
#include "processing/prs_ahrs.h"
#include "processing/prs_motor.h"
#include "hal/hal_display.h"
#include "hal/hal_battery.h"
#include <Arduino.h>


#if HAS_BT_SERIAL
#include <BluetoothSerial.h>
#else
#include "hal/hal_joystick.h"
#endif

#define PRIORITY_CONTROL 24
#define PRIORITY_AHRS 15
#define PRIORITY_COMM 10
#define PRIORITY_GAMEPAD 8
#define PRIORITY_BT 5

#define PERIOD_CONTROLL 5 // 200Hz
#define PERIOD_COMM 5     // 200Hz
#define PERIOD_GAMEPAD 10 // 100Hz
#define PERIOD_BT 5       // 200Hz

TaskHandle_t ControlTaskHandle;
TaskHandle_t AHRSTaskHandle;
TaskHandle_t CommTaskHandle;
TaskHandle_t GamepadTaskHandle;
TaskHandle_t BTTaskHandle;

ConfigStore config_store;
Processing_Motor motor(PERIOD_CONTROLL);
Processing_AHRS ahrs(PERIOD_CONTROLL);
AppMode app_mode(&motor, &config_store, PERIOD_CONTROLL);

AppScript app_script(&app_mode);

void set_app_pending_mode(Mode_t mode) {
  app_mode.set_pending_mode(mode);
}

HAL_LED system_led(LED_BUILTIN);
HAL_Display system_display;
HAL_Battery system_battery;
Failsafe failsafe(PERIOD_CONTROLL, &motor, &system_led, set_app_pending_mode);

Telemetry uart_telemetry(Serial, PORT_USB);
#if HAS_BT_SERIAL
BluetoothSerial SerialBT;
Telemetry bt_telemetry(SerialBT, PORT_BT);
#else
HAL_Joystick joystick(&app_mode, PERIOD_GAMEPAD);
Telemetry bt_telemetry(Serial, PORT_BT);
#endif


// 1. 控制任務 (Core 1)
void Control_Task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_CONTROLL);
  unsigned long last_micros = micros();

  for (;;) {
    unsigned long current_micros = micros();
    float dt = (current_micros - last_micros) * 1e-6f;
    last_micros = current_micros;
    system_state_t current_sys_state;

    // 簡單的濾波/保護，避免 dt 異常 (例如第一次執行或溢位)
    if (dt <= 0.0f || dt > 0.1f)
      dt = PERIOD_CONTROLL * 0.001f;

    // Update IMU/AHRS state
    ahrs.update();
    ahrs.get_ahrs_data(&current_sys_state.abc_state.ahrs_data);

    // update Motor State
    motor.update_rpms(dt);
    motor.get_motor_state(&current_sys_state.abc_state.motor_state);

    app_mode.update(dt, current_sys_state.abc_state.ahrs_data);

    if (!failsafe.check(current_sys_state.abc_state.ahrs_data, ahrs.is_ready())) {
      app_mode.set_mode(MODE_FREE);

      if (failsafe.get_error_state() == FS_ERROR_LOOP_SLOW) {
        ahrs.reset_att();
      }
    } 
    // else if ((app_mode.get_mode() == MODE_FREE) && failsafe.is_ready_auto_start()){
    //   app_mode.set_mode(MODE_REMOTE);
    // }

    // 2. Push fresh data to the telemetry queues (non-blocking)
    system_battery.update(dt);
    current_sys_state.battery_v = system_battery.get_voltage();
    current_sys_state.delay_count = failsafe.get_delay_count();
    current_sys_state.cmd.mode = app_mode.get_mode();
    current_sys_state.abc_state.velocity = app_mode.get_velocity();
    current_sys_state.pid_target = app_mode.get_pid_target();

    uart_telemetry.push_data(current_sys_state);
    bt_telemetry.push_data(current_sys_state);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 2. 通訊與管理任務 (Core 0)
void Comm_Task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_COMM);
  for (;;) {
    uart_telemetry.process_serial_outgoing();
    app_script.check_serial(Serial, &uart_telemetry);
    system_led.update();
    system_display.update(app_mode.get_mode(), app_mode.get_pending_mode(), system_battery.get_voltage(), failsafe.get_delay_count());

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

#if HAS_BT_SERIAL
// 3. 藍牙遙測發送任務 (Core 0)
void BT_Task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_BT);
  for (;;) {
    bt_telemetry.process_serial_outgoing();
    app_script.check_serial(SerialBT, &bt_telemetry);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
#endif

void setup() {
  system_led.set_state(INITIALIZING);

  Serial.begin(115200);
#if HAS_BT_SERIAL
  SerialBT.begin("ABC_Controller");
#endif
  config_store.begin();
  ahrs.init();
  motor.init();
  uart_telemetry.init(1000 / PERIOD_COMM);
  bt_telemetry.init(1000 / PERIOD_BT);

  app_mode.init();
  system_display.init();
  system_battery.init();
  app_mode.set_mode(MODE_FREE);

  xTaskCreatePinnedToCore(Control_Task, "ControlTask", 8192, NULL,
                          PRIORITY_CONTROL, &ControlTaskHandle, 1);

  xTaskCreatePinnedToCore(Comm_Task, "CommTask", 4096, NULL, PRIORITY_COMM,
                          &CommTaskHandle, 0);

#if HAS_BT_SERIAL
  xTaskCreatePinnedToCore(BT_Task, "BT_Task", 4096, NULL, PRIORITY_BT,
                          &BTTaskHandle, 0);
#else
  xTaskCreatePinnedToCore(HAL_Joystick::task_entry, "Gamepad_Task", 6144,
                          &joystick,
                          PRIORITY_GAMEPAD, &GamepadTaskHandle, 0);
#endif
}

void loop() { vTaskDelete(NULL); }
