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

#if HAS_WIFI_SERIAL
#include <WiFi.h>
#include "hal/hal_udp_stream.h"
#endif

#if HAS_BLUEPAD32
#include "hal/hal_joystick.h"
#endif

#define PRIORITY_CONTROL 24
#define PRIORITY_AHRS 15
#define PRIORITY_UART1 12
#define PRIORITY_COMM 10
#define PRIORITY_GAMEPAD 8

#define PERIOD_CONTROLL 5 // 200Hz
#define PERIOD_COMM 5     // 200Hz
#define PERIOD_GAMEPAD 10 // 100Hz

TaskHandle_t ControlTaskHandle;
TaskHandle_t AHRSTaskHandle;
TaskHandle_t UART1TaskHandle;
TaskHandle_t CommTaskHandle;
TaskHandle_t GamepadTaskHandle;
#if HAS_WIFI_SERIAL
TaskHandle_t WiFiTaskHandle;
#endif

ConfigStore config_store;
Processing_Motor motor(PERIOD_CONTROLL);
Processing_AHRS ahrs(PERIOD_CONTROLL);
AppMode app_mode(&motor, &config_store, PERIOD_CONTROLL);

AppScript app_script(&app_mode);

void set_app_pending_mode(Mode_t mode) {
  app_mode.set_pending_mode(mode);
}

HAL_LED system_led; // Virtual LED
HAL_Display system_display;
HAL_Battery system_battery;
Failsafe failsafe(PERIOD_CONTROLL, &motor, &system_led, set_app_pending_mode);

Telemetry uart_telemetry(Serial, PORT_USB);
Telemetry uart1_telemetry(Serial1, PORT_UART1);

#if HAS_BLUEPAD32
HAL_Joystick joystick(&app_mode, PERIOD_GAMEPAD);
#endif

#if HAS_WIFI_SERIAL
UDPStream udp_stream(UDP_PORT);
Telemetry udp_telemetry(udp_stream, PORT_WIFI);
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

    // 2. Push fresh data to the telemetry queues
    system_battery.update(dt);
    current_sys_state.battery_v = system_battery.get_voltage();
    current_sys_state.delay_count = failsafe.get_delay_count();
    current_sys_state.cmd.mode = app_mode.get_mode();
    current_sys_state.abc_state.velocity = app_mode.get_velocity();
    current_sys_state.pid_target = app_mode.get_pid_target();

    uart_telemetry.push_data(current_sys_state);
    uart1_telemetry.push_data(current_sys_state); // Push to UART1 too

#if HAS_WIFI_SERIAL
    udp_telemetry.push_data(current_sys_state);
#endif

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 2. 通訊與管理任務 (Core 0)
void UART1_Task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_COMM);
  for (;;) {
    uart1_telemetry.process_serial_outgoing();
    app_script.check_serial(Serial1, &uart1_telemetry);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 3. 通訊與管理任務 (Core 0)
void Comm_Task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_COMM);
  for (;;) {
    uart_telemetry.process_serial_outgoing();
    app_script.check_serial(Serial, &uart_telemetry);

    // system display: LED, Monitor
    system_led.update();
    system_display.update(app_mode.get_mode(), app_mode.get_pending_mode(), system_battery.get_voltage(), failsafe.get_delay_count());

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

#if HAS_WIFI_SERIAL
// 4. WiFi 遙測發送任務 (Core 0)
void WiFi_Task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_COMM);
  for (;;) {
    // 偵測是否收到第一個 UDP 封包以鎖定目標 IP
    int len = udp_stream.available();

    if (udp_stream.connected()) {
      udp_telemetry.process_serial_outgoing();
      if (len > 0) {
        app_script.check_serial(udp_stream, &udp_telemetry);
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
#endif

void setup() {
  system_led.set_state(INITIALIZING);

  Serial.begin(115200);
  // Serial1 (Telemetry/Script)
  Serial1.begin(230400, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
  // Serial2 (Sensor RX only)
  Serial2.begin(115200, SERIAL_8N1, UART2_RX_PIN, -1);

  config_store.begin();
  ahrs.init();
  motor.init();
  uart_telemetry.init(1000 / PERIOD_COMM);
  uart1_telemetry.init(1000 / PERIOD_COMM);
#if HAS_WIFI_SERIAL
  udp_telemetry.init(1000 / PERIOD_COMM);

  // WiFi Initialization
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  uint8_t timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed. Starting AP mode...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }
  udp_stream.begin();
#endif

  app_mode.init();
  system_display.init();
  system_battery.init();
  app_mode.set_mode(MODE_FREE);

  xTaskCreatePinnedToCore(Control_Task, "ControlTask", 8192, NULL,
                          PRIORITY_CONTROL, &ControlTaskHandle, 1);

  xTaskCreatePinnedToCore(UART1_Task, "UART1Task", 4096, NULL, PRIORITY_UART1,
                          &UART1TaskHandle, 0);

  xTaskCreatePinnedToCore(Comm_Task, "CommTask", 4096, NULL, PRIORITY_COMM,
                          &CommTaskHandle, 0);

#if HAS_BLUEPAD32
  xTaskCreatePinnedToCore(HAL_Joystick::task_entry, "Gamepad_Task", 6144,
                          &joystick,
                          PRIORITY_GAMEPAD, &GamepadTaskHandle, 0);
#endif

#if HAS_WIFI_SERIAL
  xTaskCreatePinnedToCore(WiFi_Task, "WiFi_Task", 4096, NULL, PRIORITY_COMM,
                          &WiFiTaskHandle, 0);
#endif
}

void loop() { vTaskDelete(NULL); }
