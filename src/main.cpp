#include "config.h"
#include "app/app_fail_safe.h"
#include "app/app_mode.h"
#include "app/app_script.h"
#include "hal/hal_storage.h"
#include "hal/hal_telemetry.h"
#include "processing/prs_ahrs.h"
#include "processing/prs_motor.h"
#include "hal/hal_display.h"
#include "hal/hal_battery.h"
#include "hal/hal_imu.h"
#include "processing/prs_i2c_sensor.h"

#if HAS_WIFI_SERIAL
#include <WiFi.h>
#include "hal/hal_udp_stream.h"
#endif
#include "hal/hal_microros.h"


#if HAS_BLUEPAD32
#include "hal/hal_joystick.h"
#endif

#define PRIORITY_CONTROL 24

#define PRIORITY_UART1 14
#define PRIORITY_WIFI 12
#define PRIORITY_COMM 12
#define PRIORITY_GAMEPAD 13

#define MAIN_LOOP_RATE_HZ  200.0f
#define PERIOD_CONTROL    1000.0f/MAIN_LOOP_RATE_HZ
#define PERIOD_COMM        2  // 500Hz
#define PERIOD_GAMEPAD     40 // 25Hz

TaskHandle_t ControlTaskHandle;
TaskHandle_t UART1TaskHandle;
TaskHandle_t CommTaskHandle;
TaskHandle_t GamepadTaskHandle;
#if HAS_WIFI_SERIAL
TaskHandle_t WiFiTaskHandle;
#endif

#include "esp_timer.h"
static SemaphoreHandle_t control_timer_sem = nullptr;
static void IRAM_ATTR control_timer_callback(void* arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (control_timer_sem) {
    xSemaphoreGiveFromISR(control_timer_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
}

ConfigStore config_store;
Processing_Motor motor(PERIOD_CONTROL);
Processing_AHRS ahrs(PERIOD_CONTROL);
Processing_I2CSensor i2c_sensor(MAIN_LOOP_RATE_HZ);
AppMode app_mode(&motor, &config_store, PERIOD_CONTROL);

AppScript app_script(&app_mode);

void set_app_pending_mode(Mode_t mode) {
  app_mode.set_pending_mode(mode);
}

HAL_Display system_display;
HAL_Battery system_battery;
Failsafe failsafe(PERIOD_CONTROL, &motor, set_app_pending_mode);
HAL_MicroROS uros_telemetry;

#if HAS_BLUEPAD32
HAL_Joystick joystick(&app_mode, PERIOD_GAMEPAD);
#endif

#if HAS_WIFI_SERIAL
UDPStream udp_stream(UDP_PORT);
#endif

void Control_Task(void *pvParameters) {
  unsigned long last_micros = micros();

  for (;;) {
    if (control_timer_sem) {
      xSemaphoreTake(control_timer_sem, portMAX_DELAY);
    } else {
      vTaskDelay(pdMS_TO_TICKS(2));
    }

    unsigned long current_micros = micros();
    float dt = (current_micros - last_micros) * 1e-6f;
    last_micros = current_micros;
    system_state_t current_sys_state;

    if (dt <= 0.0f || dt > 0.1f)
      dt = PERIOD_CONTROL * 0.001f;

    // 分頻無鎖讀取磁力計與氣壓計
    i2c_sensor.update();

    // 同步更新 IMU 數據與姿態估計
    ahrs.update();
    ahrs.get_ahrs_data(&current_sys_state.abc_state.ahrs_data);

    // update Motor State
    motor.update_rpms(dt);
    motor.get_motor_state(&current_sys_state.abc_state.motor_state);

    app_mode.update(dt, current_sys_state.abc_state.ahrs_data);

    if (!failsafe.check(current_sys_state.abc_state.ahrs_data, ahrs.is_ready(), app_mode.get_mode())) {
      app_mode.set_mode(MODE_FREE);

      if (failsafe.get_error_state() == FS_ERROR_LOOP_SLOW) {
        ahrs.reset_att();
      }
    } 

    // 2. Push fresh data to the telemetry queues
    system_battery.update(dt);
    current_sys_state.battery_v = system_battery.get_voltage();
    current_sys_state.delay_count = failsafe.get_delay_count();
    current_sys_state.cmd = app_mode.get_user_command();
    current_sys_state.abc_state.velocity = app_mode.get_velocity();
    current_sys_state.pid_target = app_mode.get_pid_target();
    
    // Copy cached sensor data into current telemetry packet
    i2c_sensor.get_sensor_data(&current_sys_state.mag_data, &current_sys_state.baro_data);

    if (Telemetry::getInstance().is_transmitting()) {
      Telemetry::getInstance().push_data(current_sys_state);
    }
    uros_telemetry.push_data(current_sys_state);
  }
}

// 2. 通訊與管理任務 (Core 0)
void UART1_Task(void *pvParameters) {
  // Wait for board startup to settle
  vTaskDelay(pdMS_TO_TICKS(2000));

  uros_telemetry.init();

  for (;;) {
    uros_telemetry.update();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}


// 3. 通訊與管理任務 (Core 0)
void Comm_Task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_COMM);
  uint8_t display_counter = 0;

  for (;;) {
    if (Telemetry::getInstance().get_port_id() == PORT_USB) {
      Telemetry::getInstance().process_serial_outgoing();
    }
    app_script.check_serial(Serial, &Telemetry::getInstance());

    uint32_t now = millis();
    if (display_counter++ % 32) { // 200/32=6.25 Hz update rate
      // system display: Monitor
      HAL_Display::WiFiStatus wifi_s;
#if HAS_WIFI_SERIAL
      wifi_s.is_ap = (WiFi.getMode() & WIFI_AP);
      wifi_s.connected = (WiFi.status() == WL_CONNECTED) || wifi_s.is_ap;
      if (wifi_s.is_ap) {
          strncpy(wifi_s.ip, WiFi.softAPIP().toString().c_str(), 16);
      } else {
          strncpy(wifi_s.ip, WiFi.localIP().toString().c_str(), 16);
      }
      wifi_s.data_active = Telemetry::getInstance().is_transmitting() && (Telemetry::getInstance().get_port_id() == PORT_WIFI);
#else
      wifi_s.connected = false;
      wifi_s.data_active = false;
      strcpy(wifi_s.ip, "OFF");
#endif

      bool joy_connected = false;
#if HAS_BLUEPAD32
      joy_connected = joystick.is_connected();
#endif

      system_display.update(app_mode.get_mode(), app_mode.get_pending_mode(), system_battery.get_voltage(), failsafe.get_error_state(), &wifi_s, joy_connected);
    }

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
      if (Telemetry::getInstance().get_port_id() == PORT_WIFI) {
        Telemetry::getInstance().process_serial_outgoing();
      }
      if (len > 0) {
        app_script.check_serial(udp_stream, &Telemetry::getInstance());
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
#endif

#ifndef UNIT_TEST
void setup() {
  Serial.setTxBufferSize(8192);
  Serial.begin(921600);
  // Serial1 (Telemetry/Script)
  Serial1.setTxBufferSize(8192);
  Serial1.begin(2000000, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);

  config_store.begin();
  if (!ahrs.init()) {
    Serial.println("WARNING: AHRS initialization failed! System will run in degraded mode (no attitude control).");
  }
  i2c_sensor.init();
  motor.init();
  Telemetry::getInstance().set_port(Serial, PORT_USB);
  Telemetry::getInstance().init(MAIN_LOOP_RATE_HZ);
#if HAS_WIFI_SERIAL
  // WiFi Initialization
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setTxPower(WIFI_POWER_15dBm);
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
    WiFi.softAP(AP_SSID, AP_PASS);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }
  udp_stream.begin();
#endif


  app_mode.init();
  system_display.init();
  system_battery.init();
  app_mode.set_mode(MODE_FREE);

  control_timer_sem = xSemaphoreCreateCounting(10, 0);
  const esp_timer_create_args_t timer_args = {
      .callback = &control_timer_callback,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "control_timer"
  };
  esp_timer_handle_t timer;
  esp_timer_create(&timer_args, &timer);
  esp_timer_start_periodic(timer, (uint64_t)(1000000.0f / MAIN_LOOP_RATE_HZ));

  xTaskCreatePinnedToCore(Control_Task, "ControlTask", 12288, NULL,
                          PRIORITY_CONTROL, &ControlTaskHandle, 1);

  xTaskCreatePinnedToCore(UART1_Task, "UART1Task", 8192, NULL, PRIORITY_UART1,
                          &UART1TaskHandle, 0);

  xTaskCreatePinnedToCore(Comm_Task, "CommTask", 8192, NULL, PRIORITY_COMM,
                          &CommTaskHandle, 0);

#if HAS_BLUEPAD32
  xTaskCreatePinnedToCore(HAL_Joystick::task_entry, "Gamepad_Task", 4096,
                          &joystick,
                          PRIORITY_GAMEPAD, &GamepadTaskHandle, 0);
#endif

#if HAS_WIFI_SERIAL
  xTaskCreatePinnedToCore(WiFi_Task, "WiFi_Task", 8192, NULL, PRIORITY_WIFI,
                          &WiFiTaskHandle, 0);
#endif
}

void loop() { vTaskDelete(NULL); }
#endif
