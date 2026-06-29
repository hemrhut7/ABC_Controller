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
#include "app/app_lidar.h"

#if HAS_BLUEPAD32
#include "hal/hal_joystick.h"
#endif

#define PRIORITY_CONTROL 24
#define PRIORITY_AHRS 15
#define PRIORITY_UART1 12
#define PRIORITY_COMM 10
#define PRIORITY_GAMEPAD 8
#define PRIORITY_LIDAR 10

#define MAIN_LOOP_RATE_HZ  200.0f
#define PERIOD_CONTROLL    1000.0f/MAIN_LOOP_RATE_HZ
#define PERIOD_COMM        5  // 200Hz
#define PERIOD_GAMEPAD     20 // 50Hz

TaskHandle_t ControlTaskHandle;
TaskHandle_t UART1TaskHandle;
TaskHandle_t CommTaskHandle;
TaskHandle_t GamepadTaskHandle;
#if HAS_WIFI_SERIAL
TaskHandle_t WiFiTaskHandle;
#endif
TaskHandle_t SensorTaskHandle;
extern AppLidar app_lidar;

void Sensor_Task(void *pvParameters) {
  for (;;) {
    // Update Lidar (runs continuously to empty UART buffer)
    app_lidar.update_step();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

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
Processing_Motor motor(PERIOD_CONTROLL);
Processing_AHRS ahrs(PERIOD_CONTROLL);
Processing_I2CSensor i2c_sensor(MAIN_LOOP_RATE_HZ);
AppMode app_mode(&motor, &config_store, PERIOD_CONTROLL);

AppScript app_script(&app_mode);

void set_app_pending_mode(Mode_t mode) {
  app_mode.set_pending_mode(mode);
}

HAL_Display system_display;
HAL_Battery system_battery;
Failsafe failsafe(PERIOD_CONTROLL, &motor, set_app_pending_mode);

Telemetry uart_telemetry(Serial, PORT_USB);
Telemetry uart1_telemetry(Serial1, PORT_UART1);

#if HAS_BLUEPAD32
HAL_Joystick joystick(&app_mode, PERIOD_GAMEPAD);
#endif

#if HAS_WIFI_SERIAL
UDPStream udp_stream(UDP_PORT);
Telemetry udp_telemetry(udp_stream, PORT_WIFI);
#endif

AppLidar app_lidar(Serial2);




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
      dt = PERIOD_CONTROLL * 0.001f;

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
    current_sys_state.cmd.mode = app_mode.get_mode();
    current_sys_state.abc_state.velocity = app_mode.get_velocity();
    current_sys_state.pid_target = app_mode.get_pid_target();
    
    // Copy cached sensor data into current telemetry packet
    i2c_sensor.get_sensor_data(&current_sys_state.mag_data, &current_sys_state.baro_data);

    uart_telemetry.push_data(current_sys_state);
    uart1_telemetry.push_data(current_sys_state); // Push to UART1 too

#if HAS_WIFI_SERIAL
    udp_telemetry.push_data(current_sys_state);
#endif
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
    wifi_s.data_active = udp_telemetry.connected();
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

#ifndef UNIT_TEST
void setup() {
  Serial.begin(230400);
  // Serial1 (Telemetry/Script)
  Serial1.begin(2000000, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
  // Serial2 (Sensor RX only)
  Serial2.setRxBufferSize(1024);
  Serial2.begin(230400, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

  config_store.begin();
  if (!ahrs.init()) {
    Serial.println("WARNING: AHRS initialization failed! System will run in degraded mode (no attitude control).");
  }
  i2c_sensor.init();
  motor.init();
  app_lidar.init();
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
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }
  udp_stream.begin();
#endif
  
  app_lidar.register_telemetry(&uart_telemetry);
  app_lidar.register_telemetry(&uart1_telemetry);
#if HAS_WIFI_SERIAL
  app_lidar.register_telemetry(&udp_telemetry);
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

  xTaskCreatePinnedToCore(UART1_Task, "UART1Task", 4096, NULL, PRIORITY_UART1,
                          &UART1TaskHandle, 0);

  xTaskCreatePinnedToCore(Comm_Task, "CommTask", 8192, NULL, PRIORITY_COMM,
                          &CommTaskHandle, 0);

#if HAS_BLUEPAD32
  xTaskCreatePinnedToCore(HAL_Joystick::task_entry, "Gamepad_Task", 8192,
                          &joystick,
                          PRIORITY_GAMEPAD, &GamepadTaskHandle, 0);
#endif

#if HAS_WIFI_SERIAL
  xTaskCreatePinnedToCore(WiFi_Task, "WiFi_Task", 4096, NULL, PRIORITY_COMM,
                          &WiFiTaskHandle, 0);
#endif

  xTaskCreatePinnedToCore(Sensor_Task, "SensorTask", 16384, NULL, 
                          PRIORITY_LIDAR, &SensorTaskHandle, 0);
}

void loop() { vTaskDelete(NULL); }
#endif
