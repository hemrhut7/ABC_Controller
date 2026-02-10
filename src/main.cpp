#include <Arduino.h>
#include "processing/prs_ahrs.h"
#include "processing/prs_motor.h"
#include "hal/hal_telemetry.h"
#include "hal/hal_led.h"
#include "hal/hal_storage.h"
#include "app/app_mode.h"
#include "app/app_script.h"


#define PRIORITY_SAFETY    25
#define PRIORITY_CONTROL   24
#define PRIORITY_ENCODER   20
#define PRIORITY_COMM      10


#define PERIOD_CONTROLL       5    // 200Hz
#define PERIOD_COMM           20   // 50Hz

TaskHandle_t ControlTaskHandle;
TaskHandle_t CommTaskHandle;

Processing_Motor motor(PERIOD_CONTROLL);
Processing_AHRS ahrs(PERIOD_CONTROLL);
Telemetry telemetry(Serial);
HAL_LED system_led(LED_BUILTIN);
ConfigStore config_store;

AppMode app_mode(&ahrs, &motor, &config_store);
AppScript app_script(&app_mode);

void Control_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_CONTROLL);

    system_led.set_state(INITIALIZING);
    motor.set_target_rpms(0, 0);
    app_mode.init();
    
    // Default to Pitch Mode (Balancing at 0 degrees)
    app_mode.set_mode(MODE_ANGLE);
    app_mode.set_target_val(0.0f);

    system_led.set_state(WORKING);

    for (;;) {
        ahrs.update();
        motor.update_rpms();

        // 1. Run Control Loop
        // app_mode 內部會自動獲取 ahrs 和 motor 的數據並計算
        // 計算結果會直接寫入 motor 物件的 target_rpm
        app_mode.update(PERIOD_CONTROLL * 0.001f);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // 確保精確的執行頻率
    }
}

// 2. 通訊與管理任務 (Core 0)
void Comm_Task(void *pvParameters) {
    // [初始化]：在此啟動 Wi-Fi/Bluetooth 或 UART2 (RPi)
    // WiFi_Init();
    // UART2_Init();

    for (;;) {
        ABC_state_t abc_state;

        ahrs.get_ahrs_data(&abc_state.ahrs_data);
        motor.get_motor_state(&abc_state.motor_state);

        // 將數據打包成 Binary 傳送至 VOFA+
        telemetry.queue_vofa_data(abc_state);
        telemetry.send_data();

        system_led.update();
        config_store.update();
                
        // 讀取與 RPi 通訊的 UART Buffer
        // 支援 Serial, 未來可輕易擴充 BluetoothSerial 等
        app_script.check_serial(Serial);

        vTaskDelay(pdMS_TO_TICKS(PERIOD_COMM));
    }
}

void setup() {
    Serial.begin(115200);

    // 硬體初始化 (HAL 層)
    motor.init();
    telemetry.init();

    // 建立任務 參數：函數名, 名稱, 堆棧, 參數, 優先級, Handle, 核心ID
    // Core 1   
    xTaskCreatePinnedToCore(Control_Task, "ControlTask", 8192, NULL, PRIORITY_CONTROL, &ControlTaskHandle, 1);
    
    // Core 0
    xTaskCreatePinnedToCore(Comm_Task, "CommTask", 4096, NULL, PRIORITY_COMM, &CommTaskHandle, 0);
}

void loop() {
    vTaskDelete(NULL); 
}