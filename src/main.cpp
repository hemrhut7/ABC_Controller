#include <Arduino.h>
#include "processing/processing_ahrs.h"
#include "processing/processing_motor.h"
#include "hal/hal_telemetry.h"


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


void Control_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_CONTROLL);

    // [初始化]：在此初始化 IMU (MPU6050) 與 PID 參數
    ahrs.init();
    motor.set_target_rpms(0, 0);

    for (;;) {
        ahrs.update();
        motor.update_rpms();
        // 3. Cascaded PID Calculation
        // 4. Output to LEDC (PWM)
                
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
                
        // 讀取與 RPi 通訊的 UART Buffer
        // 處理遙控器 (Xbox/Gamepad) 封包
        // 藍芽/WIFI
        
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