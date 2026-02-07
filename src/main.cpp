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

Processing_Motor motor_controller;
Processing_AHRS ahrs;
Telemetry telemetry(Serial);


void Control_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_CONTROLL);

    // [初始化]：在此初始化 IMU (MPU6050) 與 PID 參數
    ahrs.init();
    motor_controller.set_target_rpms(0, 0);

    for (;;) {
        ahrs.update();
        motor_controller.update_rpms();

        // 3. Cascaded PID Calculation
        // 4. Output to LEDC (PWM)
        
        
        
        // --- 核心邏輯結束 ---

        // 確保精確的執行頻率
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 2. 通訊與管理任務 (Core 0)
void Comm_Task(void *pvParameters) {
    // [初始化]：在此啟動 Wi-Fi/Bluetooth 或 UART2 (RPi)
    // WiFi_Init();
    // UART2_Init();

    for (;;) {
        ahrs_data_t ahrs_data;
        motor_state_t motor_state;

        ahrs.get_euler(ahrs_data.euler);
        ahrs.get_imu(&ahrs_data.imu_data);
        motor_controller.get_motor_state(&motor_state);

        

        // 將數據打包成 Binary 傳送至 VOFA+
        telemetry.queue_vofa_data(ahrs_data.euler, motor_state.rpm_L, motor_state.target_rpm_L, motor_state.rpm_R, motor_state.target_rpm_R,
                                  ahrs_data.imu_data.gyro, ahrs_data.imu_data.accl);
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
    motor_controller.init();
    telemetry.init();

    // 建立任務
    // 參數：函數名, 名稱, 堆棧, 參數, 優先級, Handle, 核心ID
    xTaskCreatePinnedToCore(
        Control_Task,   "ControlTask",  8192,  NULL, 
        PRIORITY_CONTROL, &ControlTaskHandle, 1         // 固定在 Core 1
    );

    

    xTaskCreatePinnedToCore(
        Comm_Task,      "CommTask",     4096,     NULL, 
        PRIORITY_COMM,    &CommTaskHandle,    0         // 固定在 Core 0 (與 Wi-Fi 共用)
    );
}

void loop() {
    vTaskDelete(NULL); 
}