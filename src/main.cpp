#include <Arduino.h>
#include "processing/processing_ahrs.h"
#include "processing/processing_motor.h"


#define PRIORITY_SAFETY    25
#define PRIORITY_CONTROL   24
#define PRIORITY_ENCODER   20
#define PRIORITY_COMM      10


#define PERIOD_CONTROLL       5    // 200Hz
#define PERIOD_COMM           20   // 50Hz


TaskHandle_t ControlTaskHandle;
TaskHandle_t CommTaskHandle;

Processing_Motor motor_controller;


void Control_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_CONTROLL);

    // [初始化]：在此初始化 IMU (MPU6050) 與 PID 參數
    processing_ahrs_init();
    motor_controller.set_target_rpms(0, 0);

    for (;;) {
        processing_ahrs_update();
        motor_controller.update_rpms();


        float euler[3];
        imu_data_t imu;
        processing_ahrs_get_euler(euler);
        processing_ahrs_get_imu(&imu);

        // Print out the values for verification
        int left_rpm = motor_controller.get_left_rpm();
        int right_rpm = motor_controller.get_right_rpm();

        // Print out the values for VOFA+
        Serial.printf("%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                      euler[0], euler[1], euler[2],
                      left_rpm, right_rpm,
                      imu.gyro[0], imu.gyro[1], imu.gyro[2],
                      imu.accl[0], imu.accl[1], imu.accl[2]);

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
        // 1. 處理遙控器 (Xbox/Gamepad) 封包
        // 2. 將數據打包成 Binary 傳送至 VOFA+
        // 3. 讀取與 RPi 通訊的 UART Buffer
        
        vTaskDelay(pdMS_TO_TICKS(PERIOD_COMM));
    }
}

void setup() {
    Serial.begin(115200);

    // 硬體初始化 (HAL 層)
    motor_controller.init();

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