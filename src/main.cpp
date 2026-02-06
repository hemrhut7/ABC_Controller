#include <Arduino.h>
#include "hal/hal_imu.h"


#define PRIORITY_SAFETY    25
#define PRIORITY_CONTROL   24
#define PRIORITY_ENCODER   20
#define PRIORITY_COMM      10


#define PERIOD_CONTROLL       5    // 200Hz
#define PERIOD_COMM           20   // 50Hz


TaskHandle_t ControlTaskHandle;


TaskHandle_t CommTaskHandle;

void Control_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_CONTROLL);

    // [初始化]：在此初始化 IMU (MPU6050) 與 PID 參數
    hal_imu_init();
    // PID_Init();

    for (;;) {
        // --- 核心邏輯開始 ---
        // 1. Read IMU
        imu_data_t imu_data;
        hal_imu_read(&imu_data);

        // Print out the values for verification
        char print_buffer[256];
        snprintf(print_buffer, sizeof(print_buffer),
                 "Accel X: %.2f, Y: %.2f, Z: %.2f m/s^2\tGyro X: %.2f, Y: %.2f, Z: %.2f rad/s",
                 imu_data.accl[0], imu_data.accl[1], imu_data.accl[2],
                 imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);
        Serial.println(print_buffer);

        // 2. Complementary Filter (Eigen Based)
        // 3. Cascaded PID Calculation
        // 4. Output to LEDC (PWM)
        
        // 模擬運算：Serial.println 在此處為罪行，正式開發請移除
        
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
    // Motor_PWM_Init(); // LEDC 初始化
    // Encoder_Init();   // PCNT 初始化

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