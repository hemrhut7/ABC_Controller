#include <Arduino.h>
#include "processing/prs_ahrs.h"
#include "processing/prs_motor.h"
#include "processing/prs_controll.h"
#include "hal/hal_telemetry.h"


#define PRIORITY_SAFETY    25
#define PRIORITY_CONTROL   24
#define PRIORITY_ENCODER   20
#define PRIORITY_COMM      10


#define PERIOD_CONTROLL       5    // 200Hz
#define PERIOD_COMM           20   // 50Hz

// PID Parameters (需根據實際機體調整)
#define PITCH_KP 12.0f
#define PITCH_KI 0.0f
#define PITCH_KD 0.5f

#define RATE_KP 1.5f
#define RATE_KI 8.0f
#define RATE_KD 0.04f

#define YAW_KP 2.0f
#define YAW_KI 0.0f
#define YAW_KD 0.0f


TaskHandle_t ControlTaskHandle;
TaskHandle_t CommTaskHandle;

Processing_Motor motor(PERIOD_CONTROLL);
Processing_AHRS ahrs(PERIOD_CONTROLL);
Telemetry telemetry(Serial);

Pitch_Controller pitch_ctrl(PITCH_KP, PITCH_KI, PITCH_KD);
Rate_Controller rate_ctrl(RATE_KP, RATE_KI, RATE_KD);
Yaw_Controller yaw_ctrl(YAW_KP, YAW_KI, YAW_KD);

void Control_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_CONTROLL);

    // [初始化]：在此初始化 IMU (MPU6050) 與 PID 參數
    ahrs.init();
    motor.set_target_rpms(0, 0);
    
    // 設定 PID 輸出限制
    pitch_ctrl.setOutputLimits(-300 * DEG_TO_RAD, 300 * DEG_TO_RAD); // 外環輸出為目標角速度 (deg/s)
    rate_ctrl.setOutputLimits(-300 * DEG_TO_RAD, 300 * DEG_TO_RAD);  // 內環輸出為馬達轉速 (RPM)
    yaw_ctrl.setOutputLimits(-100 * DEG_TO_RAD, 100 * DEG_TO_RAD);   // 轉向差速限制

    for (;;) {
        ahrs.update();
        motor.update_rpms();
        
        // 1. 獲取姿態數據
        ahrs_data_t data;
        ahrs.get_ahrs_data(&data);

        // 不進行單位轉換
        float pitch_deg = data.euler[0];
        float pitch_rate_deg = data.imu_data.gyro[0]; // 假設 X 軸為 Pitch 軸
        float yaw_rate_deg = data.imu_data.gyro[2];

        // 2. 串級 PID 計算
        // 外環：角度控制 (目標角度 0 度) -> 輸出目標角速度
        float target_pitch_rate = pitch_ctrl.set_target(0.0f, pitch_deg);

        // 內環：角速度控制 -> 輸出平衡控制量 (RPM)
        float balance_output = rate_ctrl.set_target(target_pitch_rate, pitch_rate_deg);

        // 轉向控制：角速度控制 (目標轉向速度 0) -> 輸出轉向差速
        float yaw_output = yaw_ctrl.set_target(0.0f, yaw_rate_deg);

        // 3. 混合輸出至馬達
        int target_L = (int)(balance_output + yaw_output);
        int target_R = (int)(balance_output - yaw_output);

        motor.set_target_rpms(target_L, target_R);

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