#include <Arduino.h>
#include <BluetoothSerial.h>
#include "processing/prs_ahrs.h"
#include "processing/prs_motor.h"
#include "hal/hal_telemetry.h"
#include "hal/hal_led.h"
#include "hal/hal_storage.h"
#include "app/app_mode.h"
#include "app/app_script.h"
#include "app/app_fail_safe.h"


#define PRIORITY_CONTROL   24
#define PRIORITY_AHRS      15
#define PRIORITY_COMM      10
#define PRIORITY_BT        5

#define PERIOD_CONTROLL       10    // 100Hz
#define PERIOD_AHRS           10    // 100Hz
#define PERIOD_COMM           20   // 50Hz
#define PERIOD_BT             20   // 50Hz

TaskHandle_t ControlTaskHandle;
TaskHandle_t AHRSTaskHandle;
TaskHandle_t CommTaskHandle;
TaskHandle_t BTTaskHandle;

BluetoothSerial SerialBT;
Processing_Motor motor(PERIOD_CONTROLL);
Processing_AHRS ahrs(PERIOD_AHRS);
Telemetry uart_telemetry(Serial);
Telemetry bt_telemetry(SerialBT);
HAL_LED system_led(LED_BUILTIN);
ConfigStore config_store;

AppMode app_mode(&motor, &config_store);
AppScript app_script(&app_mode);
Failsafe failsafe(&motor, &system_led);

QueueHandle_t ahrs_queue;

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
        static ahrs_data_t last_ahrs_data = {0};

        // 簡單的濾波/保護，避免 dt 異常 (例如第一次執行或溢位)
        if (dt <= 0.0f || dt > 0.1f) dt = PERIOD_CONTROLL * 0.001f;

        motor.update_rpms(dt);
        // 從 AHRS 任務的 Queue 接收 AHRS 資料
        if (xQueueReceive(ahrs_queue, &current_sys_state.abc_state.ahrs_data, 0) != pdTRUE) {
            current_sys_state.abc_state.ahrs_data = last_ahrs_data;
            current_sys_state.abc_state.ahrs_data.imu_data.timestamp = current_micros;
        } else {
            last_ahrs_data = current_sys_state.abc_state.ahrs_data;
        }

        // 1. Run Control Loop
        app_mode.update(dt, current_sys_state.abc_state.ahrs_data);

        failsafe.check(current_sys_state.abc_state.ahrs_data);

        // 2. Push fresh data to the telemetry queues (non-blocking)
        motor.get_motor_state(&current_sys_state.abc_state.motor_state);
        current_sys_state.loop_time_ms = failsafe.get_delay_count();
        current_sys_state.target_val = app_mode.get_current_target_val();
        current_sys_state.mode = app_mode.get_current_mode();
        current_sys_state.abc_state.velocity = app_mode.get_current_velocity();
        
        uart_telemetry.push_data(current_sys_state);
        bt_telemetry.push_data(current_sys_state);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // 確保精確的執行頻率
    }
}

// 4. AHRS 任務 (Core 1)
void AHRS_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_AHRS);
    bool ahrs_ready_notified = false;

    for (;;) {
        ahrs.update();

        if (!ahrs_ready_notified && ahrs.is_ready()) {
            failsafe.set_ahrs_ready(true);
            ahrs_ready_notified = true;
        }

        ahrs_data_t ahrs_data;
        ahrs.get_ahrs_data(&ahrs_data);

        // 將 AHRS 資料推送到 Queue
        // xQueueSend(ahrs_queue, &ahrs_data, 0);
        xQueueOverwrite(ahrs_queue, &ahrs_data); // 使用 Overwrite 以確保最新資料可用
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // 確保精確的執行頻率
    }
}

// 2. 通訊與管理任務 (Core 0)
void Comm_Task(void *pvParameters) {
    for (;;) {
        uart_telemetry.process_serial_outgoing(); 
        app_script.check_serial(Serial, &uart_telemetry);
        system_led.update();
        
        vTaskDelay(pdMS_TO_TICKS(PERIOD_COMM));
    }
}

// 3. 藍牙遙測發送任務 (Core 0)
void BT_Task(void *pvParameters) {
    for (;;) {
        bt_telemetry.process_bt_outgoing();
        app_script.check_serial(SerialBT, &bt_telemetry);

        vTaskDelay(pdMS_TO_TICKS(PERIOD_COMM));
    }
}


void setup() {
    system_led.set_state(INITIALIZING);

    Serial.begin(115200);
    SerialBT.begin("ABC_Controller");
    config_store.begin();
    ahrs.init();
    motor.init();
    uart_telemetry.init();
    bt_telemetry.init();

    app_mode.init();
    app_mode.set_mode(MODE_FREE);

    ahrs_queue = xQueueCreate(1, sizeof(ahrs_data_t));

    // 建立任務 參數：函數名, 名稱, 堆棧, 參數, 優先級, Handle, 核心ID
    // Core 1   
    xTaskCreatePinnedToCore(Control_Task, "ControlTask", 8192, NULL, PRIORITY_CONTROL, &ControlTaskHandle, 1);
    xTaskCreatePinnedToCore(AHRS_Task, "AHRSTask", 4096, NULL, PRIORITY_AHRS, &AHRSTaskHandle, 1);
    
    // Core 0
    xTaskCreatePinnedToCore(Comm_Task, "CommTask", 4096, NULL, PRIORITY_COMM, &CommTaskHandle, 0);
    // xTaskCreatePinnedToCore(BT_Task,   "BT_Task" , 4096, NULL, PRIORITY_BT, &BTTaskHandle, 0);
}

void loop() {
    vTaskDelete(NULL); 
}