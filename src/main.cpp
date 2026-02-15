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


#define PRIORITY_SAFETY    25
#define PRIORITY_CONTROL   24
#define PRIORITY_ENCODER   20
#define PRIORITY_COMM      10
#define PRIORITY_BT        5


#define PERIOD_CONTROLL       5    // 200Hz
#define PERIOD_COMM           20   // 50Hz
#define PERIOD_BT             20   // 50Hz

TaskHandle_t ControlTaskHandle;
TaskHandle_t CommTaskHandle;
TaskHandle_t BTTaskHandle;

BluetoothSerial SerialBT;
Processing_Motor motor(PERIOD_CONTROLL);
Processing_AHRS ahrs(PERIOD_CONTROLL);
Telemetry uart_telemetry(Serial);
Telemetry bt_telemetry(SerialBT);
HAL_LED system_led(LED_BUILTIN);
ConfigStore config_store;

AppMode app_mode(&ahrs, &motor, &config_store);
AppScript app_script(&app_mode);
Failsafe failsafe(&ahrs, &motor, &system_led);


// 1. 控制任務 (Core 1)

void Control_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_CONTROLL);

    for (;;) {
        ahrs.update();
        motor.update_rpms();

        // 1. Run Control Loop
        app_mode.update(PERIOD_CONTROLL * 0.001f);

        failsafe.check();

        // 2. Push fresh data to the telemetry queues (non-blocking)
        system_state_t current_sys_state;
        ahrs.get_ahrs_data(&current_sys_state.abc_state.ahrs_data);
        motor.get_motor_state(&current_sys_state.abc_state.motor_state);
        current_sys_state.loop_time_ms = failsafe.get_loop_time_ms();
        current_sys_state.target_val = app_mode.get_current_target_val();
        current_sys_state.mode = app_mode.get_current_mode();
        current_sys_state.abc_state.velocity = app_mode.get_current_velocity();
        
        uart_telemetry.push_data(current_sys_state);
        bt_telemetry.push_data(current_sys_state);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // 確保精確的執行頻率
    }
}

// 2. 通訊與管理任務 (Core 0)
void Comm_Task(void *pvParameters) {
    for (;;) {
        uart_telemetry.process_serial_outgoing(); 
        app_script.check_serial(Serial, &uart_telemetry);

        system_led.update();
        config_store.update();
        
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
    app_mode.set_target_val(0.0f);

    // 建立任務 參數：函數名, 名稱, 堆棧, 參數, 優先級, Handle, 核心ID
    // Core 1   
    xTaskCreatePinnedToCore(Control_Task, "ControlTask", 8192, NULL, PRIORITY_CONTROL, &ControlTaskHandle, 1);
    
    // Core 0
    xTaskCreatePinnedToCore(Comm_Task, "CommTask", 4096, NULL, PRIORITY_COMM, &CommTaskHandle, 0);
    xTaskCreatePinnedToCore(BT_Task, "BT_Task", 4096, NULL, PRIORITY_BT, &BTTaskHandle, 0);
}

void loop() {
    vTaskDelete(NULL); 
}