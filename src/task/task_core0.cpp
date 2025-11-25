#include "task/task_core0.h"
#include "hal/hal_led.h"

static QueueHandle_t AHRS_data_queue;

void task_read_AHRS(void * pvParameters) {
    static uint32_t last_time = millis();
    for (;;) {
        AHRS_Data data;
        if (hal_msg_getAHRS_Data(data)) {
            BaseType_t xStatus = xQueueOverwrite(AHRS_data_queue, &data);
            





            uint32_t now = millis();
            if (now - last_time > 3000) {
                set_led_state(CONFIGURING);
            } else if (now == last_time) {
                set_led_state(IMU_MEASURING);
            }
        }

        //test for timer comparison
        uint32_t t1 = millis();
        uint32_t t2 = xTaskGetTickCount() * portTICK_PERIOD_MS;
        debug_print(UART_USB, "Core0 Time: millis=%lu, Tick=%lu\n", t1, t2);
        vTaskDelay(100 / portTICK_PERIOD_MS); // 等待秒  ------------待測試
    }
}

void task_get_AHRS(AHRS_Data *data){
    // The second argument of xQueueReceive is a pointer to the buffer that will receive the data.
    // 'data' is already a pointer to AHRS_Data, so we pass it directly.
    xQueueReceive(AHRS_data_queue, data, portMAX_DELAY); // This will block until data is available.
}