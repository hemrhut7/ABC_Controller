#include <Arduino.h>
#include "type_defs.h"
#include "hal/hal_led.h"
#include "hal/hal_motor.h"
#include "task/task_core0.h"
#include "task/task_script.h"

#if ENABLE_BT
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
#endif

void setup() {
  init_led();
  led_blink_1sec_block();
  UART_USB.begin(115200);
  UART_CTL.begin(230400);
  
  // Create second core task
  // AHRS_data_queue = xQueueCreate(5, sizeof(AHRS_Data));

  xTaskCreatePinnedToCore(
      task_read_AHRS,   // 任務函數名稱
      "Core0_Task",// 任務名稱
      10000,       // 堆棧大小 (10KB)
      NULL,        // 傳遞參數 (無)
      1,           // 優先級 (低於預設的 20)
      NULL,        // 任務句柄 (不需要)
      0            // **核心 ID 0**
  );

  // Initialize PCNT
  led_blink_1sec_block();
  setupPCNT();
  UART_USB.println("Initialize Encoder PCNT...");

  led_blink_1sec_block();
  UART_USB.println("Initialize Motor Driver...");

  set_led_state(IMU_MEASURING);
  UART_USB.println("System initialized.");
}

void loop() {

#if DEBUG_PRINT
  AHRS_Data data;
  task_get_AHRS(&data);
  debug_print(UART_USB, "t:%.2f/t%.1f/t%.1f/t%.1f\n", data.time_sec, data.euler_deg[0], data.euler_deg[1], data.euler_deg[2]);
#endif

  handle_serial_commands(UART_USB);
#if ENABLE_BT
  handle_serial_commands(SerialBT);
#endif

}



