#pragma once

#include <Arduino.h>
#include "app_mode.h"
#include "hal/hal_telemetry.h"

class AppScript {
public:
    AppScript(AppMode* app_mode);
    
    // 通用解析函數，可供 Serial, Bluetooth, WiFi 等不同來源調用
    // 傳入一行完整的指令字串 (例如 "CMD 3 0.0 0.0")
    void parse_packet(const String& packet, Telemetry *telemetry);

    // 針對 Stream (如 Serial, BluetoothSerial) 的輔助函數
    void check_serial(Stream& stream, Telemetry *telemetry);

private:
    AppMode* _app_mode;
};
