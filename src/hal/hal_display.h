#pragma once

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "hal/hal_type_define.h"

// OLED SPI Pins (Software SPI)
#define OLED_MOSI_PIN   13
#define OLED_SCLK_PIN   14
#define OLED_DC_PIN     27
#define OLED_RST_PIN    12
#define OLED_CS_PIN     -1 // Not used or grounded

class HAL_Display {
public:
    HAL_Display(uint8_t w = 128, uint8_t h = 32);
    void init();
    bool is_connected() const { return _connected; }

    // 主要更新介面：傳入當前模式與待選模式
    void update(Mode_t current_mode, Mode_t pending_mode, float battery_v = 0.0f);
    void show_message(const char* line1, const char* line2 = nullptr);

private:
    static const char* mode_to_str(Mode_t mode);

    Adafruit_SSD1306 _display;
    uint8_t _w, _h;
    bool _connected = false;

    // Dirty flag: 只在狀態改變時刷新
    Mode_t _last_current_mode = MODE_STOP;
    Mode_t _last_pending_mode = MODE_STOP;
    float  _last_battery_v = -1.0f;
};
