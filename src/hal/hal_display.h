#pragma once

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "hal/hal_type_define.h"
#include "config.h"

#include "app/app_fail_safe.h"

#define LCD_WIDTH      240
#define LCD_HEIGHT     320

class HAL_Display {
public:
    HAL_Display(uint16_t w = LCD_WIDTH, uint16_t h = LCD_HEIGHT);
    void init();
    bool is_connected() const { return _connected; }

    // 主要更新介面：傳入當前模式與待選模式
    void update(Mode_t current_mode, Mode_t pending_mode, float battery_v = 0.0f, failsafe_error_t error_state = FS_ERROR_NONE);
    void show_message(const char* line1, const char* line2 = nullptr);

private:
    static const char* mode_to_str(Mode_t mode);
    static const char* error_to_str(failsafe_error_t error);

    Arduino_GFX* _gfx = nullptr;
    uint16_t _w, _h;
    bool _connected = false;

    // Dirty flag: 只在狀態改變時刷新
    bool   _layout_drawn = false;
    Mode_t _last_current_mode = MODE_STOP;
    Mode_t _last_pending_mode = MODE_STOP;
    failsafe_error_t _last_error_state = FS_ERROR_NONE;
    float  _last_battery_v = -1.0f;

};
