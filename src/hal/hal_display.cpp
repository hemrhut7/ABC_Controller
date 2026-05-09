#include "hal_display.h"

HAL_Display::HAL_Display(uint8_t w, uint8_t h) 
    : _display(w, h, OLED_MOSI_PIN, OLED_SCLK_PIN, OLED_DC_PIN, OLED_RST_PIN, OLED_CS_PIN), 
      _w(w), _h(h) {
}

void HAL_Display::init() {
    // 使用 Software SPI，不依賴全局 Wire 總線
    // 因為是 SPI，通常無法像 I2C 那樣簡單偵測，所以假定連線成功
    if (!_display.begin(SSD1306_SWITCHCAPVCC)) { 
        Serial.println(F("[DISPLAY] SSD1306 SPI failed"));
        _connected = false;
        return;
    }
    _connected = true;
    Serial.println(F("[DISPLAY] SSD1306 SPI OK"));

    _display.clearDisplay();
    _display.setTextColor(SSD1306_WHITE);
    _display.setCursor(0, 0);
    _display.setTextSize(2);
    _display.println(F("ABC Ctrl"));
    _display.setTextSize(1);
    _display.println(F("SPI Mode Init..."));
    _display.display();
}

void HAL_Display::update(Mode_t current_mode, Mode_t pending_mode, float battery_v, int delay_count) {
    if (!_connected) return;

    // Dirty check: 只在狀態改變時重繪
    if (current_mode == _last_current_mode 
     && pending_mode == _last_pending_mode
     && abs(battery_v - _last_battery_v) < 0.05f
     && delay_count == _last_delay_count) {
        return;
    }
    _last_current_mode = current_mode;
    _last_pending_mode = pending_mode;
    _last_battery_v = battery_v;
    _last_delay_count = delay_count;

    _display.clearDisplay();

    // 第一行：當前模式 (大字)
    _display.setCursor(0, 0);
    _display.setTextSize(2);
    _display.print(mode_to_str(current_mode));

    // 待選模式移到當前模式模式右邊 (小字)
    if (current_mode == MODE_FREE) {
        _display.setTextSize(1);
        _display.setCursor(70, 6); 
        _display.print(F("> "));
        _display.print(mode_to_str(pending_mode));
    }

    // 第二行：電池電壓 (小字，始終顯示)
    _display.setCursor(0, 18);
    _display.setTextSize(1);
    _display.print(F("V:"));
    _display.print(battery_v, 1);
    _display.print(F("V"));

    // 右下角：簡易狀態指示
    _display.setCursor(100, 24);
    _display.print(F("D:"));
    _display.print(delay_count);

    _display.display();
}

void HAL_Display::show_message(const char* line1, const char* line2) {
    if (!_connected) return;

    _display.clearDisplay();
    _display.setCursor(0, 0);
    _display.setTextSize(2);
    _display.println(line1);
    if (line2) {
        _display.setTextSize(1);
        _display.println(line2);
    }
    _display.display();
}

const char* HAL_Display::mode_to_str(Mode_t mode) {
    switch (mode) {
        case MODE_STOP:     return "STOP";
        case MODE_FREE:     return "FREE";
        case MODE_PWM:      return "PWM";
        case MODE_MOTOR:    return "MOTOR";
        case MODE_ANGLE:    return "ANGLE";
        case MODE_VELOCITY: return "VELOCITY";
        case MODE_REMOTE:   return "REMOTE";
        default:            return "???";
    }
}
