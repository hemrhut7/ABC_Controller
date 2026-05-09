#include "hal_display.h"

HAL_Display::HAL_Display(uint16_t w, uint16_t h) 
    : _w(w), _h(h) {
}

void HAL_Display::init() {
    // Arduino_ESP32SPI(int8_t dc, int8_t cs, int8_t sclk, int8_t mosi, int8_t miso)
    Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC_PIN, LCD_CS_PIN, LCD_SCLK_PIN, LCD_MOSI_PIN, LCD_MISO_PIN);
    
    // Arduino_ST7789(Arduino_DataBus *bus, int8_t rst, uint8_t rotation, bool ips, int16_t w, int16_t h)
    // Rotation = 1 (Landscape, 320x240)
    _gfx = new Arduino_ST7789(bus, -1 /* RST */, 1 /* rotation */, true /* IPS */, _w, _h);

    if (!_gfx->begin()) {
        Serial.println(F("[DISPLAY] ST7789 GFX failed"));
        _connected = false;
        return;
    }

    _connected = true;
    Serial.println(F("[DISPLAY] ST7789 GFX OK"));

    // Backlight on
    pinMode(LCD_BL_PIN, OUTPUT);
    digitalWrite(LCD_BL_PIN, HIGH);

    _gfx->fillScreen(BLACK);
    _gfx->setTextColor(WHITE);
    _gfx->setTextSize(3);
    _gfx->setCursor(20, 40);
    _gfx->println(F("ABC Controller"));
    _gfx->setTextSize(2);
    _gfx->setCursor(20, 80);
    _gfx->println(F("S3-LCD-2 Migration..."));
}

void HAL_Display::update(Mode_t current_mode, Mode_t pending_mode, float battery_v, int delay_count) {
    if (!_connected) return;

    // Dirty check
    bool force_redraw = !_layout_drawn;
    if (!force_redraw
     && current_mode == _last_current_mode 
     && pending_mode == _last_pending_mode
     && abs(battery_v - _last_battery_v) < 0.05f
     && delay_count == _last_delay_count) {
        return;
    }

    if (force_redraw) {
        _gfx->fillScreen(BLACK);
        
        // Static Header
        _gfx->setTextSize(3);
        _gfx->setCursor(10, 10);
        _gfx->setTextColor(YELLOW);
        _gfx->print(F("MODE: "));

        // Static Pending label
        _gfx->setTextSize(2);
        _gfx->setCursor(10, 50);
        _gfx->setTextColor(DARKGREY);
        _gfx->print(F("Pending: "));

        // Static Delay label
        _gfx->setTextSize(2);
        _gfx->setCursor(10, 160);
        _gfx->setTextColor(WHITE);
        _gfx->print(F("Loop Delay: "));

        _layout_drawn = true;
    }

    // Dynamic Current Mode
    if (force_redraw || current_mode != _last_current_mode) {
        _gfx->fillRect(120, 10, 190, 30, BLACK); // Clear mode area
        _gfx->setTextSize(3);
        _gfx->setCursor(120, 10);
        _gfx->setTextColor(CYAN);
        _gfx->println(mode_to_str(current_mode));
    }

    // Dynamic Pending Mode info
    if (force_redraw || pending_mode != _last_pending_mode || current_mode != _last_current_mode) {
        _gfx->fillRect(120, 50, 190, 20, BLACK); // Clear pending area
        if (current_mode == MODE_FREE || current_mode == MODE_STOP) {
            _gfx->setTextSize(2);
            _gfx->setCursor(120, 50);
            _gfx->setTextColor(DARKGREY);
            _gfx->print(mode_to_str(pending_mode));
        }
    }

    // Dynamic Battery Info
    if (force_redraw || abs(battery_v - _last_battery_v) >= 0.05f) {
        _gfx->fillRect(10, 100, 300, 40, BLACK); // Clear battery area
        _gfx->setTextSize(4);
        _gfx->setCursor(10, 100);
        if (battery_v < 11.0f) _gfx->setTextColor(RED);
        else if (battery_v < 11.5f) _gfx->setTextColor(ORANGE);
        else _gfx->setTextColor(GREEN);
        _gfx->print(battery_v, 2);
        _gfx->print(F(" V"));
    }

    // Dynamic Delay Info
    if (force_redraw || delay_count != _last_delay_count) {
        _gfx->fillRect(160, 160, 100, 20, BLACK); // Clear delay value area
        _gfx->setTextSize(2);
        _gfx->setCursor(160, 160);
        _gfx->setTextColor(WHITE);
        _gfx->print(delay_count);
        _gfx->print(F(" ms"));
    }

    // Status Indicator (Virtual LED)
    if (force_redraw || current_mode != _last_current_mode) {
        uint16_t status_color = RED;
        const char* status_text = "DISARMED";
        if (current_mode != MODE_STOP && current_mode != MODE_FREE) {
            status_color = GREEN;
            status_text = "ARMED";
        }
        
        _gfx->fillRect(0, 200, 320, 40, status_color);
        _gfx->setTextColor(WHITE);
        _gfx->setTextSize(3);
        _gfx->setCursor(80, 210);
        _gfx->print(status_text);
    }

    _last_current_mode = current_mode;
    _last_pending_mode = pending_mode;
    _last_battery_v = battery_v;
    _last_delay_count = delay_count;
}

void HAL_Display::show_message(const char* line1, const char* line2) {
    if (!_connected) return;

    _layout_drawn = false; // Layout needs to be redrawn after message
    _gfx->fillScreen(BLACK);
    _gfx->setTextColor(WHITE);
    _gfx->setTextSize(3);
    _gfx->setCursor(10, 40);
    _gfx->println(line1);
    if (line2) {
        _gfx->setTextSize(2);
        _gfx->setCursor(10, 100);
        _gfx->println(line2);
    }
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
