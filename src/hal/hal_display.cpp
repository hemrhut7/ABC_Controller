#include "hal_display.h"

HAL_Display::HAL_Display(uint16_t w, uint16_t h) 
    : _w(w), _h(h) {
}

void HAL_Display::init() {
    // Arduino_ESP32SPI(int8_t dc, int8_t cs, int8_t sclk, int8_t mosi, int8_t miso)
    Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC_PIN, LCD_CS_PIN, LCD_SCLK_PIN, LCD_MOSI_PIN, LCD_MISO_PIN);

    // Arduino_ST7789(Arduino_DataBus *bus, int8_t rst, uint8_t rotation, bool ips, int16_t w, int16_t h)
    // Rotation = 3 (Landscape, 320x240)
    _gfx = new Arduino_ST7789(bus, -1 /* RST */, 3 /* rotation */, true /* IPS */, _w, _h);

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

void HAL_Display::update(Mode_t current_mode, Mode_t pending_mode, float battery_v, failsafe_error_t error_state, 
                         const WiFiStatus* wifi_status, bool joystick_connected) {
    if (!_connected) return;

    bool wifi_connected = wifi_status ? wifi_status->connected : false;
    bool wifi_data_active = wifi_status ? wifi_status->data_active : false;
    const char* wifi_ip = wifi_status ? wifi_status->ip : "0.0.0.0";

    // Dirty check
    bool force_redraw = !_layout_drawn;
    if (!force_redraw
     && current_mode == _last_current_mode 
     && pending_mode == _last_pending_mode
     && error_state == _last_error_state
     && abs(battery_v - _last_battery_v) < 0.05f
     && wifi_connected == _last_wifi_connected
     && wifi_data_active == _last_wifi_data_active
     && joystick_connected == _last_joystick_connected
     && strcmp(wifi_ip, _last_ip) == 0) {
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

        // Status Area Labels (Optional, but icons/circles are better as requested)
        _gfx->setTextSize(1);
        _gfx->setTextColor(LIGHTGREY);
        _gfx->setCursor(10, 190);
        _gfx->print(F("WIFI:"));
        _gfx->setCursor(200, 190);
        _gfx->print(F("JOY:"));
        _gfx->setCursor(260, 190);
        _gfx->print(F("DATA:"));

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

    // Error Message Area
    if (force_redraw || error_state != _last_error_state) {
        _gfx->fillRect(10, 80, 300, 20, BLACK);
        if (error_state != FS_ERROR_NONE) {
            _gfx->setTextSize(2);
            _gfx->setCursor(10, 80);
            _gfx->setTextColor(RED);
            _gfx->print(F("ERR: "));
            _gfx->print(error_to_str(error_state));
        }
    }

    // Dynamic Battery Info
    if (force_redraw || abs(battery_v - _last_battery_v) >= 0.05f) {
        _gfx->fillRect(10, 110, 300, 40, BLACK); // Clear battery area (moved down slightly)
        _gfx->setTextSize(4);
        _gfx->setCursor(10, 110);
        if (battery_v < 11.0f) _gfx->setTextColor(RED);
        else if (battery_v < 11.5f) _gfx->setTextColor(ORANGE);
        else _gfx->setTextColor(GREEN);
        _gfx->print(battery_v, 1);
        _gfx->print(F(" V"));
    }

    // WiFi Status (Circle + IP)
    if (force_redraw || wifi_connected != _last_wifi_connected || strcmp(wifi_ip, _last_ip) != 0) {
        _gfx->fillCircle(20, 215, 8, wifi_connected ? GREEN : RED);
        _gfx->fillRect(40, 210, 150, 20, BLACK);
        _gfx->setTextSize(2);
        _gfx->setTextColor(WHITE);
        _gfx->setCursor(40, 210);
        _gfx->print(wifi_ip);
    }

    // Joystick Status (Circle)
    if (force_redraw || joystick_connected != _last_joystick_connected) {
        _gfx->fillCircle(220, 215, 8, joystick_connected ? GREEN : RED);
    }

    // Data Activity Status (Circle)
    if (force_redraw || wifi_data_active != _last_wifi_data_active) {
        _gfx->fillCircle(280, 215, 8, wifi_data_active ? GREEN : BLUE); // Use BLUE for active data? User said Red/Green
        // User requested Red/Green for "連線/斷線" but "傳輸資料" is also status.
        // I'll use Green for active, Gray/Red for idle.
        _gfx->fillCircle(280, 215, 8, wifi_data_active ? GREEN : DARKGREY);
    }

    _last_current_mode = current_mode;
    _last_pending_mode = pending_mode;
    _last_error_state = error_state;
    _last_battery_v = battery_v;
    _last_wifi_connected = wifi_connected;
    _last_wifi_data_active = wifi_data_active;
    _last_joystick_connected = joystick_connected;
    strncpy(_last_ip, wifi_ip, 16);
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

const char* HAL_Display::error_to_str(failsafe_error_t error) {
    switch (error) {
        case FS_ERROR_LOOP_SLOW:     return "LOOP SLOW";
        case FS_ERROR_IMU_FAILED:    return "IMU FAILED";
        case FS_ERROR_AHRS_UNREADY:  return "AHRS CALIB";
        case FS_ERROR_CRITICAL_ANGLE:return "TILT ERROR";
        case FS_ERROR_PICKUP_DETECTED:return "PICKUP";
        default:                     return "NONE";
    }
}
