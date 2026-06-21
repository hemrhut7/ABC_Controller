#include "app_lidar.h"

AppLidar::AppLidar(Stream &serial) : _lidar(serial) {
    _mutex = xSemaphoreCreateMutex();
    _latest_scan = (lidar_scan_t *)heap_caps_malloc(sizeof(lidar_scan_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (_latest_scan) {
        _latest_scan->count = 0;
    }
    _current_scan = nullptr;
    _last_diag_ms = 0;
    _scan_count = 0;
    _last_data_ms = 0;
}
 
AppLidar::~AppLidar() {
    if (_latest_scan) {
        free(_latest_scan);
        _latest_scan = nullptr;
    }
    if (_current_scan) {
        free(_current_scan);
        _current_scan = nullptr;
    }
    if (_mutex) {
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }
}
 void AppLidar::init() {
    _lidar.init();
}

void AppLidar::get_latest_scan(lidar_scan_t &scan) {
    if (_latest_scan && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        scan = *_latest_scan;
        xSemaphoreGive(_mutex);
    }
}

void AppLidar::register_telemetry(Telemetry *telemetry) {
    if (telemetry) {
        _telemetry_list.push_back(telemetry);
    }
}

void AppLidar::update_step() {
    if (!_current_scan) {
        _current_scan = (lidar_scan_t *)heap_caps_malloc(sizeof(lidar_scan_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!_current_scan) {
            Serial.println("[LIDAR] PSRAM alloc failed for scan buffer");
            return;
        }
        _current_scan->count = 0;
        _last_diag_ms = millis();
        _scan_count = 0;
        _last_data_ms = millis();
    }

    if (_lidar.update(*_current_scan)) {
        _scan_count++;
        _last_data_ms = millis();

        // New scan completed
        if (_latest_scan && xSemaphoreTake(_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            *_latest_scan = *_current_scan;
            xSemaphoreGive(_mutex);

            // Notify telemetry
            for (auto tele : _telemetry_list) {
                tele->update_lidar_data(*_current_scan);
            }
        }
    }
    
    // --- Diagnostic Print (1Hz) ---
    uint32_t now = millis();
    if (now - _last_diag_ms >= 1000) {
        if (_scan_count > 0) {
            Serial.printf("[LIDAR DIAG] Scans/sec: %d, Last Pts: %d\n", _scan_count, _current_scan->count);
        } else if (now - _last_data_ms > 2000) {
            Serial.println("[LIDAR DIAG] WARNING: No data received for 2 seconds!");
        }
        _scan_count = 0;
        _last_diag_ms = now;
    }
}
