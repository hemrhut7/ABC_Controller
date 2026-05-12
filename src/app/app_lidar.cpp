#include "app_lidar.h"

AppLidar::AppLidar(Stream &serial) : _lidar(serial) {
    _mutex = xSemaphoreCreateMutex();
    _latest_scan = (lidar_scan_t *)heap_caps_malloc(sizeof(lidar_scan_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (_latest_scan) {
        _latest_scan->count = 0;
    }
}
 
AppLidar::~AppLidar() {
    if (_latest_scan) {
        free(_latest_scan);
        _latest_scan = nullptr;
    }
    if (_mutex) {
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }
}
 void AppLidar::init() {
    _lidar.init();
}

void AppLidar::task_entry(void *pvParameters) {
    AppLidar *instance = (AppLidar *)pvParameters;
    instance->run();
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

void AppLidar::run() {
    lidar_scan_t *current_scan = (lidar_scan_t *)heap_caps_malloc(sizeof(lidar_scan_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!current_scan) {
        Serial.println("[LIDAR] PSRAM alloc failed for scan buffer");
        vTaskDelete(NULL);
        return;
    }
    
    uint32_t last_diag_ms = millis();
    uint32_t scan_count = 0;
    uint32_t last_data_ms = millis();

    for (;;) {
        // Lidar update internally handles serial reading
        if (_lidar.update(*current_scan)) {
            scan_count++;
            last_data_ms = millis();

            // New scan completed
            if (_latest_scan && xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
                *_latest_scan = *current_scan;
                xSemaphoreGive(_mutex);

                // Notify telemetry
                for (auto tele : _telemetry_list) {
                    tele->update_lidar_data(*current_scan);
                }
            }
        }
        
        // --- Diagnostic Print (1Hz) ---
        uint32_t now = millis();
        if (now - last_diag_ms >= 1000) {
            if (scan_count > 0) {
                Serial.printf("[LIDAR DIAG] Scans/sec: %d, Last Pts: %d\n", scan_count, current_scan->count);
            } else if (now - last_data_ms > 2000) {
                Serial.println("[LIDAR DIAG] WARNING: No data received for 2 seconds!");
            }
            scan_count = 0;
            last_diag_ms = now;
        }

        // Small delay to prevent watchdog issues
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    free(current_scan);
}
