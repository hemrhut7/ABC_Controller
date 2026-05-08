#include "app_lidar.h"

AppLidar::AppLidar(Stream &serial) : _lidar(serial) {
    _mutex = xSemaphoreCreateMutex();
    _latest_scan = (lidar_scan_t *)malloc(sizeof(lidar_scan_t));
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
    lidar_scan_t current_scan;
    for (;;) {
        // Lidar update internally handles serial reading
        if (_lidar.update(current_scan)) {
            // New scan completed
            if (_latest_scan && xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
                *_latest_scan = current_scan;
                xSemaphoreGive(_mutex);

                // Notify telemetry
                for (auto tele : _telemetry_list) {
                    tele->update_lidar_data(current_scan);
                }
            }
        }
        
        // Small delay to prevent watchdog issues, although lidar.update() 
        // is mostly non-blocking serial read.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
