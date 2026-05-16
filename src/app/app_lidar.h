#ifndef APP_LIDAR_H
#define APP_LIDAR_H

#include <Arduino.h>
#include <vector>
#include "hal/hal_lidar.h"
#include "hal/hal_telemetry.h"

class AppLidar {
public:
    AppLidar(Stream &serial);
    ~AppLidar();
    void init();
    
    // Task entry point for FreeRTOS
    static void task_entry(void *pvParameters);
    
    // Get latest scan
    void get_latest_scan(lidar_scan_t &scan);

    // Register telemetry for new scan notifications
    void register_telemetry(Telemetry *telemetry);

private:
    HAL_Lidar _lidar;
    lidar_scan_t *_latest_scan = nullptr;
    
    // Mutex for latest scan access
    SemaphoreHandle_t _mutex;
    
    void run();

    std::vector<Telemetry*> _telemetry_list;
};

#endif // APP_LIDAR_H
