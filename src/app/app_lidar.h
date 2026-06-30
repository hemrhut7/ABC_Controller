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
    
    // Get latest scan
    void get_latest_scan(lidar_scan_t &scan);

    // Register telemetry for new scan notifications
    void register_telemetry(Telemetry *telemetry);

    // Single step update called by external Sensor_Task
    void update_step();

private:
    HAL_Lidar _lidar;
    lidar_scan_t *_latest_scan = nullptr;
    lidar_scan_t *_current_scan = nullptr;
    
    // Mutex for latest scan access
    SemaphoreHandle_t _mutex;
    
    uint32_t _last_diag_ms = 0;
    uint32_t _scan_count = 0;
    uint32_t _last_data_ms = 0;

    std::vector<Telemetry*> _telemetry_list;
};

#endif // APP_LIDAR_H
