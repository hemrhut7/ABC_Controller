#ifndef APP_LIDAR_H
#define APP_LIDAR_H

#include <Arduino.h>
#include <vector>
#include "hal/hal_lidar.h"

class AppLidar {
public:
    AppLidar(Stream &serial);
    ~AppLidar();
    void init() { _lidar.init(); }
    
    // Single step update called by external Sensor_Task
    void update_step();

private:
    HAL_Lidar _lidar;
    lidar_scan_t _current_scan;    
    uint32_t _scan_count = 0;
    uint32_t _last_data_ms = 0;
};

#endif // APP_LIDAR_H
