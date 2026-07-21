#include "app_lidar.h"
#include "hal/hal_telemetry.h"


AppLidar::AppLidar(Stream &serial) : _lidar(serial) {
    _scan_count = 0;
    _last_data_ms = 0;
}
 
AppLidar::~AppLidar() {
}

void AppLidar::update_step() {
    if (_lidar.update(_current_scan)) {
        _scan_count++;
        _last_data_ms = millis();

        // New scan completed
        Telemetry::getInstance().push_lidar_data(_current_scan);
    }
}
