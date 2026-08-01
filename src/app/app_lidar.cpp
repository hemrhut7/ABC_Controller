#include "app_lidar.h"
#include "hal/hal_telemetry.h"
#include "hal/hal_microros.h"

extern HAL_MicroROS uros_telemetry;


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

        // push data to the queue of transmission
        if (uros_telemetry.is_connected()) {
            uros_telemetry.push_lidar_data(_current_scan);
        } else if (Telemetry::getInstance().is_transmitting()) {
            Telemetry::getInstance().push_lidar_data(_current_scan);
        }
    }
}
