#include "hal_lidar.h"
#include <math.h>

HAL_Lidar::HAL_Lidar(Stream &stream) : _port(stream) {
    _current_scan = (lidar_scan_t *)malloc(sizeof(lidar_scan_t));
    if (_current_scan) {
        _current_scan->count = 0;
    }
}
 
HAL_Lidar::~HAL_Lidar() {
    if (_current_scan) {
        free(_current_scan);
        _current_scan = nullptr;
    }
}
 void HAL_Lidar::init() {
    // Port should be initialized externally (e.g. Serial2.begin(230400))
}

uint8_t HAL_Lidar::calculate_crc8(const uint8_t *data, uint8_t len) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

bool HAL_Lidar::update(lidar_scan_t &scan) {
    if (!_current_scan) return false;
    bool revolution_completed = false;

    while (_port.available()) {
        uint8_t b = _port.read();

        if (_buffer_idx == 0 && b != 0xA5) continue;
        if (_buffer_idx == 1 && b != 0x5A) {
            _buffer_idx = 0;
            continue;
        }

        _packet_buffer[_buffer_idx++] = b;

        if (_buffer_idx == 58) {
            // Check CRC
            if (_packet_buffer[57] == calculate_crc8(_packet_buffer, 57)) {
                
                // Extract angles
                float start_angle = (_packet_buffer[5] * 256 + _packet_buffer[6]) / 100.0f;
                float end_angle = (_packet_buffer[55] * 256 + _packet_buffer[56]) / 100.0f;

                // Handle angle wrap
                float angle_diff;
                if (end_angle < start_angle) {
                    angle_diff = (end_angle + 360.0f) - start_angle;
                } else {
                    angle_diff = end_angle - start_angle;
                }

                // Check for revolution completion
                if (start_angle < _last_angle - 180.0f) {
                    if (_current_scan && _current_scan->count > 0) {
                        _current_scan->timestamp = micros();
                        scan = *_current_scan;
                        revolution_completed = true;
                        _current_scan->count = 0; // Reset for next scan
                    }
                }
                _last_angle = start_angle;

                // Parse 16 points
                const uint8_t points_in_packet = 16;
                for (int i = 0; i < points_in_packet; i++) {
                    if (!_current_scan || _current_scan->count >= MAX_LIDAR_POINTS) {
                        _overflow_count++;
                        break;
                    }

                    uint8_t idx = 7 + i * 3;
                    uint16_t dist_raw = _packet_buffer[idx] * 256 + _packet_buffer[idx + 1];
                    uint8_t intensity = _packet_buffer[idx + 2];

                    if (dist_raw == 0xFFFF) continue; // Invalid point

                    float distance = dist_raw / 1000.0f; // meters
                    float angle = start_angle + (angle_diff / (float)(points_in_packet - 1)) * i;
                    if (angle >= 360.0f) angle -= 360.0f;

                    // Polar to Cartesian (Matching Python logic)
                    // Python: angle_rad = -np.deg2rad(angle)
                    float angle_rad = -(angle * M_PI / 180.0f);
                    float x = distance * cosf(angle_rad);
                    float y = distance * sinf(angle_rad);

                    lidar_point_t &p = _current_scan->points[_current_scan->count++];
                    p.x = x;
                    p.y = y;
                    p.distance = distance;
                    p.angle = angle;
                    p.intensity = intensity;
                }
            }
            _buffer_idx = 0; // Reset for next packet
            
            // If a revolution was just detected, return true
            if (revolution_completed) return true;
        }
    }

    return false;
}
