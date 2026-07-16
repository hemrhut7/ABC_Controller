#ifndef HAL_LIDAR_H
#define HAL_LIDAR_H

#include <Arduino.h>
#include "hal_type_define.h"

class HAL_Lidar {
public:
    HAL_Lidar(Stream &stream);
    ~HAL_Lidar();
    void init();
    
    /**
     * @brief Process incoming bytes from the lidar.
     * @param scan Out parameter. Filled with a complete scan if return is true.
     * @return true if a complete scan (full revolution) is ready.
     */
    bool update(lidar_scan_t &scan);

private:
    Stream &_port;
    
    // Packet parsing state
    uint8_t _packet_buffer[58];
    uint8_t _buffer_idx = 0;
    
    float _last_angle = 0;
    
    uint8_t calculate_crc8(const uint8_t *data, uint8_t len);
    
    uint8_t pt_count = 0;
    uint32_t _overflow_count = 0;
};

#endif // HAL_LIDAR_H
