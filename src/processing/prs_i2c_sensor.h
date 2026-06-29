#pragma once

#include "hal/hal_mag.h"
#include "hal/hal_baro.h"

#define MAG_ODR  20.0f
#define BARO_ODR 10.0f

class Processing_I2CSensor {
public:
    Processing_I2CSensor(float loop_rate_hz);
    ~Processing_I2CSensor();

    void init();
    void update();
    void get_sensor_data(mag_data_t *mag, baro_data_t *baro) const;

private:
    float _loop_rate_hz;
    uint32_t _mag_interval;
    uint32_t _baro_interval;
    uint32_t _loop_cnt;

    mutable mag_data_t _cached_mag;
    mutable baro_data_t _cached_baro;
};
