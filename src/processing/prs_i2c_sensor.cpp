#include "prs_i2c_sensor.h"

Processing_I2CSensor::Processing_I2CSensor(float loop_rate_hz) : _loop_rate_hz(loop_rate_hz), _loop_cnt(0) {
    _cached_mag = {0};
    _cached_baro = {0};

    // Calculate intervals once on initialization to avoid divisions in the loop
    _mag_interval = (uint32_t)(_loop_rate_hz / MAG_ODR);
    if (_mag_interval < 1) _mag_interval = 1;

    _baro_interval = (uint32_t)(_loop_rate_hz / BARO_ODR);
    if (_baro_interval < 1) _baro_interval = 1;
}

Processing_I2CSensor::~Processing_I2CSensor() {
    // Destructor
}

void Processing_I2CSensor::init() {
    hal_mag_init();
    hal_baro_init();
}

void Processing_I2CSensor::update() {
    _loop_cnt++;
    if (_loop_cnt % _mag_interval == 0) {
        hal_mag_read(&_cached_mag);
    }
    if ((_loop_cnt + 1) % _baro_interval == 0) {
        hal_baro_read(&_cached_baro);
    }
}

void Processing_I2CSensor::get_sensor_data(mag_data_t *mag, baro_data_t *baro) const {
    if (mag) *mag = _cached_mag;
    if (baro) *baro = _cached_baro;
}
