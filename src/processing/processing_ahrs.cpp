#include "processing_ahrs.h"
#include "hal/hal_imu.h"
#include <Arduino.h>

Processing_AHRS::Processing_AHRS() {
    // Constructor
}

Processing_AHRS::~Processing_AHRS() {
    // Destructor
}

void Processing_AHRS::init()
{
    hal_imu_init();
    cpf.setInit(200); // 200 Hz
}

void Processing_AHRS::update()
{
    hal_imu_read(&ahrs_data.imu_data);
    float time_s = millis() / 1000.0f;
    cpf.update(time_s, ahrs_data.imu_data.gyro, ahrs_data.imu_data.accl);
}

void Processing_AHRS::get_euler(float euler[3])
{
    cpf.getEuler(euler);
}

void Processing_AHRS::get_imu(imu_data_t *data)
{
    *data = ahrs_data.imu_data;
}
