#include "prs_ahrs.h"
#include "hal/hal_imu.h"
#include <Arduino.h>

Processing_AHRS::Processing_AHRS(uint32_t period_ms) : cpf(period_ms){
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
    cpf.update(ahrs_data.imu_data.gyro, ahrs_data.imu_data.accl);
    cpf.getEuler(ahrs_data.euler);
}

void Processing_AHRS::get_ahrs_data(ahrs_data_t *data)
{
    *data = ahrs_data;
}
