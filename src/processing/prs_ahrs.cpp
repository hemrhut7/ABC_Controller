#include "prs_ahrs.h"
#include "hal/hal_imu.h"
#include <Arduino.h>

Processing_AHRS::Processing_AHRS(uint32_t period_ms) : cpf(period_ms){
    // Constructor
}

Processing_AHRS::~Processing_AHRS() {
    // Destructor
}

bool Processing_AHRS::init()
{
    hal_imu_init();
    last_time = micros();
    is_stable = false;
    return hal_imu_healthy();
}

void Processing_AHRS::update()
{
    if (!hal_imu_healthy()) return;
    hal_imu_read(&ahrs_data.imu_data);
    const uint64_t now = ahrs_data.imu_data.timestamp;

    if (!is_stable) { 
        if (now - last_time > 1000000) {
            is_stable = true;
            last_time = now; // 穩定後重置時間，確保第一幀的 dt 正確
        }
        return;
    }

    if (should_reset_att) {
        cpf.reset_att(ahrs_data.imu_data.accl);
        should_reset_att = false;
        return;
    }
    
    float dt = (now - last_time) * 1e-6f;
    last_time = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; // 保護性濾波，避免異常 dt

    cpf.update(ahrs_data.imu_data.gyro, ahrs_data.imu_data.accl, dt);
    cpf.getEuler(ahrs_data.euler);

    float bias[3];
    cpf.getBias(bias);
    ahrs_data.imu_data_calibrated = ahrs_data.imu_data;
    ahrs_data.imu_data_calibrated.gyro[0] -= bias[0];
    ahrs_data.imu_data_calibrated.gyro[1] -= bias[1];
    ahrs_data.imu_data_calibrated.gyro[2] -= bias[2];
}

void Processing_AHRS::get_ahrs_data(ahrs_data_t *data)
{
    *data = ahrs_data;
}
