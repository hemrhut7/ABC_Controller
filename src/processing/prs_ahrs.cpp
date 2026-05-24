#include "prs_ahrs.h"
#include "hal/hal_imu.h"
#include <Arduino.h>

#if defined(ESP_PLATFORM)
static portMUX_TYPE ahrs_mux = portMUX_INITIALIZER_UNLOCKED;
#endif

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
    bool imu_healthy = hal_imu_healthy();
    ahrs_state = imu_healthy ? AHRS_INITIALIZING : IMU_FAILED;
    return imu_healthy;
}

void Processing_AHRS::update()
{
    if (!hal_imu_healthy()) {
        ahrs_state = IMU_FAILED;
        return;
    }
    
    imu_data_t temp_imu;
    if (!hal_imu_read(&temp_imu)) {
        // Discard timed out/invalid sample, skip complementary filter update
        return;
    }
    const uint64_t now = temp_imu.timestamp;

    if (!is_stable) { 
        if (now - last_time > 1000000) {
            is_stable = true;
            last_time = now; // 穩定後重置時間，確保第一幀的 dt 正確
        }
        return;
    }

    if (should_reset_att) {
        cpf.reset_att(temp_imu.accl);
        should_reset_att = false;
        return;
    }
    
    float dt = (now - last_time) * 1e-6f;
    last_time = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; // 保護性濾波，避免異常 dt

    cpf.update(temp_imu.gyro, temp_imu.accl, dt);
    
    float euler_temp[3];
    cpf.getEuler(euler_temp);
    if (cpf.is_ready()) ahrs_state = AHRS_READY;

    float bias[3];
    cpf.getBias(bias);
    
    imu_data_t temp_calibrated = temp_imu;
    temp_calibrated.gyro[0] -= bias[0];
    temp_calibrated.gyro[1] -= bias[1];
    temp_calibrated.gyro[2] -= bias[2];

    // Thread-safe update of the shared data structure
#if defined(ESP_PLATFORM)
    portENTER_CRITICAL(&ahrs_mux);
#endif
    ahrs_data.imu_data = temp_imu;
    ahrs_data.euler[0] = euler_temp[0];
    ahrs_data.euler[1] = euler_temp[1];
    ahrs_data.euler[2] = euler_temp[2];
    ahrs_data.imu_data_calibrated = temp_calibrated;
#if defined(ESP_PLATFORM)
    portEXIT_CRITICAL(&ahrs_mux);
#endif
}

void Processing_AHRS::get_ahrs_data(ahrs_data_t *data)
{
    // Thread-safe copy of the data structure to prevent data tearing
#if defined(ESP_PLATFORM)
    portENTER_CRITICAL(&ahrs_mux);
#endif
    *data = ahrs_data;
#if defined(ESP_PLATFORM)
    portEXIT_CRITICAL(&ahrs_mux);
#endif
}
