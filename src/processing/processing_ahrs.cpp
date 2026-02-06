#include "processing_ahrs.h"
#include "hal/hal_imu.h"
#include "hal/hal_CPF.h"
#include <Arduino.h>

static CPF cpf;
static ahrs_data_t ahrs_data;

void processing_ahrs_init()
{
    hal_imu_init();
    cpf.setInit(200); // 200 Hz
}

void processing_ahrs_update()
{
    hal_imu_read(&ahrs_data.imu_data);
    float time_s = millis() / 1000.0f;
    cpf.update(time_s, ahrs_data.imu_data.gyro, ahrs_data.imu_data.accl);
}

void processing_ahrs_get_euler(float euler[3])
{
    cpf.getEuler(euler);
}

void processing_ahrs_get_imu(imu_data_t *data)
{
    *data = ahrs_data.imu_data;
}
