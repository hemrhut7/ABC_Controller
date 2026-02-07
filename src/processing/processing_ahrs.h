#ifndef PROCESSING_AHRS_H
#define PROCESSING_AHRS_H

#include "hal/hal_type_define.h"
#include "hal/hal_CPF.h"

class Processing_AHRS {
public:
    Processing_AHRS();
    ~Processing_AHRS();

    void init();
    void update();
    void get_euler(float euler[3]);
    void get_imu(imu_data_t *data);

private:
    CPF cpf;
    ahrs_data_t ahrs_data;
};

#endif // PROCESSING_AHRS_H
