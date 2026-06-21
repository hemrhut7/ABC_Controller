#ifndef PROCESSING_AHRS_H
#define PROCESSING_AHRS_H

#include "hal/hal_type_define.h"
#include "hal/hal_CPF.h"


class Processing_AHRS {
public:
    Processing_AHRS(float period_ms);
    ~Processing_AHRS();

    bool init();
    void update();
    void reset_att() { should_reset_att = true; }
    void get_ahrs_data(ahrs_data_t *data);
    AHRS_STATE is_ready() { return ahrs_state; }

private:
    CPF cpf;
    ahrs_data_t ahrs_data;
    uint64_t last_time = 0;
    bool is_stable = false;
    bool should_reset_att = false;
    AHRS_STATE ahrs_state = IMU_FAILED;
};

#endif // PROCESSING_AHRS_H
