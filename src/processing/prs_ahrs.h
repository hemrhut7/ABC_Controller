#ifndef PROCESSING_AHRS_H
#define PROCESSING_AHRS_H

#include "hal/hal_type_define.h"
#include "hal/hal_CPF.h"

class Processing_AHRS {
public:
    Processing_AHRS(uint32_t period_ms);
    ~Processing_AHRS();

    void init();
    void update();
    void get_ahrs_data(ahrs_data_t *data);
    bool is_ready() { return cpf.is_ready(); }

private:
    CPF cpf;
    ahrs_data_t ahrs_data;
    uint64_t last_time = 0;
    bool is_stable = false;
};

#endif // PROCESSING_AHRS_H
