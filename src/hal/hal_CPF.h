#ifndef CPF_H
#define CPF_H

#define LC_WINDOW_SEC 3
#define LPF_FREQ_ACCL_CHECK 5
#define RESET_ATT_ERR_THR 5.0f * M_PI / 180.0f
#define RESET_ATT_PITCH_THR 50.0f * M_PI / 180.0f
#define RESET_ATT_MAX_CNT 200

#include "LPF.h"
#include <Arduino.h>


class CPF {
public:
    CPF(uint32_t period_ms);
    ~CPF();
    void setInit(int fs = 100);
    void update(float omg[3], float acc[3], float dt);
    void reset_att(float acc[3]);
    void getEuler(float euler[3]);
    void getBias(float bias[3]);
    void enableCheckACC(bool is_enable);
    bool is_ready () { return is_initialized; }

private:
    bool is_initialized = false;
    float dt = 0.005f;
    float pre_omg[3];
    float g0;
    float weight;
    float acc_error;
    float gyro_error;
    bool enable_check_acc;
    float lever_arm[3] = {-0.01, 0, 0.08};

    float dcm[3][3];
    float euler[3];

    float (*LC_list)[3];
    uint16_t lc_list_count = 0;
    uint16_t LC_WINDOW_SIZE = 0;
    float bias_omg[3];
    float omg_threshold[3];
    
    LPF_3D lpf_acc;
    uint8_t reset_counter = 0;

    float K_bias;

    void check_reset_att(const float acc_cg[3], bool acc_is_reliable);
    void check_acc(const float acc[3], float& current_weight);

    // Orientation functions
    void gen_dcm_by_euler(const float euler[3], float dcm[3][3]);
    void gen_euler_by_dcm(const float dcm[3][3], float euler[3]);
    void rotate_dcm_by_vec_b(float dcm[3][3], const float vec[3]);
    void accLeveling(const float acc[3], float& pitch, float& roll);

    // Vector/Matrix operations
    void cross_product(const float a[3], const float b[3], float result[3]);
    float norm(const float a[3]);
    void mat_vec_mult(const float mat[3][3], const float vec[3], float result[3]);
    void mat_transpose_vec_mult(const float mat[3][3], const float vec[3], float result[3]);
    void mat_mult(const float a[3][3], const float b[3][3], float result[3][3]);
};

#endif // CPF_H
