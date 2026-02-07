#ifndef CPF_H
#define CPF_H

#define LC_WINDOW_SIZE 200

#include "LPF.h"

class CPF {
public:
    CPF(float pitch = 0, float roll = 0, float yaw = 0);
    void setInit(int fs = 100);
    void update(float t, float omg[3], float acc[3]);
    void getEuler(float euler[3]);
    void getBias(float bias[3]);
    void enableCheckACC(bool is_enable);

private:
    float time;
    float pre_omg[3];
    float g0;
    float weight;
    float acc_error;
    float gyro_error;
    bool enable_check_acc;

    float dcm[3][3];
    float euler[3];

    float LC_list[LC_WINDOW_SIZE][3];
    int lc_list_count;
    float bias_omg[3];
    float omg_threshold[3];
    
    LPF_3D lpf_acc;

    float K_bias;

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
