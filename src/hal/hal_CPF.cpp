#include "hal_CPF.h"
#include <cmath>
#include <numeric>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper for vector operations
namespace Vec {
    void add(const float a[3], const float b[3], float out[3]) {
        for (int i = 0; i < 3; ++i) out[i] = a[i] + b[i];
    }
    void sub(const float a[3], const float b[3], float out[3]) {
        for (int i = 0; i < 3; ++i) out[i] = a[i] - b[i];
    }
    void scale(const float a[3], float s, float out[3]) {
        for (int i = 0; i < 3; ++i) out[i] = a[i] * s;
    }
    float dot(const float a[3], const float b[3]) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }
}


CPF::CPF(uint32_t period_ms) : lpf_acc(1000 / period_ms, 5) {
    first_update = true;
    std::fill(pre_omg, pre_omg + 3, 0.0f);
    g0 = 9.80665f;
    weight = 0.01f;
    acc_error = 0.0f;
    gyro_error = 0.0f;
    enable_check_acc = true;
    
    float euler[3] = {0};
    gen_dcm_by_euler(euler, dcm);

    lc_list_count = 0;
    std::fill(bias_omg, bias_omg + 3, 0.0f);
    std::fill(omg_threshold, omg_threshold + 3, 0.1f);

    dt = period_ms * 1e-3f;
    float fs = 1000.0f / period_ms;
    K_bias = 1.0f / fs;
    LC_WINDOW_SIZE = fs * LC_WINDOW_SEC;
    LC_list = new float[LC_WINDOW_SIZE][3];
    setInit((int)fs);
}

CPF::~CPF() {
    delete[] LC_list;
}

void CPF::setInit(int fs) {
    // Values from MTI7_params in SensorParams.py
    float gyro_ARW_deg_hr_sqrt = 0.7f;
    float accl_VRW_ug_Hz_sqrt = 150.0f;
    float gyro_RRW_deg_hr_1_5 = 20.0f;
    
    float gyro_ARW = gyro_ARW_deg_hr_sqrt * (M_PI/180.0f) / 60.0f; // to rad/s^0.5
    float accl_VRW = accl_VRW_ug_Hz_sqrt * 9.8e-6f; // to m/s/s^0.5
    float gyro_RRW = gyro_RRW_deg_hr_1_5 * (M_PI/180.0f) / pow(3600, 1.5); // to rad/s^1.5

    gyro_error = sqrt(pow(gyro_ARW, 2) * dt + pow(gyro_RRW, 2) * pow(dt, 3));
    acc_error = accl_VRW / g0 * sqrt(fs);
    weight = gyro_error / (gyro_error + acc_error);
    
    K_bias = 1.0f / fs;
}

void CPF::enableCheckACC(bool is_enable) {
    enable_check_acc = is_enable;
}

void CPF::update(float omg[3], float acc[3]) {
    if (first_update) {
        first_update = false;
        
        if (lc_list_count < LC_WINDOW_SIZE) {
            std::copy(omg, omg + 3, LC_list[lc_list_count]);
            lc_list_count++;
        } else {
            if (lc_list_count == LC_WINDOW_SIZE) {
                for(int i=0; i<3; ++i) {
                    float sum = 0;
                    for(int j=0; j < LC_WINDOW_SIZE; ++j) {
                        sum += LC_list[j][i];
                    }
                    bias_omg[i] = sum / LC_WINDOW_SIZE;
                }
                lc_list_count++; // Increment to prevent re-calculation
            }
            
            float avg_omg[3];
            Vec::add(omg, pre_omg, avg_omg);
            Vec::scale(avg_omg, 0.5f, avg_omg);
            
            float vec_rotation[3];
            Vec::sub(avg_omg, bias_omg, vec_rotation);
            Vec::scale(vec_rotation, dt, vec_rotation);

            rotate_dcm_by_vec_b(dcm, vec_rotation);
            
            float current_w = 0.0f;
            check_acc(acc, current_w);
            current_w *= weight;
            
            if (current_w > 0) {
                float g_b[3] = {-dcm[2][0], -dcm[2][1], -dcm[2][2]};
                float acc_norm_val = norm(acc);
                if (acc_norm_val > 1e-6) {
                    float acc_n[3];
                    Vec::scale(acc, 1.0f / acc_norm_val, acc_n);

                    float error_vec[3];
                    cross_product(g_b, acc_n, error_vec);
                    
                    float correction_vec[3];
                    Vec::scale(error_vec, current_w, correction_vec);
                    rotate_dcm_by_vec_b(dcm, correction_vec);
                    
                    float bias_update[3];
                    Vec::scale(error_vec, -current_w * K_bias, bias_update);
                    Vec::add(bias_omg, bias_update, bias_omg);
                }
            }
        }
    } else {
        float p, r;
        accLeveling(acc, p, r);
        euler[0] = p;
        euler[1] = r;
        gen_dcm_by_euler(euler, dcm);
    }

    std::copy(omg, omg + 3, pre_omg);
    gen_euler_by_dcm(dcm, euler);
}

void CPF::check_acc(const float acc[3], float& current_weight) {
    if (enable_check_acc) {
        float acc_filtered[3];
        lpf_acc.update(acc, acc_filtered);
        float error = norm(acc_filtered) - g0;
        float D = std::abs(error / acc_error);

        if (D > 3.0f) {
            current_weight = 0.0f;
        } else if (D > 2.0f) {
            current_weight = 3.0f - D;
        } else {
            current_weight = 1.0f;
        }
    } else {
        current_weight = 1.0f;
    }
}


void CPF::getEuler(float e[3]) {
    std::copy(euler, euler + 3, e);
}

void CPF::getBias(float b[3]) {
    std::copy(bias_omg, bias_omg + 3, b);
}

// Orientation and math helpers

void CPF::accLeveling(const float acc[3], float& pitch, float& roll) {
    pitch = atan2(-acc[1], -acc[2]);
    roll = atan2(acc[0], sqrt(acc[1] * acc[1] + acc[2] * acc[2]));
}

void CPF::gen_dcm_by_euler(const float euler[3], float dcm[3][3]) {
    float p = -euler[0]; // pitch
    float r = -euler[1]; // roll
    float y = -euler[2]; // yaw

    float cp = cos(p), sp = sin(p);
    float cr = cos(r), sr = sin(r);
    float cy = cos(y), sy = sin(y);

    float Rx[3][3] = {{1, 0, 0}, {0, cp, sp}, {0, -sp, cp}};
    float Ry[3][3] = {{cr, 0, -sr}, {0, 1, 0}, {sr, 0, cr}};
    float Rz[3][3] = {{cy, sy, 0}, {-sy, cy, 0}, {0, 0, 1}};
    
    float temp[3][3];
    mat_mult(Rz, Rx, temp);
    mat_mult(temp, Ry, dcm);
}

void CPF::gen_euler_by_dcm(const float dcm[3][3], float euler[3]) {
    euler[0] = atan2(dcm[2][1], sqrt(dcm[0][1] * dcm[0][1] + dcm[1][1] * dcm[1][1]));
    euler[1] = -atan2(-dcm[2][0], dcm[2][2]);
    euler[2] = -atan2(-dcm[0][1], dcm[1][1]);
}


void vector2skew(const float vec[3], float res[3][3]) {
    res[0][0] = 0;      res[0][1] = -vec[2]; res[0][2] = vec[1];
    res[1][0] = vec[2]; res[1][1] = 0;       res[1][2] = -vec[0];
    res[2][0] = -vec[1]; res[2][1] = vec[0];  res[2][2] = 0;
}

void CPF::rotate_dcm_by_vec_b(float dcm[3][3], const float vec[3]) {
    float theta = norm(vec);
    if (theta <= 1e-9) return;

    float sk_w[3][3];
    vector2skew(vec, sk_w);

    float s = sin(theta);
    float c = 1 - cos(theta);
    
    float R_update[3][3];
    for(int i=0; i<3; ++i) R_update[i][i] = 1.0;

    float sk_w_sq[3][3];
    mat_mult(sk_w, sk_w, sk_w_sq);

    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) {
            R_update[i][j] += (s/theta) * sk_w[i][j] + (c/(theta*theta)) * sk_w_sq[i][j];
        }
    }
    
    float new_dcm[3][3];
    mat_mult(dcm, R_update, new_dcm);
    std::copy(&new_dcm[0][0], &new_dcm[0][0] + 9, &dcm[0][0]);
}

// Generic math functions
void CPF::cross_product(const float a[3], const float b[3], float result[3]) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

float CPF::norm(const float a[3]) {
    return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

void CPF::mat_vec_mult(const float mat[3][3], const float vec[3], float result[3]) {
    for (int i = 0; i < 3; ++i) {
        result[i] = mat[i][0] * vec[0] + mat[i][1] * vec[1] + mat[i][2] * vec[2];
    }
}

void CPF::mat_transpose_vec_mult(const float mat[3][3], const float vec[3], float result[3]) {
    for (int i = 0; i < 3; ++i) {
        result[i] = mat[0][i] * vec[0] + mat[1][i] * vec[1] + mat[2][i] * vec[2];
    }
}

void CPF::mat_mult(const float a[3][3], const float b[3][3], float result[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}
