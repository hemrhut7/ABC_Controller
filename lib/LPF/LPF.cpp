#include "LPF.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LPF_1D::LPF_1D(int fs, float f_cut) {
    float wc = 2.0 * M_PI * f_cut;
    float T = 1.0 / fs;
    float K = tan(wc * T / 2.0);
    float K2 = K * K;
    float norm = 1.0 + sqrt(2.0) * K + K2;

    b[0] = K2 / norm;
    b[1] = 2.0 * b[0];
    b[2] = b[0];

    a[0] = (2.0 * (K2 - 1.0)) / norm;
    a[1] = (1.0 - sqrt(2.0) * K + K2) / norm;

    for (int i = 0; i < 3; ++i) {
        x[i] = 0;
        y[i] = 0;
    }
}

float LPF_1D::update(float in) {
    if (x[0] == 0 && x[1] == 0 && x[2] == 0) {
        for (int i = 0; i < 3; ++i) {
            x[i] = in;
            y[i] = in;
        }
        return in;
    }

    x[0] = in;
    y[0] = b[0] * x[0] + b[1] * x[1] + b[2] * x[2] - a[0] * y[1] - a[1] * y[2];
    
    x[2] = x[1];
    x[1] = x[0];
    y[2] = y[1];
    y[1] = y[0];

    return y[0];
}

LPF_3D::LPF_3D(int fs, float f_cut) {
    float wc = 2.0 * M_PI * f_cut;
    float T = 1.0 / fs;
    float K = tan(wc * T / 2.0);
    float K2 = K * K;
    float norm = 1.0 + sqrt(2.0) * K + K2;

    b[0] = K2 / norm;
    b[1] = 2.0 * b[0];
    b[2] = b[0];

    a[0] = (2.0 * (K2 - 1.0)) / norm;
    a[1] = (1.0 - sqrt(2.0) * K + K2) / norm;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            x[i][j] = 0;
            y[i][j] = 0;
        }
    }
}

void LPF_3D::update(const float in[3], float out[3]) {
    for (int i = 0; i < 3; ++i) {
        x[i][0] = in[i];
        y[i][0] = b[0] * x[i][0] + b[1] * x[i][1] + b[2] * x[i][2] - a[0] * y[i][1] - a[1] * y[i][2];
        out[i] = y[i][0];

        x[i][2] = x[i][1];
        x[i][1] = x[i][0];
        y[i][2] = y[i][1];
        y[i][1] = y[i][0];
    }
}

AlphaBetaFilter::AlphaBetaFilter(float alpha, float beta) {
    this->alpha = alpha;
    this->beta = beta;
    this->est_pos = 0.0f;
    this->est_vel = 0.0f;
}

float AlphaBetaFilter::update(float measured_pos, float dt) {
    // 1. 預測步
    float pred_pos = est_pos + (est_vel * dt);
    
    // 2. 計算殘差 & 3. 更新步
    float residual = measured_pos - pred_pos;
    est_pos = pred_pos + alpha * residual;
    
    if (dt > 0.0f) {
        est_vel = est_vel + (beta / dt) * residual;
    }

    return est_vel;
}
