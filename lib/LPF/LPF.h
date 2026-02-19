#ifndef LPF_H
#define LPF_H

struct LPF_1D {
    LPF_1D(int fs, float f_cut);
    float update(float in);
    void reset() {
        for (int i = 0; i < 3; ++i) {
            x[i] = 0;
            y[i] = 0;
        }
    }
private:
    float a[2];
    float b[3];
    float x[3]; // [n, n-1, n-2]
    float y[3]; // [n, n-1, n-2]
};

struct LPF_3D {
    LPF_3D(int fs, float f_cut);
    void update(const float in[3], float out[3]);
private:
    float a[2];
    float b[3];
    float x[3][3]; // [axis][n, n-1, n-2]
    float y[3][3]; // [axis][n, n-1, n-2]
};

struct AlphaBetaFilter {
    AlphaBetaFilter(float alpha, float beta);
    float update(float measured_pos, float dt);
private:
    float alpha;
    float beta;
    float est_pos;
    float est_vel;
};

#endif // LPF_H
