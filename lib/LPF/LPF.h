#ifndef LPF_H
#define LPF_H

struct LPF_3D {
    LPF_3D(int fs, float f_cut);
    void update(const float in[3], float out[3]);
private:
    float a[2];
    float b[3];
    float x[3][3]; // [axis][n, n-1, n-2]
    float y[3][3]; // [axis][n, n-1, n-2]
};

#endif // LPF_H
