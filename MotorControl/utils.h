#ifndef __UTILS_H
#define __UTILS_H

#include "drive_main.h"

#ifdef M_PI
#undef M_PI
#endif
#define M_PI         (3.14159265358979323846f)
#define SQ(x)        ((x) * (x))

#define one_by_sqrt3 0.57735026919f
#define two_by_sqrt3 1.15470053838f
#define sqrt3_by_2   0.86602540378f

static __inline int round_int(float x)
{
#ifdef __arm__

    int res;
    float res_f;
    __asm
    {
        vcvtr.s32.f32 res_f, x
        vmov res, res_f
    }
    return res;
#else
    return (int)nearbyint(x);
#endif
}

// Wrap value to range.
// With default rounding mode (round to nearest),
// the result will be in range -y/2 to y/2
static __inline float wrap_pm(float x, float y)
{
#ifdef FPU_FPV4
    float intval = (float)round_int(x / y);
#else
    float intval = nearbyintf(x / y);
#endif
    return x - intval * y;
}

// Same as fmodf but result is positive and y must be positive
static __inline float fmodf_pos(float x, float y)
{
    float res = wrap_pm(x, y);
    if (res < 0) res += y;
    return res;
}

static __inline float wrap_pm_pi(float x) { return wrap_pm(x, 2 * M_PI); }

// Modulo (as opposed to remainder), per https://stackoverflow.com/a/19288271
static __inline int mod(const int dividend, const int divisor)
{
    int r = dividend % divisor;
    if (r < 0) r += divisor;
    return r;
}

float our_arm_sin_f32(float x);
float our_arm_cos_f32(float x);

int SVM(float alpha, float beta, float *tA, float *tB, float *tC);

#endif
