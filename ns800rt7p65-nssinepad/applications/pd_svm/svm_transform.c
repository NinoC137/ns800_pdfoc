/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "svm_transform.h"

#include "svm_config.h"

#if SVM_USE_NS800_MMATH
#include "mmath.h"
#endif

#if !SVM_USE_NS800_MMATH
static const float svm_sin_quarter_lut[SVM_LUT_QUARTER_SIZE + 1u] = {
    0.000000000f, 0.024541229f, 0.049067674f, 0.073564564f,
    0.098017140f, 0.122410675f, 0.146730474f, 0.170961889f,
    0.195090322f, 0.219101240f, 0.242980180f, 0.266712757f,
    0.290284677f, 0.313681740f, 0.336889853f, 0.359895037f,
    0.382683432f, 0.405241314f, 0.427555093f, 0.449611330f,
    0.471396737f, 0.492898192f, 0.514102744f, 0.534997620f,
    0.555570233f, 0.575808191f, 0.595699304f, 0.615231591f,
    0.634393284f, 0.653172843f, 0.671558955f, 0.689540545f,
    0.707106781f, 0.724247083f, 0.740951125f, 0.757208847f,
    0.773010453f, 0.788346428f, 0.803207531f, 0.817584813f,
    0.831469612f, 0.844853565f, 0.857728610f, 0.870086991f,
    0.881921264f, 0.893224301f, 0.903989293f, 0.914209756f,
    0.923879533f, 0.932992799f, 0.941544065f, 0.949528181f,
    0.956940336f, 0.963776066f, 0.970031253f, 0.975702130f,
    0.980785280f, 0.985277642f, 0.989176510f, 0.992479535f,
    0.995184727f, 0.997290457f, 0.998795456f, 0.999698819f,
    1.000000000f
};
#endif

static float svm_wrap_angle(float angle_rad)
{
    while (angle_rad >= SVM_TWO_PI) angle_rad -= SVM_TWO_PI;
    while (angle_rad < 0.0f) angle_rad += SVM_TWO_PI;
    return angle_rad;
}

#if !SVM_USE_NS800_MMATH
static float svm_abs_f32(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float svm_lut_sin_positive_quarter(float angle_rad)
{
    float scaled = angle_rad * ((float)SVM_LUT_QUARTER_SIZE / SVM_HALF_PI);
    uint32_t idx = (uint32_t)scaled;
    float frac;
    float y0;
    float y1;

    if (idx >= SVM_LUT_QUARTER_SIZE) return 1.0f;
    frac = scaled - (float)idx;
    y0 = svm_sin_quarter_lut[idx];
    y1 = svm_sin_quarter_lut[idx + 1u];
    return y0 + (y1 - y0) * frac;
}

static float svm_lut_sin(float angle_rad)
{
    float a = svm_wrap_angle(angle_rad);
    float sign = 1.0f;

    if (a > SVM_PI) {
        a -= SVM_PI;
        sign = -1.0f;
    }
    if (a > SVM_HALF_PI) {
        a = SVM_PI - a;
    }
    return sign * svm_lut_sin_positive_quarter(svm_abs_f32(a));
}
#endif

void svm_clarke(const svm_abc_f32_t *abc, svm_ab_f32_t *ab)
{
    if ((abc == 0) || (ab == 0)) return;
    ab->alpha = (2.0f / 3.0f) * (abc->a - 0.5f * abc->b - 0.5f * abc->c);
    ab->beta = (abc->b - abc->c) * SVM_INV_SQRT3;
}

void svm_inverse_clarke(const svm_ab_f32_t *ab, svm_abc_f32_t *abc)
{
    if ((ab == 0) || (abc == 0)) return;
    abc->a = ab->alpha;
    abc->b = -0.5f * ab->alpha + SVM_SQRT3_OVER_2 * ab->beta;
    abc->c = -0.5f * ab->alpha - SVM_SQRT3_OVER_2 * ab->beta;
}

void svm_park(const svm_ab_f32_t *ab, const svm_sincos_f32_t *sc, svm_dq_f32_t *dq)
{
    if ((ab == 0) || (sc == 0) || (dq == 0)) return;
    dq->d = sc->cos_theta * ab->alpha + sc->sin_theta * ab->beta;
    dq->q = -sc->sin_theta * ab->alpha + sc->cos_theta * ab->beta;
}

void svm_inverse_park(const svm_dq_f32_t *dq, const svm_sincos_f32_t *sc, svm_ab_f32_t *ab)
{
    if ((dq == 0) || (sc == 0) || (ab == 0)) return;
    ab->alpha = sc->cos_theta * dq->d - sc->sin_theta * dq->q;
    ab->beta = sc->sin_theta * dq->d + sc->cos_theta * dq->q;
}

void svm_nco_init(svm_nco_t *nco, float freq_hz, float sample_time_s)
{
    if (nco == 0) return;
    nco->angle_rad = 0.0f;
    svm_nco_set_frequency(nco, freq_hz, sample_time_s);
}

void svm_nco_set_frequency(svm_nco_t *nco, float freq_hz, float sample_time_s)
{
    if (nco == 0) return;
    nco->step_rad = SVM_TWO_PI * freq_hz * sample_time_s;
}

void svm_nco_set_angle(svm_nco_t *nco, float angle_rad)
{
    if (nco == 0) return;
    nco->angle_rad = svm_wrap_angle(angle_rad);
}

void svm_nco_step(svm_nco_t *nco)
{
    if (nco == 0) return;
    nco->angle_rad = svm_wrap_angle(nco->angle_rad + nco->step_rad);
}

void svm_lut_sincos(float angle_rad, svm_sincos_f32_t *sc)
{
    if (sc == 0) return;
#if SVM_USE_NS800_MMATH
    MMATH_sincosF32(angle_rad, &sc->sin_theta, &sc->cos_theta);
#else
    sc->sin_theta = svm_lut_sin(angle_rad);
    sc->cos_theta = svm_lut_sin(angle_rad + SVM_HALF_PI);
#endif
}
