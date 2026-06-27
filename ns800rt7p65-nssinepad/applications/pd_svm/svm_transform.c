/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "svm_transform.h"

#include "emath.h"
#include "svm_config.h"

static float svm_wrap_angle(float angle_rad)
{
    while (angle_rad >= SVM_TWO_PI) angle_rad -= SVM_TWO_PI;
    while (angle_rad < 0.0f) angle_rad += SVM_TWO_PI;
    return angle_rad;
}

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

void svm_sincos(float angle_rad, svm_sincos_f32_t *sc)
{
    if (sc == 0) return;
    EMATH_sincosF32(angle_rad, &sc->sin_theta, &sc->cos_theta);
}
