/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SVM_TRANSFORM_H
#define SVM_TRANSFORM_H

#include "svm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float angle_rad;
    float step_rad;
} svm_nco_t;

void svm_clarke(const svm_abc_f32_t *abc, svm_ab_f32_t *ab);
void svm_inverse_clarke(const svm_ab_f32_t *ab, svm_abc_f32_t *abc);
void svm_park(const svm_ab_f32_t *ab, const svm_sincos_f32_t *sc, svm_dq_f32_t *dq);
void svm_inverse_park(const svm_dq_f32_t *dq, const svm_sincos_f32_t *sc, svm_ab_f32_t *ab);
void svm_nco_init(svm_nco_t *nco, float freq_hz, float sample_time_s);
void svm_nco_set_frequency(svm_nco_t *nco, float freq_hz, float sample_time_s);
void svm_nco_set_angle(svm_nco_t *nco, float angle_rad);
void svm_nco_step(svm_nco_t *nco);
void svm_sincos(float angle_rad, svm_sincos_f32_t *sc);

#ifdef __cplusplus
}
#endif

#endif /* SVM_TRANSFORM_H */
