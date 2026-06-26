/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SVM_POWER_ALLOC_H
#define SVM_POWER_ALLOC_H

#include "svm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float sample_time_s;
    float min_dc_voltage;
} svm_svm_config_t;

svm_svm_config_t svm_svm_default_config(void);
float svm_clamp_unit(float x);
void svm_power_split(const svm_ab_f32_t *u_ab, float power_factor, svm_split_ab_f32_t *split);
uint8_t svm_sector_from_ab(const svm_ab_f32_t *u_ab);
svm_status_t svm_compute_port(const svm_ab_f32_t *u_ab,
                              float dc_voltage,
                              svm_port_select_t port,
                              const svm_svm_config_t *cfg,
                              svm_svm_out_t *out);
svm_status_t svm_compute_dual(const svm_ab_f32_t *u_ab,
                              float power_factor,
                              float dc_upper_voltage,
                              float dc_lower_voltage,
                              const svm_svm_config_t *cfg,
                              svm_dual_out_t *out);
void svm_pwm_logic_preview(const svm_svm_out_t *upper,
                           const svm_svm_out_t *lower,
                           float carrier,
                           svm_pwm_logic_t *logic);

#ifdef __cplusplus
}
#endif

#endif /* SVM_POWER_ALLOC_H */
