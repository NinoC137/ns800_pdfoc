/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SVM_CONTROL_H
#define SVM_CONTROL_H

#include <stdbool.h>
#include "svm_pi.h"
#include "svm_power_alloc.h"
#include "svm_transform.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float power_factor;
    float ud_ref;
    float uq_ref;
    float sample_time_s;
    float output_frequency_hz;
    float voltage_cross_gain;
    float current_cross_gain;
    svm_pi_config_t outer_d;
    svm_pi_config_t outer_q;
    svm_pi_config_t inner_d;
    svm_pi_config_t inner_q;
    svm_svm_config_t svm;
} svm_control_config_t;

typedef struct {
    svm_nco_t nco;
    svm_pi_state_t outer_d;
    svm_pi_state_t outer_q;
    svm_pi_state_t inner_d;
    svm_pi_state_t inner_q;
} svm_control_state_t;

typedef struct {
    svm_ab_f32_t voltage_ab;
    svm_ab_f32_t current_ab;
    svm_dq_f32_t voltage_dq;
    svm_dq_f32_t current_dq;
    svm_dq_f32_t current_ref_dq;
    svm_dq_f32_t current_error_dq;
    svm_dq_f32_t voltage_cmd_dq;
    svm_ab_f32_t voltage_cmd_ab;
    svm_dual_out_t duty;
    uint8_t status;
} svm_control_output_t;

void svm_control_default_config(svm_control_config_t *cfg);
void svm_control_init(svm_control_state_t *state, const svm_control_config_t *cfg);
svm_status_t svm_control_step(svm_control_state_t *state,
                              const svm_control_config_t *cfg,
                              const svm_abc_f32_t *ac_voltage_abc,
                              const svm_abc_f32_t *ac_current_abc,
                              float dc_upper_voltage,
                              float dc_lower_voltage,
                              bool reset_pi,
                              svm_control_output_t *out);
svm_status_t svm_control_isr_step(svm_control_state_t *state,
                                  const svm_control_config_t *cfg,
                                  svm_control_output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* SVM_CONTROL_H */
