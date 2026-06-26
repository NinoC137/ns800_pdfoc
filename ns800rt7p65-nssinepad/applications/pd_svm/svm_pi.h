/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SVM_PI_H
#define SVM_PI_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float sample_time_s;
    float out_min;
    float out_max;
    float integrator_min;
    float integrator_max;
    bool limit_output;
    bool limit_integrator;
    bool anti_windup;
} svm_pi_config_t;

typedef struct {
    float integrator;
    float previous_error;
} svm_pi_state_t;

void svm_pi_init_state(svm_pi_state_t *state);
float svm_pi_update(svm_pi_state_t *state, const svm_pi_config_t *cfg, float error, bool reset);
float svm_pi_update_error(svm_pi_state_t *state,
                          const svm_pi_config_t *cfg,
                          float reference,
                          float feedback,
                          bool reset);
svm_pi_config_t svm_pi_default_outer_voltage(void);
svm_pi_config_t svm_pi_default_inner_current(void);

#ifdef __cplusplus
}
#endif

#endif /* SVM_PI_H */
