/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "svm_pi.h"

#include "svm_config.h"

static float svm_pi_clamp(float x, float min_value, float max_value)
{
    if (x > max_value) return max_value;
    if (x < min_value) return min_value;
    return x;
}

void svm_pi_init_state(svm_pi_state_t *state)
{
    if (state == 0) return;
    state->integrator = 0.0f;
    state->previous_error = 0.0f;
}

float svm_pi_update(svm_pi_state_t *state, const svm_pi_config_t *cfg, float error, bool reset)
{
    float candidate_integrator;
    float output;
    float saturated;
    bool allow_integrator_update = true;

    if ((state == 0) || (cfg == 0)) return 0.0f;
    if (reset) svm_pi_init_state(state);

    candidate_integrator = state->integrator + cfg->ki * cfg->sample_time_s * error;
    if (cfg->limit_integrator) {
        candidate_integrator = svm_pi_clamp(candidate_integrator, cfg->integrator_min, cfg->integrator_max);
    }

    output = cfg->kp * error + candidate_integrator;
    saturated = output;

    if (cfg->limit_output) {
        saturated = svm_pi_clamp(output, cfg->out_min, cfg->out_max);
        if (cfg->anti_windup && (saturated != output)) {
            if ((output > cfg->out_max) && (error > 0.0f)) allow_integrator_update = false;
            if ((output < cfg->out_min) && (error < 0.0f)) allow_integrator_update = false;
        }
    }

    if (allow_integrator_update) state->integrator = candidate_integrator;
    state->previous_error = error;

    if (!allow_integrator_update) {
        output = cfg->kp * error + state->integrator;
        saturated = cfg->limit_output ? svm_pi_clamp(output, cfg->out_min, cfg->out_max) : output;
    }

    return saturated;
}

float svm_pi_update_error(svm_pi_state_t *state,
                          const svm_pi_config_t *cfg,
                          float reference,
                          float feedback,
                          bool reset)
{
    return svm_pi_update(state, cfg, reference - feedback, reset);
}

svm_pi_config_t svm_pi_default_outer_voltage(void)
{
    svm_pi_config_t cfg;
    cfg.kp = SVM_DEFAULT_OUTER_VOLTAGE_KP;
    cfg.ki = SVM_DEFAULT_OUTER_VOLTAGE_KI;
    cfg.sample_time_s = SVM_CONTROL_PERIOD_S;
    cfg.out_min = -700.0f;
    cfg.out_max = 700.0f;
    cfg.integrator_min = -700.0f;
    cfg.integrator_max = 700.0f;
    cfg.limit_output = false;
    cfg.limit_integrator = false;
    cfg.anti_windup = false;
    return cfg;
}

svm_pi_config_t svm_pi_default_inner_current(void)
{
    svm_pi_config_t cfg;
    cfg.kp = SVM_DEFAULT_INNER_CURRENT_KP;
    cfg.ki = SVM_DEFAULT_INNER_CURRENT_KI;
    cfg.sample_time_s = SVM_CONTROL_PERIOD_S;
    cfg.out_min = -700.0f;
    cfg.out_max = 700.0f;
    cfg.integrator_min = -700.0f;
    cfg.integrator_max = 700.0f;
    cfg.limit_output = false;
    cfg.limit_integrator = false;
    cfg.anti_windup = false;
    return cfg;
}
