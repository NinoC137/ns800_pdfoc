/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "svm_control.h"

#include "svm_config.h"
#include "svm_port.h"

void svm_control_default_config(svm_control_config_t *cfg)
{
    if (cfg == 0) return;
    cfg->power_factor = 0.5f;
    cfg->ud_ref = SVM_DEFAULT_UD_REF;
    cfg->uq_ref = SVM_DEFAULT_UQ_REF;
    cfg->sample_time_s = SVM_CONTROL_PERIOD_S;
    cfg->output_frequency_hz = SVM_DEFAULT_FREQ_HZ;
    cfg->voltage_cross_gain = SVM_DEFAULT_VOLTAGE_CROSS_GAIN;
    cfg->current_cross_gain = SVM_DEFAULT_CURRENT_CROSS_GAIN;
    cfg->outer_d = svm_pi_default_outer_voltage();
    cfg->outer_q = svm_pi_default_outer_voltage();
    cfg->inner_d = svm_pi_default_inner_current();
    cfg->inner_q = svm_pi_default_inner_current();
    cfg->svm = svm_svm_default_config();
}

void svm_control_init(svm_control_state_t *state, const svm_control_config_t *cfg)
{
    svm_control_config_t default_cfg;

    if (state == 0) return;
    if (cfg == 0) {
        svm_control_default_config(&default_cfg);
        cfg = &default_cfg;
    }
    svm_nco_init(&state->nco, cfg->output_frequency_hz, cfg->sample_time_s);
    svm_pi_init_state(&state->outer_d);
    svm_pi_init_state(&state->outer_q);
    svm_pi_init_state(&state->inner_d);
    svm_pi_init_state(&state->inner_q);
}

svm_status_t svm_control_step(svm_control_state_t *state,
                              const svm_control_config_t *cfg,
                              const svm_abc_f32_t *ac_voltage_abc,
                              const svm_abc_f32_t *ac_current_abc,
                              float dc_upper_voltage,
                              float dc_lower_voltage,
                              bool reset_pi,
                              svm_control_output_t *out)
{
    svm_control_config_t default_cfg;
    svm_sincos_f32_t sc;
    uint8_t status;

    if ((state == 0) || (ac_voltage_abc == 0) || (ac_current_abc == 0) || (out == 0)) {
        return SVM_STATUS_NULL_POINTER;
    }
    if (cfg == 0) {
        svm_control_default_config(&default_cfg);
        cfg = &default_cfg;
    }

    svm_sincos(state->nco.angle_rad, &sc);
    svm_clarke(ac_voltage_abc, &out->voltage_ab);
    svm_clarke(ac_current_abc, &out->current_ab);
    svm_park(&out->voltage_ab, &sc, &out->voltage_dq);
    svm_park(&out->current_ab, &sc, &out->current_dq);

    out->current_ref_dq.d = svm_pi_update_error(&state->outer_d, &cfg->outer_d, cfg->ud_ref, out->voltage_dq.d, reset_pi) -
                            cfg->voltage_cross_gain * out->voltage_dq.q;
    out->current_ref_dq.q = svm_pi_update_error(&state->outer_q, &cfg->outer_q, cfg->uq_ref, out->voltage_dq.q, reset_pi) +
                            cfg->voltage_cross_gain * out->voltage_dq.d;

    out->current_error_dq.d = out->current_ref_dq.d - out->current_dq.d;
    out->current_error_dq.q = out->current_ref_dq.q - out->current_dq.q;
    out->voltage_cmd_dq.d = svm_pi_update(&state->inner_d, &cfg->inner_d, out->current_error_dq.d, reset_pi) +
                            out->voltage_dq.d -
                            cfg->current_cross_gain * out->current_dq.q;
    out->voltage_cmd_dq.q = svm_pi_update(&state->inner_q, &cfg->inner_q, out->current_error_dq.q, reset_pi) +
                            out->voltage_dq.q +
                            cfg->current_cross_gain * out->current_dq.d;

    svm_inverse_park(&out->voltage_cmd_dq, &sc, &out->voltage_cmd_ab);
    status = (uint8_t)svm_compute_dual(&out->voltage_cmd_ab,
                                       cfg->power_factor,
                                       dc_upper_voltage,
                                       dc_lower_voltage,
                                       &cfg->svm,
                                       &out->duty);
    out->status = status;
    svm_nco_step(&state->nco);
    return (svm_status_t)status;
}

svm_status_t svm_control_isr_step(svm_control_state_t *state,
                                  const svm_control_config_t *cfg,
                                  svm_control_output_t *out)
{
    svm_adc_sample_t sample;
    svm_status_t status;

    if ((state == 0) || (out == 0)) return SVM_STATUS_NULL_POINTER;
    if (!svm_port_read_adc(&sample)) {
        svm_port_report_fault(SVM_STATUS_NULL_POINTER);
        return SVM_STATUS_NULL_POINTER;
    }

    status = svm_control_step(state,
                              cfg,
                              &sample.ac_voltage_abc,
                              &sample.ac_current_abc,
                              sample.dc_upper_voltage,
                              sample.dc_lower_voltage,
                              false,
                              out);

    svm_port_enter_critical();
    svm_port_write_pwm_compare(&out->duty);
    svm_port_exit_critical();

    if (status != SVM_STATUS_OK) {
        // svm_port_report_fault((uint32_t)status);
    }
    return status;
}
