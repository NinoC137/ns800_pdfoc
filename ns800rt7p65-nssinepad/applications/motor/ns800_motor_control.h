/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_MOTOR_CONTROL_H__
#define __NS800_MOTOR_CONTROL_H__

#include <stdbool.h>
#include <stdint.h>

#include "svm_pi.h"
#include "svm_power_alloc.h"
#include "svm_transform.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NS800_MOTOR_CONTROL_PERIOD_S       (1.0e-4f)
#define NS800_MOTOR_PI                     (3.1415926535897932385f)
#define NS800_MOTOR_TWO_PI                 (6.2831853071795864769f)

typedef enum
{
    NS800_MOTOR_MODE_STOP = 0,
    NS800_MOTOR_MODE_OPEN_LOOP,
    NS800_MOTOR_MODE_TORQUE,
    NS800_MOTOR_MODE_SPEED,
} ns800_motor_mode_t;

typedef enum
{
    NS800_MOTOR_STATUS_OK = 0,
    NS800_MOTOR_STATUS_NULL_POINTER = 1u << 0,
    NS800_MOTOR_STATUS_INVALID_PARAM = 1u << 1,
    NS800_MOTOR_STATUS_FEEDBACK_FAULT = 1u << 2,
    NS800_MOTOR_STATUS_SAMPLE_FAULT = 1u << 3,
    NS800_MOTOR_STATUS_LOW_DC_VOLTAGE = 1u << 4,
    NS800_MOTOR_STATUS_OVERMODULATION = 1u << 5,
    NS800_MOTOR_STATUS_CLAMPED = 1u << 6,
} ns800_motor_status_t;

typedef struct
{
    uint16_t pole_pairs;
    float torque_constant_nm_a;
    float flux_wb;
    float ld_h;
    float lq_h;
    float encoder_offset_rad;
} ns800_motor_params_t;

typedef struct
{
    float sample_time_s;
    float open_loop_freq_hz;
    float open_loop_voltage_v;
    float dc_upper_voltage_v;
    float dc_lower_voltage_v;
    float id_ref_a;
    float max_current_a;
    float max_torque_nm;
    float max_speed_rad_s;
    float max_voltage_v;
    svm_pi_config_t current_d;
    svm_pi_config_t current_q;
    svm_pi_config_t speed;
    svm_svm_config_t svm;
} ns800_motor_config_t;

typedef struct
{
    svm_nco_t open_loop_nco;
    svm_pi_state_t current_d;
    svm_pi_state_t current_q;
    svm_pi_state_t speed;
} ns800_motor_state_t;

typedef struct
{
    ns800_motor_mode_t mode;
    float xi;
    float speed_ref_rpm;
    float iq_limit_a;
    float iq_ref_a;
    bool reset;
} ns800_motor_command_t;

typedef struct
{
    svm_abc_f32_t phase_current_abc;
    float theta_m_rad;
    float omega_m_rad_s;
    bool adc_valid;
    bool feedback_valid;
} ns800_motor_sample_t;

typedef struct
{
    ns800_motor_sample_t sample;
    ns800_motor_command_t command;
} ns800_motor_input_t;

typedef struct
{
    ns800_motor_mode_t mode;
    float xi;
    float theta_e_rad;
    float omega_m_rad_s;
    float speed_ref_rad_s;
    float iq_limit_a;
    svm_ab_f32_t current_ab;
    svm_dq_f32_t current_dq;
    svm_dq_f32_t current_ref_dq;
    svm_dq_f32_t current_error_dq;
    svm_dq_f32_t voltage_cmd_dq;
    svm_ab_f32_t voltage_cmd_ab;
    svm_abc_f32_t voltage_cmd_abc;
    svm_dual_out_t duty;
    uint32_t status;
} ns800_motor_output_t;

void ns800_motor_default_params(ns800_motor_params_t *params);
void ns800_motor_default_config(ns800_motor_config_t *cfg);
void ns800_motor_init(ns800_motor_state_t *state, const ns800_motor_config_t *cfg);
void ns800_motor_reset(ns800_motor_state_t *state);
float ns800_motor_wrap_angle_0_2pi(float angle_rad);
float ns800_motor_mech_to_elec_angle(const ns800_motor_params_t *params, float theta_m_rad);
ns800_motor_status_t ns800_motor_step(ns800_motor_state_t *state,
                                      const ns800_motor_config_t *cfg,
                                      const ns800_motor_params_t *params,
                                      const ns800_motor_input_t *in,
                                      ns800_motor_output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_MOTOR_CONTROL_H__ */
