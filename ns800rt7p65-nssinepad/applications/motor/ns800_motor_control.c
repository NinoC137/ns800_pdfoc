/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_motor_control.h"

#define NS800_MOTOR_DEFAULT_POLE_PAIRS            4u
#define NS800_MOTOR_DEFAULT_TORQUE_CONSTANT_NM_A  (0.10f)
#define NS800_MOTOR_DEFAULT_FLUX_WB               (0.02f)
#define NS800_MOTOR_DEFAULT_LD_H                  (1.0e-3f)
#define NS800_MOTOR_DEFAULT_LQ_H                  (1.0e-3f)
#define NS800_MOTOR_DEFAULT_CURRENT_KP            (2.0f)
#define NS800_MOTOR_DEFAULT_CURRENT_KI            (800.0f)
#define NS800_MOTOR_DEFAULT_SPEED_KP              (0.02f)
#define NS800_MOTOR_DEFAULT_SPEED_KI              (2.0f)
#define NS800_MOTOR_DEFAULT_MAX_CURRENT_A         (20.0f)
#define NS800_MOTOR_DEFAULT_MAX_TORQUE_NM         (2.0f)
#define NS800_MOTOR_DEFAULT_MAX_SPEED_RAD_S       (600.0f)
#define NS800_MOTOR_DEFAULT_MAX_VOLTAGE_V         (12.0f)
#define NS800_MOTOR_RPM_TO_RAD_S                  (0.1047197551f)

/**
 * @brief 计算 float 绝对值，避免在 ISR 控制路径中引入额外数学库依赖。
 *
 * @param value 输入值。
 * @return 输入值的非负绝对值。
 */
static float motor_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 将输入值裁剪到闭区间。
 *
 * @param value 输入值。
 * @param min_value 下限。
 * @param max_value 上限。
 * @return 裁剪后的值。
 */
static float motor_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/**
 * @brief 将限幅参数规整为非负值。
 *
 * @param value 原始限幅值。
 * @return 非负限幅值。
 */
static float motor_positive_limit(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 生成带输出限幅、积分限幅和抗饱和的 PI 配置。
 *
 * @param kp 比例系数。
 * @param ki 积分系数。
 * @param sample_time_s 控制周期，单位 s。
 * @param limit 对称输出/积分限幅。
 * @return PI 配置。
 */
static svm_pi_config_t motor_make_pi(float kp, float ki, float sample_time_s, float limit)
{
    svm_pi_config_t pi;

    limit = motor_positive_limit(limit);
    pi.kp = kp;
    pi.ki = ki;
    pi.sample_time_s = sample_time_s;
    pi.out_min = -limit;
    pi.out_max = limit;
    pi.integrator_min = -limit;
    pi.integrator_max = limit;
    pi.limit_output = true;
    pi.limit_integrator = true;
    pi.anti_windup = true;

    return pi;
}

/**
 * @brief 检查电机参数是否足以执行 FOC 计算。
 *
 * @param params 电机参数。
 * @return true 参数有效；false 参数非法。
 */
static bool motor_params_valid(const ns800_motor_params_t *params)
{
    if (params == 0)
    {
        return false;
    }
    if (params->pole_pairs == 0u)
    {
        return false;
    }
    if (motor_abs(params->torque_constant_nm_a) <= 1.0e-6f)
    {
        return false;
    }
    if ((params->ld_h < 0.0f) || (params->lq_h < 0.0f))
    {
        return false;
    }

    return true;
}

/**
 * @brief 将 SVM 状态位映射到电机控制状态位。
 *
 * @param status 已累计的电机状态位。
 * @param svm_status 本拍 SVM 返回状态。
 * @return 合并后的电机状态位。
 */
static uint32_t motor_status_from_svm(uint32_t status, svm_status_t svm_status)
{
    if (((uint8_t)svm_status & (uint8_t)SVM_STATUS_LOW_DC_VOLTAGE) != 0u)
    {
        status |= (uint32_t)NS800_MOTOR_STATUS_LOW_DC_VOLTAGE;
    }
    if (((uint8_t)svm_status & (uint8_t)SVM_STATUS_OVERMODULATION) != 0u)
    {
        status |= (uint32_t)NS800_MOTOR_STATUS_OVERMODULATION;
    }
    if (((uint8_t)svm_status & (uint8_t)SVM_STATUS_CLAMPED) != 0u)
    {
        status |= (uint32_t)NS800_MOTOR_STATUS_CLAMPED;
    }
    if (((uint8_t)svm_status & (uint8_t)SVM_STATUS_NULL_POINTER) != 0u)
    {
        status |= (uint32_t)NS800_MOTOR_STATUS_NULL_POINTER;
    }

    return status;
}

/**
 * @brief 清零单拍输出结构。
 *
 * @param out 输出结构指针；为空时直接返回。
 */
static void motor_clear_output(ns800_motor_output_t *out)
{
    if (out == 0)
    {
        return;
    }

    out->mode = NS800_MOTOR_MODE_STOP;
    out->xi = 0.0f;
    out->theta_e_rad = 0.0f;
    out->omega_m_rad_s = 0.0f;
    out->speed_ref_rad_s = 0.0f;
    out->iq_limit_a = 0.0f;
    out->current_ab.alpha = 0.0f;
    out->current_ab.beta = 0.0f;
    out->current_dq.d = 0.0f;
    out->current_dq.q = 0.0f;
    out->current_ref_dq.d = 0.0f;
    out->current_ref_dq.q = 0.0f;
    out->current_error_dq.d = 0.0f;
    out->current_error_dq.q = 0.0f;
    out->voltage_cmd_dq.d = 0.0f;
    out->voltage_cmd_dq.q = 0.0f;
    out->voltage_cmd_ab.alpha = 0.0f;
    out->voltage_cmd_ab.beta = 0.0f;
    out->voltage_cmd_abc.a = 0.0f;
    out->voltage_cmd_abc.b = 0.0f;
    out->voltage_cmd_abc.c = 0.0f;
    out->duty.upper.ta = 0.0f;
    out->duty.upper.tb = 0.0f;
    out->duty.upper.tc = 0.0f;
    out->duty.upper.sector = 0u;
    out->duty.upper.status = 0u;
    out->duty.lower.ta = 0.0f;
    out->duty.lower.tb = 0.0f;
    out->duty.lower.tc = 0.0f;
    out->duty.lower.sector = 0u;
    out->duty.lower.status = 0u;
    out->status = (uint32_t)NS800_MOTOR_STATUS_OK;
}

/**
 * @brief 将 q 轴电流转换为电磁转矩。
 *
 * @param params 电机参数。
 * @param iq_a q 轴电流，单位 A。
 * @return 转矩，单位 Nm。
 */
static float motor_iq_to_torque(const ns800_motor_params_t *params, float iq_a)
{
    return params->torque_constant_nm_a * iq_a;
}

/**
 * @brief 将转矩参考转换为 q 轴电流参考。
 *
 * @param params 电机参数。
 * @param torque_nm 转矩参考，单位 Nm。
 * @return q 轴电流参考，单位 A。
 */
static float motor_torque_to_iq(const ns800_motor_params_t *params, float torque_nm)
{
    return torque_nm / params->torque_constant_nm_a;
}

/**
 * @brief 填充默认电机参数。
 *
 * 默认值仅用于 bring-up 和软件链路验证，实际闭环前应根据电机实测参数更新。
 *
 * @param params 输出参数指针。
 */
void ns800_motor_default_params(ns800_motor_params_t *params)
{
    if (params == 0)
    {
        return;
    }

    params->pole_pairs = NS800_MOTOR_DEFAULT_POLE_PAIRS;
    params->torque_constant_nm_a = NS800_MOTOR_DEFAULT_TORQUE_CONSTANT_NM_A;
    params->flux_wb = NS800_MOTOR_DEFAULT_FLUX_WB;
    params->ld_h = NS800_MOTOR_DEFAULT_LD_H;
    params->lq_h = NS800_MOTOR_DEFAULT_LQ_H;
    params->encoder_offset_rad = 0.0f;
}

/**
 * @brief 填充默认电机控制配置。
 *
 * 默认上电开环输出为 12 V peak、50 Hz，双端口母线电压固定为 48 V/24 V。
 *
 * @param cfg 输出配置指针。
 */
void ns800_motor_default_config(ns800_motor_config_t *cfg)
{
    if (cfg == 0)
    {
        return;
    }

    cfg->sample_time_s = NS800_MOTOR_CONTROL_PERIOD_S;
    cfg->open_loop_freq_hz = 50.0f;
    cfg->open_loop_voltage_v = 12.0f;
    cfg->dc_upper_voltage_v = 48.0f;
    cfg->dc_lower_voltage_v = 24.0f;
    cfg->id_ref_a = 0.0f;
    cfg->max_current_a = NS800_MOTOR_DEFAULT_MAX_CURRENT_A;
    cfg->max_torque_nm = NS800_MOTOR_DEFAULT_MAX_TORQUE_NM;
    cfg->max_speed_rad_s = NS800_MOTOR_DEFAULT_MAX_SPEED_RAD_S;
    cfg->max_voltage_v = NS800_MOTOR_DEFAULT_MAX_VOLTAGE_V;
    cfg->current_d = motor_make_pi(NS800_MOTOR_DEFAULT_CURRENT_KP,
                                   NS800_MOTOR_DEFAULT_CURRENT_KI,
                                   cfg->sample_time_s,
                                   cfg->max_voltage_v);
    cfg->current_q = motor_make_pi(NS800_MOTOR_DEFAULT_CURRENT_KP,
                                   NS800_MOTOR_DEFAULT_CURRENT_KI,
                                   cfg->sample_time_s,
                                   cfg->max_voltage_v);
    cfg->speed = motor_make_pi(NS800_MOTOR_DEFAULT_SPEED_KP,
                               NS800_MOTOR_DEFAULT_SPEED_KI,
                               cfg->sample_time_s,
                               cfg->max_torque_nm);
    cfg->svm = svm_svm_default_config();
    cfg->svm.sample_time_s = cfg->sample_time_s;
    cfg->svm.min_dc_voltage = 1.0f;
}

/**
 * @brief 初始化电机控制状态。
 *
 * 初始化开环 NCO，并清零速度环和电流环 PI 状态。
 *
 * @param state 控制状态。
 * @param cfg 控制配置；为空时使用默认配置。
 */
void ns800_motor_init(ns800_motor_state_t *state, const ns800_motor_config_t *cfg)
{
    ns800_motor_config_t default_cfg;

    if (state == 0)
    {
        return;
    }
    if (cfg == 0)
    {
        ns800_motor_default_config(&default_cfg);
        cfg = &default_cfg;
    }

    svm_nco_init(&state->open_loop_nco, cfg->open_loop_freq_hz, cfg->sample_time_s);
    ns800_motor_reset(state);
}

/**
 * @brief 清零速度环和电流环 PI 状态。
 *
 * @param state 控制状态。
 */
void ns800_motor_reset(ns800_motor_state_t *state)
{
    if (state == 0)
    {
        return;
    }

    svm_pi_init_state(&state->current_d);
    svm_pi_init_state(&state->current_q);
    svm_pi_init_state(&state->speed);
}

/**
 * @brief 将角度折返到 [0, 2*pi)。
 *
 * @param angle_rad 输入角度，单位 rad。
 * @return 折返后的角度。
 */
float ns800_motor_wrap_angle_0_2pi(float angle_rad)
{
    while (angle_rad >= NS800_MOTOR_TWO_PI)
    {
        angle_rad -= NS800_MOTOR_TWO_PI;
    }
    while (angle_rad < 0.0f)
    {
        angle_rad += NS800_MOTOR_TWO_PI;
    }
    return angle_rad;
}

/**
 * @brief 由机械角计算电角度。
 *
 * @param params 电机参数。
 * @param theta_m_rad 机械角，单位 rad。
 * @return 电角度，单位 rad。
 */
float ns800_motor_mech_to_elec_angle(const ns800_motor_params_t *params, float theta_m_rad)
{
    if ((params == 0) || (params->pole_pairs == 0u))
    {
        return 0.0f;
    }

    return ns800_motor_wrap_angle_0_2pi(((float)params->pole_pairs * theta_m_rad) +
                                        params->encoder_offset_rad);
}

/**
 * @brief 执行一拍功率分配无刷电机控制。
 *
 * - STOP：保持输出清零。
 * - OPEN_LOOP：用 NCO 生成旋转电压矢量，直接进入双端口 SVM。
 * - TORQUE/SPEED：需要 ADC 电流和转子反馈，执行 Clarke/Park、速度 PI、
 *   电流 PI、逆 Park 与双端口 SVM。
 *
 * @param state 控制状态，会被本函数更新。
 * @param cfg 控制配置；为空时使用默认配置。
 * @param params 电机参数；为空时使用默认参数。
 * @param in 单拍输入。
 * @param out 单拍输出。
 * @return 状态位图。
 */
ns800_motor_status_t ns800_motor_step(ns800_motor_state_t *state,
                                      const ns800_motor_config_t *cfg,
                                      const ns800_motor_params_t *params,
                                      const ns800_motor_input_t *in,
                                      ns800_motor_output_t *out)
{
    ns800_motor_config_t default_cfg;
    ns800_motor_params_t default_params;
    svm_sincos_f32_t sc;
    svm_status_t svm_status;
    uint32_t status = (uint32_t)NS800_MOTOR_STATUS_OK;
    float xi;
    float current_limit;
    float iq_limit;
    float voltage_limit;
    float torque_limit;
    float torque_cmd_nm;
    float vd_ff;
    float vq_ff;

    if ((state == 0) || (in == 0) || (out == 0))
    {
        return NS800_MOTOR_STATUS_NULL_POINTER;
    }
    if (cfg == 0)
    {
        ns800_motor_default_config(&default_cfg);
        cfg = &default_cfg;
    }
    if (params == 0)
    {
        ns800_motor_default_params(&default_params);
        params = &default_params;
    }

    motor_clear_output(out);
    out->mode = in->command.mode;

    if (!motor_params_valid(params))
    {
        out->status = (uint32_t)NS800_MOTOR_STATUS_INVALID_PARAM;
        return NS800_MOTOR_STATUS_INVALID_PARAM;
    }
    if (in->command.reset)
    {
        ns800_motor_reset(state);
    }

    xi = motor_clamp(in->command.xi, 0.0f, 1.0f);
    current_limit = motor_positive_limit(cfg->max_current_a);
    iq_limit = motor_clamp(motor_positive_limit(in->command.iq_limit_a), 0.0f, current_limit);
    voltage_limit = motor_positive_limit(cfg->max_voltage_v);
    torque_limit = motor_positive_limit(cfg->max_torque_nm);
    out->xi = xi;
    out->iq_limit_a = iq_limit;

    if (in->command.mode == NS800_MOTOR_MODE_STOP)
    {
        return NS800_MOTOR_STATUS_OK;
    }

    if (in->command.mode == NS800_MOTOR_MODE_OPEN_LOOP)
    {
        out->theta_e_rad = state->open_loop_nco.angle_rad;
        svm_sincos(out->theta_e_rad, &sc);
        out->voltage_cmd_ab.alpha = cfg->open_loop_voltage_v * sc.cos_theta;
        out->voltage_cmd_ab.beta = cfg->open_loop_voltage_v * sc.sin_theta;
        svm_inverse_clarke(&out->voltage_cmd_ab, &out->voltage_cmd_abc);
        svm_status = svm_compute_dual(&out->voltage_cmd_ab,
                                      xi,
                                      cfg->dc_upper_voltage_v,
                                      cfg->dc_lower_voltage_v,
                                      &cfg->svm,
                                      &out->duty);
        status = motor_status_from_svm(status, svm_status);
        out->status = status;
        svm_nco_step(&state->open_loop_nco);
        return (ns800_motor_status_t)status;
    }

    if (!in->sample.adc_valid)
    {
        status |= (uint32_t)NS800_MOTOR_STATUS_SAMPLE_FAULT;
    }
    if (!in->sample.feedback_valid)
    {
        out->status = status | (uint32_t)NS800_MOTOR_STATUS_FEEDBACK_FAULT;
        return (ns800_motor_status_t)out->status;
    }

    out->theta_e_rad = ns800_motor_mech_to_elec_angle(params, in->sample.theta_m_rad);
    out->omega_m_rad_s = in->sample.omega_m_rad_s;
    svm_sincos(out->theta_e_rad, &sc);
    svm_clarke(&in->sample.phase_current_abc, &out->current_ab);
    svm_park(&out->current_ab, &sc, &out->current_dq);

    out->current_ref_dq.d = motor_clamp(cfg->id_ref_a, -current_limit, current_limit);
    out->speed_ref_rad_s = motor_clamp(in->command.speed_ref_rpm * NS800_MOTOR_RPM_TO_RAD_S,
                                       -cfg->max_speed_rad_s,
                                       cfg->max_speed_rad_s);

    if (in->command.mode == NS800_MOTOR_MODE_SPEED)
    {
        torque_cmd_nm = svm_pi_update_error(&state->speed,
                                            &cfg->speed,
                                            out->speed_ref_rad_s,
                                            in->sample.omega_m_rad_s,
                                            in->command.reset);
        torque_cmd_nm = motor_clamp(torque_cmd_nm, -torque_limit, torque_limit);
        out->current_ref_dq.q = motor_torque_to_iq(params, torque_cmd_nm);
    }
    else
    {
        out->current_ref_dq.q = in->command.iq_ref_a;
    }

    out->current_ref_dq.q = motor_clamp(out->current_ref_dq.q, -iq_limit, iq_limit);
    out->current_error_dq.d = out->current_ref_dq.d - out->current_dq.d;
    out->current_error_dq.q = out->current_ref_dq.q - out->current_dq.q;

    vd_ff = -((float)params->pole_pairs * out->omega_m_rad_s) * params->lq_h * out->current_dq.q;
    vq_ff = ((float)params->pole_pairs * out->omega_m_rad_s) *
            ((params->ld_h * out->current_dq.d) + params->flux_wb);

    out->voltage_cmd_dq.d = svm_pi_update(&state->current_d,
                                          &cfg->current_d,
                                          out->current_error_dq.d,
                                          in->command.reset) +
                            vd_ff;
    out->voltage_cmd_dq.q = svm_pi_update(&state->current_q,
                                          &cfg->current_q,
                                          out->current_error_dq.q,
                                          in->command.reset) +
                            vq_ff;
    out->voltage_cmd_dq.d = motor_clamp(out->voltage_cmd_dq.d, -voltage_limit, voltage_limit);
    out->voltage_cmd_dq.q = motor_clamp(out->voltage_cmd_dq.q, -voltage_limit, voltage_limit);

    svm_inverse_park(&out->voltage_cmd_dq, &sc, &out->voltage_cmd_ab);
    svm_inverse_clarke(&out->voltage_cmd_ab, &out->voltage_cmd_abc);
    svm_status = svm_compute_dual(&out->voltage_cmd_ab,
                                  xi,
                                  cfg->dc_upper_voltage_v,
                                  cfg->dc_lower_voltage_v,
                                  &cfg->svm,
                                  &out->duty);
    status = motor_status_from_svm(status, svm_status);
    out->status = status;

    (void)motor_iq_to_torque(params, out->current_ref_dq.q);
    return (ns800_motor_status_t)status;
}
