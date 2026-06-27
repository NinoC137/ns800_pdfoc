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

/**
 * @brief 电机控制运行模式。
 *
 * OPEN_LOOP 用于无传感器的开环 SVM/PWM 链路验证；TORQUE 与 SPEED
 * 需要有效的转子角度/速度反馈，当前由平台层反馈接口提供。
 */
typedef enum
{
    /**< 停止输出，控制器不更新有效占空比。 */
    NS800_MOTOR_MODE_STOP = 0,
    /**< 开环旋转电压矢量输出，默认 12 V peak、50 Hz。 */
    NS800_MOTOR_MODE_OPEN_LOOP,
    /**< q 轴电流给定模式，作为力矩开环/电流闭环调试入口。 */
    NS800_MOTOR_MODE_TORQUE,
    /**< 速度-电流串级 PID 闭环模式。 */
    NS800_MOTOR_MODE_SPEED,
} ns800_motor_mode_t;

/**
 * @brief 电机控制状态位。
 *
 * 状态采用位图形式返回，便于同时表达采样、反馈与 SVM 调制异常。
 */
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

/**
 * @brief 无刷电机本体参数。
 *
 * 首版使用占位默认参数，后续应按实际电机铭牌或辨识结果更新。
 */
typedef struct
{
    /**< 电机极对数。 */
    uint16_t pole_pairs;
    /**< 转矩常数，单位 Nm/A。 */
    float torque_constant_nm_a;
    /**< 永磁体磁链，单位 Wb，用于 q 轴前馈。 */
    float flux_wb;
    /**< d 轴电感，单位 H。 */
    float ld_h;
    /**< q 轴电感，单位 H。 */
    float lq_h;
    /**< 编码器/反馈零位偏置，机械角到电角度转换时叠加。 */
    float encoder_offset_rad;
} ns800_motor_params_t;

/**
 * @brief 电机控制器配置。
 *
 * 包含控制周期、开环测试参数、固定母线电压、PI 参数和功率分配 SVM 配置。
 */
typedef struct
{
    /**< 控制周期，单位 s；当前 EPWM2 ISR 为 10 kHz。 */
    float sample_time_s;
    /**< 开环电角频率，单位 Hz。 */
    float open_loop_freq_hz;
    /**< 开环 alpha/beta 电压矢量峰值，单位 V。 */
    float open_loop_voltage_v;
    /**< upper 侧固定母线电压，单位 V。 */
    float dc_upper_voltage_v;
    /**< lower 侧固定母线电压，单位 V。 */
    float dc_lower_voltage_v;
    /**< d 轴电流参考，默认 0 A。 */
    float id_ref_a;
    /**< 电流绝对限幅，单位 A。 */
    float max_current_a;
    /**< 速度环输出力矩限幅，单位 Nm。 */
    float max_torque_nm;
    /**< 机械速度参考限幅，单位 rad/s。 */
    float max_speed_rad_s;
    /**< 电流环输出 dq 电压限幅，单位 V。 */
    float max_voltage_v;
    /**< d 轴电流 PI 配置。 */
    svm_pi_config_t current_d;
    /**< q 轴电流 PI 配置。 */
    svm_pi_config_t current_q;
    /**< 速度 PI 配置，输出力矩参考。 */
    svm_pi_config_t speed;
    /**< 双端口功率分配 SVM 配置。 */
    svm_svm_config_t svm;
} ns800_motor_config_t;

/**
 * @brief 电机控制器运行状态。
 *
 * 该结构由控制 ISR 持有并更新，不应在中断外直接修改。
 */
typedef struct
{
    /**< 开环模式使用的电角度 NCO。 */
    svm_nco_t open_loop_nco;
    /**< d 轴电流 PI 状态。 */
    svm_pi_state_t current_d;
    /**< q 轴电流 PI 状态。 */
    svm_pi_state_t current_q;
    /**< 速度 PI 状态。 */
    svm_pi_state_t speed;
} ns800_motor_state_t;

/**
 * @brief 单拍控制命令。
 *
 * 平台层从按键全局变量生成该命令：xi 为功率分配系数，speed_ref_rpm
 * 为目标机械转速，iq_limit_a 由 F(mA) 转换得到。
 */
typedef struct
{
    ns800_motor_mode_t mode;
    float xi;
    float speed_ref_rpm;
    float iq_limit_a;
    float iq_ref_a;
    bool reset;
} ns800_motor_command_t;

/**
 * @brief 单拍控制采样。
 *
 * phase_current_abc 来自 ADC DMA；theta/omega 来自外部转子反馈接口。
 */
typedef struct
{
    svm_abc_f32_t phase_current_abc;
    float theta_m_rad;
    float omega_m_rad_s;
    bool adc_valid;
    bool feedback_valid;
} ns800_motor_sample_t;

/**
 * @brief 单拍电机控制输入。
 */
typedef struct
{
    ns800_motor_sample_t sample;
    ns800_motor_command_t command;
} ns800_motor_input_t;

/**
 * @brief 单拍电机控制输出和诊断量。
 *
 * duty 直接映射到 EPWM8~13：A 相 EPWM8/9，B 相 EPWM10/11，
 * C 相 EPWM12/13。
 */
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

/**
 * @brief 填充默认电机参数。
 *
 * @param params 输出参数指针；为空时直接返回。
 */
void ns800_motor_default_params(ns800_motor_params_t *params);

/**
 * @brief 填充默认控制器配置。
 *
 * @param cfg 输出配置指针；为空时直接返回。
 */
void ns800_motor_default_config(ns800_motor_config_t *cfg);

/**
 * @brief 初始化控制器状态。
 *
 * @param state 控制器状态指针。
 * @param cfg 控制器配置；为空时使用默认配置。
 */
void ns800_motor_init(ns800_motor_state_t *state, const ns800_motor_config_t *cfg);

/**
 * @brief 清零所有 PI 状态。
 *
 * @param state 控制器状态指针。
 */
void ns800_motor_reset(ns800_motor_state_t *state);

/**
 * @brief 将角度折返到 [0, 2*pi)。
 *
 * @param angle_rad 输入角度，单位 rad。
 * @return 折返后的角度，单位 rad。
 */
float ns800_motor_wrap_angle_0_2pi(float angle_rad);

/**
 * @brief 根据机械角计算电角度。
 *
 * @param params 电机参数，使用极对数和零位偏置。
 * @param theta_m_rad 机械角，单位 rad。
 * @return 电角度，单位 rad，范围 [0, 2*pi)。
 */
float ns800_motor_mech_to_elec_angle(const ns800_motor_params_t *params, float theta_m_rad);

/**
 * @brief 执行一拍功率分配无刷电机控制。
 *
 * 开环模式直接生成 12 V 旋转电压矢量；闭环模式执行 Clarke/Park、
 * 速度 PI、电流 PI、逆 Park 和双端口 SVM。
 *
 * @param state 控制状态，函数会更新 PI 与 NCO。
 * @param cfg 控制配置；为空时使用默认配置。
 * @param params 电机参数；为空时使用默认参数。
 * @param in 单拍采样和命令。
 * @param out 单拍输出和诊断量。
 * @return 状态位图。
 */
ns800_motor_status_t ns800_motor_step(ns800_motor_state_t *state,
                                      const ns800_motor_config_t *cfg,
                                      const ns800_motor_params_t *params,
                                      const ns800_motor_input_t *in,
                                      ns800_motor_output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_MOTOR_CONTROL_H__ */
