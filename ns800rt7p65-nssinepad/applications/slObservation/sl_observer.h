/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    sl_observer.h
 * @brief   三相无刷电机无感转子观测器:滑模观测器(SMO)+ 锁相环(PLL)。
 *
 * 观测器工作在 alpha/beta 静止坐标系,输入为施加相电压矢量 v_ab 与实测相电流矢量
 * i_ab,输出估计电角度 theta_e 与估计电/机械角速度。核心思路:
 *
 * 1. 滑模电流观测器由电压电流重构反电动势(用连续边界层函数抑制抖振);
 * 2. 一阶低通平滑反电动势;
 * 3. PLL 从反电动势提取角度,并直接给出转速(PLL 的频率状态即转速,无需对角度差分)。
 *
 * 本模块为纯算法层,不含任何硬件/RTOS 依赖,可单独进行单元测试。硬件适配(ISR 喂数据、
 * FinSH 读出)见 @ref sl_observer_app.h。
 *
 * @note  该文件仅实现观测,不解决零速起动(反电动势趋零时观测失效),起动仍依赖开环。
 */

#ifndef SL_OBSERVER_H
#define SL_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

#include "svm_pi.h"
#include "svm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 无感观测器配置。
 *
 * 电气参数(rs/ls/flux/pole_pairs)决定观测精度,须与实际电机一致;其余为观测器
 * 调谐参数。默认值见 @ref sl_observer_default_config。
 */
typedef struct
{
    float rs_ohm;             /**< 定子相电阻,单位 ohm。 */
    float ls_h;               /**< 定子相电感,单位 H(表贴式取 Ld=Lq)。 */
    float flux_wb;            /**< 永磁体磁链,单位 Wb(仅用于换算,不进入电流观测器)。 */
    uint16_t pole_pairs;      /**< 极对数,用于电角速度换算机械角速度。 */
    float sample_time_s;      /**< 离散步长,单位 s。 */

    float k_slide;            /**< 滑模增益,单位 V。 */
    float boundary_delta;     /**< 边界层宽度,单位 A;抗抖振核心参数。 */

    float emf_lpf_wc;         /**< 反电动势一阶低通截止角频率,单位 rad/s。 */
    bool comp_phase_lag;      /**< 是否补偿反电动势 LPF 相位滞后。 */

    svm_pi_config_t pll_pi;   /**< PLL 环路 PI 配置(复用 svm_pi)。 */

    float emf_valid_min;      /**< 反电动势幅值有效阈值,单位 V。 */
    float omega_valid_min;    /**< 估计电角速度有效阈值,单位 rad/s。 */
} sl_observer_config_t;

/**
 * @brief 无感观测器运行状态。
 *
 * 该结构由 @ref sl_observer_step 持续更新,调用方不应在步进之外直接修改。
 */
typedef struct
{
    float i_hat_alpha;        /**< 观测电流 alpha 轴,单位 A。 */
    float i_hat_beta;         /**< 观测电流 beta 轴,单位 A。 */
    float emf_alpha;          /**< 低通后反电动势 alpha 轴,单位 V。 */
    float emf_beta;           /**< 低通后反电动势 beta 轴,单位 V。 */
    float theta_pll;          /**< PLL 内部角度(未做相位补偿),单位 rad,范围 [0,2*pi)。 */
    svm_pi_state_t pll_pi;    /**< PLL 环路 PI 状态。 */
} sl_observer_state_t;

/**
 * @brief 无感观测器单拍输出与诊断量。
 */
typedef struct
{
    float theta_e_rad;        /**< 估计电角度(已相位补偿),单位 rad,范围 [0,2*pi)。 */
    float omega_e_rad_s;      /**< 估计电角速度,单位 rad/s。 */
    float omega_m_rad_s;      /**< 估计机械角速度,单位 rad/s。 */
    float speed_rpm;          /**< 估计机械转速,单位 rpm。 */
    float emf_alpha;          /**< 反电动势 alpha 轴,单位 V。 */
    float emf_beta;           /**< 反电动势 beta 轴,单位 V。 */
    float emf_mag;            /**< 反电动势幅值,单位 V。 */
    float i_hat_alpha;        /**< 观测电流 alpha 轴,单位 A(调试)。 */
    float i_hat_beta;         /**< 观测电流 beta 轴,单位 A(调试)。 */
    bool valid;              /**< 观测是否有效(低速/反电动势过小时为 false)。 */
} sl_observer_output_t;

/**
 * @brief 填充默认观测器配置。
 *
 * 使用 @ref sl_observer_config.h 中的占位默认值。实测前须整定电气参数。
 *
 * @param cfg 输出配置指针;为空时直接返回。
 */
void sl_observer_default_config(sl_observer_config_t *cfg);

/**
 * @brief 初始化观测器状态。
 *
 * 清零观测电流、反电动势、PLL 角度与 PI 状态。
 *
 * @param state 状态指针;为空时直接返回。
 * @param cfg   配置指针;当前仅用于接口对齐,允许为空。
 */
void sl_observer_init(sl_observer_state_t *state, const sl_observer_config_t *cfg);

/**
 * @brief 复位观测器状态(不改变配置)。
 *
 * @param state 状态指针;为空时直接返回。
 */
void sl_observer_reset(sl_observer_state_t *state);

/**
 * @brief 执行一拍无感观测。
 *
 * 依次完成:滑模电流观测(边界层抗抖振)→ 反电动势低通 → PLL 角度/转速跟踪
 * →(可选)相位滞后补偿 → 有效性判定。
 *
 * @param state 观测器状态,函数会更新。
 * @param cfg   观测器配置;为空时使用默认配置。
 * @param v_ab  施加相电压矢量(alpha/beta),单位 V。
 * @param i_ab  实测相电流矢量(alpha/beta),单位 A。
 * @param out   单拍输出;为空时直接返回。
 */
void sl_observer_step(sl_observer_state_t *state,
                      const sl_observer_config_t *cfg,
                      const svm_ab_f32_t *v_ab,
                      const svm_ab_f32_t *i_ab,
                      sl_observer_output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* SL_OBSERVER_H */
