/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    sl_observer.c
 * @brief   三相无刷电机无感转子观测器实现:滑模观测器(SMO)+ 锁相环(PLL)。
 *
 * 本文件是 @ref sl_observer.h 声明的纯算法实现,工作在 alpha/beta 静止坐标系。
 * 一拍 @ref sl_observer_step 依次完成:
 *
 *   1. 滑模电流观测器  —— 由电压/电流重构反电动势的开关等效控制量;
 *   2. 反电动势低通    —— 平滑滑模切换纹波,得到反电动势估计;
 *   3. PLL 角度/转速   —— 从反电动势提取电角度,并直接给出电角速度。
 *
 * @par 抗抖振(chattering)三层措施
 * - 第 1 层:滑模开关函数用连续边界层 @f$ s/(|s|+\delta) @f$ 取代硬 @f$ sign(s) @f$,
 *   从源头消除高频切换;
 * - 第 2 层:对滑模等效控制量做一阶低通,滤除残余开关纹波;
 * - 第 3 层:PLL 环路本身是二阶低通,进一步平滑角度与转速,且转速由 PLL 的
 *   PI 输出直接得到,无需对角度做差分,避免噪声放大。
 *
 * @par EMATH 复用与重入
 * 本文件通过 @ref svm_sincos 及 @ref EMATH_sqrtF32 / @ref EMATH_invF32 /
 * @ref EMATH_atanF32 调用 NS800 硬件数学协处理器。这些调用均为单次阻塞式操作。
 * 观测器只在 EPWM2 控制 ISR 内被调用,与现有 FOC 控制的 EMATH 调用处于同一
 * 中断上下文、天然串行,不存在重入冲突。若日后在其他线程/中断上下文复用本模块,
 * 需自行对 EMATH 访问加互斥保护。
 *
 * @par 已知局限:仅正向旋转
 * 归一化鉴相器 @f$ \varepsilon \approx sign(\omega_e)\,sin(\theta_e-\hat\theta) @f$
 * 在反向旋转(@f$ \omega_e<0 @f$)时环路增益变号、锁相环失稳。本实现面向工程
 * 默认的开环正向起旋验证场景,只保证正向可靠跟踪;反向运行超出本次范围。
 */

#include "sl_observer.h"

#include "sl_observer_config.h"
#include "emath.h"
#include "svm_transform.h"

/**
 * @brief 计算 float 绝对值。
 *
 * 避免在 ISR 控制路径中引入额外的 libm 依赖,与工程既有风格保持一致。
 *
 * @param value 输入值。
 * @return 非负绝对值。
 */
static float sl_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 将角度折返到 [0, 2*pi)。
 *
 * @param angle_rad 输入角度,单位 rad。
 * @return 折返后的角度,单位 rad。
 */
static float sl_wrap_0_2pi(float angle_rad)
{
    while (angle_rad >= SL_OBS_TWO_PI)
    {
        angle_rad -= SL_OBS_TWO_PI;
    }
    while (angle_rad < 0.0f)
    {
        angle_rad += SL_OBS_TWO_PI;
    }
    return angle_rad;
}

void sl_observer_default_config(sl_observer_config_t *cfg)
{
    if (cfg == 0)
    {
        return;
    }

    /* 电机电气参数(占位默认,实测后须整定)。 */
    cfg->rs_ohm = SL_OBS_DEFAULT_RS_OHM;
    cfg->ls_h = SL_OBS_DEFAULT_LS_H;
    cfg->flux_wb = SL_OBS_DEFAULT_FLUX_WB;
    cfg->pole_pairs = SL_OBS_DEFAULT_POLE_PAIRS;
    cfg->sample_time_s = SL_OBS_PERIOD_S;

    /* 滑模电流观测器增益与边界层。 */
    cfg->k_slide = SL_OBS_DEFAULT_K_SLIDE;
    cfg->boundary_delta = SL_OBS_DEFAULT_BOUNDARY_DELTA;

    /* 反电动势低通与相位补偿。 */
    cfg->emf_lpf_wc = SL_OBS_DEFAULT_EMF_LPF_WC;
    cfg->comp_phase_lag = (SL_OBS_DEFAULT_COMP_PHASE_LAG != 0);

    /* PLL 环路 PI:归一化鉴相器输出即估计电角速度,故输出/积分限幅取电角速度上限。 */
    cfg->pll_pi.kp = SL_OBS_DEFAULT_PLL_KP;
    cfg->pll_pi.ki = SL_OBS_DEFAULT_PLL_KI;
    cfg->pll_pi.sample_time_s = SL_OBS_PERIOD_S;
    cfg->pll_pi.out_min = -SL_OBS_DEFAULT_OMEGA_MAX_RAD_S;
    cfg->pll_pi.out_max = SL_OBS_DEFAULT_OMEGA_MAX_RAD_S;
    cfg->pll_pi.integrator_min = -SL_OBS_DEFAULT_OMEGA_MAX_RAD_S;
    cfg->pll_pi.integrator_max = SL_OBS_DEFAULT_OMEGA_MAX_RAD_S;
    cfg->pll_pi.limit_output = true;
    cfg->pll_pi.limit_integrator = true;
    cfg->pll_pi.anti_windup = true;

    /* 观测有效性阈值。 */
    cfg->emf_valid_min = SL_OBS_DEFAULT_EMF_VALID_MIN;
    cfg->omega_valid_min = SL_OBS_DEFAULT_OMEGA_VALID_MIN;
}

void sl_observer_init(sl_observer_state_t *state, const sl_observer_config_t *cfg)
{
    /* cfg 当前仅用于接口对齐,状态初始化不依赖具体参数。 */
    (void)cfg;
    sl_observer_reset(state);
}

void sl_observer_reset(sl_observer_state_t *state)
{
    if (state == 0)
    {
        return;
    }

    state->i_hat_alpha = 0.0f;
    state->i_hat_beta = 0.0f;
    state->emf_alpha = 0.0f;
    state->emf_beta = 0.0f;
    state->theta_pll = 0.0f;
    svm_pi_init_state(&state->pll_pi);
}

/**
 * @brief 单轴滑模电流观测器 + 反电动势低通。
 *
 * 实现电流观测器前向欧拉更新与反电动势一阶低通(抗抖振第 1、2 层)。
 * 电机模型 @f$ L_s\,di/dt = -R_s i + v - e @f$,取估计电流误差为滑模面
 * @f$ s=\hat i - i @f$,连续边界层开关量 @f$ z=k\,s/(|s|+\delta) @f$ 在滑模面
 * 上等效于反电动势,低通后即得反电动势估计。
 *
 * @param i_hat        [in,out] 观测电流状态指针,单位 A;函数内前向欧拉更新。
 * @param emf          [in,out] 低通后反电动势状态指针,单位 V;函数内更新。
 * @param i_meas       实测电流,单位 A。
 * @param v_in         施加电压,单位 V。
 * @param ts_over_ls   Ts/Ls 预计算系数,单位 s/H。
 * @param rs           定子相电阻,单位 ohm。
 * @param k_slide      滑模增益,单位 V。
 * @param delta        边界层宽度,单位 A。
 * @param lpf_gain     反电动势低通增益 wc*Ts(无量纲,应 < 1)。
 */
static void sl_observer_axis_step(float *i_hat,
                                  float *emf,
                                  float i_meas,
                                  float v_in,
                                  float ts_over_ls,
                                  float rs,
                                  float k_slide,
                                  float delta,
                                  float lpf_gain)
{
    float s;
    float z;

    /* 抗抖振第 1 层:连续边界层开关函数取代硬 sign(s)。 */
    s = *i_hat - i_meas;
    z = k_slide * s / (sl_abs(s) + delta);

    /* 电流观测器前向欧拉更新:i_hat += (Ts/Ls)*(-Rs*i_hat + v - z)。 */
    *i_hat += ts_over_ls * ((v_in - (rs * (*i_hat))) - z);

    /* 抗抖振第 2 层:一阶低通平滑滑模等效控制量,得到反电动势估计。 */
    *emf += lpf_gain * (z - *emf);
}

void sl_observer_step(sl_observer_state_t *state,
                      const sl_observer_config_t *cfg,
                      const svm_ab_f32_t *v_ab,
                      const svm_ab_f32_t *i_ab,
                      sl_observer_output_t *out)
{
    sl_observer_config_t default_cfg;
    svm_sincos_f32_t sc;
    float ts_over_ls;
    float lpf_gain;
    float ea;
    float eb;
    float emf_mag;
    float eps;
    float omega_e;
    float theta_e;
    float pole_pairs_f;

    if ((state == 0) || (out == 0))
    {
        return;
    }
    if (cfg == 0)
    {
        sl_observer_default_config(&default_cfg);
        cfg = &default_cfg;
    }
    if ((v_ab == 0) || (i_ab == 0))
    {
        /* 输入缺失:保留上一拍角度,但标记本拍无效。 */
        out->valid = false;
        return;
    }

    /* 预计算离散系数,防止 Ls 非法(<=0)导致除零/发散。 */
    if (cfg->ls_h > 1.0e-9f)
    {
        ts_over_ls = cfg->sample_time_s / cfg->ls_h;
    }
    else
    {
        ts_over_ls = 0.0f;
    }
    lpf_gain = cfg->emf_lpf_wc * cfg->sample_time_s;

    /* ---- ①②滑模电流观测 + 反电动势低通(逐轴)---- */
    sl_observer_axis_step(&state->i_hat_alpha, &state->emf_alpha,
                          i_ab->alpha, v_ab->alpha,
                          ts_over_ls, cfg->rs_ohm,
                          cfg->k_slide, cfg->boundary_delta, lpf_gain);
    sl_observer_axis_step(&state->i_hat_beta, &state->emf_beta,
                          i_ab->beta, v_ab->beta,
                          ts_over_ls, cfg->rs_ohm,
                          cfg->k_slide, cfg->boundary_delta, lpf_gain);

    ea = state->emf_alpha;
    eb = state->emf_beta;

    /* ---- ③PLL 角度/转速跟踪 ---- */
    emf_mag = EMATH_sqrtF32((ea * ea) + (eb * eb));

    if (emf_mag > cfg->emf_valid_min)
    {
        float inv = EMATH_invF32(emf_mag);
        float na = ea * inv;   /* 归一化反电动势 alpha = -sin(theta_e)(正转) */
        float nb = eb * inv;   /* 归一化反电动势 beta  =  cos(theta_e)(正转) */

        /* 鉴相器 eps = -na*cos(theta_hat) - nb*sin(theta_hat) ≈ sin(theta_e - theta_hat)。
         * 归一化使环路增益与转速/反电动势幅值无关,便于整定并抑制幅值扰动。 */
        svm_sincos(state->theta_pll, &sc);
        eps = -(na * sc.cos_theta) - (nb * sc.sin_theta);
    }
    else
    {
        /* 低速/静止失效区:反电动势过小,鉴相无意义。喂 0 误差使 PLL 保持
         * 上一拍频率(积分器不变),角度按该频率惯性外推,避免噪声注入。 */
        eps = 0.0f;
    }

    /* PLL 环路 PI:输出即估计电角速度(复用 svm_pi,含钳位与抗饱和)。 */
    omega_e = svm_pi_update(&state->pll_pi, &cfg->pll_pi, eps, false);

    /* 积分得到 PLL 内部电角度(锁定到低通后反电动势相位)。 */
    state->theta_pll = sl_wrap_0_2pi(state->theta_pll + (omega_e * cfg->sample_time_s));

    /* 抗抖振/相位补偿:低通对反电动势引入相位滞后 atan(omega_e/wc),
     * 补回到真实电角度。wc>0 恒成立,故用 atan(y/x) 即可,无需象限修正。 */
    if (cfg->comp_phase_lag && (cfg->emf_lpf_wc > 1.0e-6f))
    {
        float phase_lag = EMATH_atanF32(omega_e, cfg->emf_lpf_wc);
        theta_e = sl_wrap_0_2pi(state->theta_pll + phase_lag);
    }
    else
    {
        theta_e = state->theta_pll;
    }

    /* ---- 输出与诊断量 ---- */
    pole_pairs_f = (cfg->pole_pairs > 0u) ? (float)cfg->pole_pairs : 1.0f;

    out->theta_e_rad = theta_e;
    out->omega_e_rad_s = omega_e;
    out->omega_m_rad_s = omega_e / pole_pairs_f;
    out->speed_rpm = out->omega_m_rad_s * SL_OBS_RAD_S_TO_RPM;
    out->emf_alpha = ea;
    out->emf_beta = eb;
    out->emf_mag = emf_mag;
    out->i_hat_alpha = state->i_hat_alpha;
    out->i_hat_beta = state->i_hat_beta;

    /* 有效性:反电动势足够大且估计转速高于阈值时,角度/转速方可信。 */
    out->valid = (emf_mag > cfg->emf_valid_min) &&
                 (sl_abs(omega_e) > cfg->omega_valid_min);
}
