/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    sl_observer_config.h
 * @brief   无感滑模观测器(SMO + PLL)默认参数与常量。
 *
 * 本文件集中放置观测器的默认电机电气参数、滑模增益、反电动势低通截止频率、
 * PLL 环路增益以及有效性阈值。所有默认值均为“占位值”,仅用于保证软件链路
 * 可编译、可跑通;实测电机后应通过 @ref sl_observer_config_t 或 FinSH 命令
 * `sl_obs_set` 在线整定为真实参数。
 *
 * @note  控制周期与电机控制 ISR 保持一致(EPWM2 @ 10 kHz,Ts = 1e-4 s)。
 */

#ifndef SL_OBSERVER_CONFIG_H
#define SL_OBSERVER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/** @name 数学常量 */
/** @{ */
#define SL_OBS_PI                 (3.1415926535897932385f)   /**< 圆周率 pi。 */
#define SL_OBS_TWO_PI             (6.2831853071795864769f)   /**< 2*pi,用于电角度折返。 */
#define SL_OBS_RAD_S_TO_RPM       (9.5492965855137201461f)   /**< rad/s 转 rpm 系数,即 60/(2*pi)。 */
/** @} */

/** @name 控制周期 */
/** @{ */
#define SL_OBS_PERIOD_S           (1.0e-4f)                  /**< 观测器离散步长,单位 s(与 EPWM2 控制中断同步)。 */
/** @} */

/** @name 电机电气参数(实测后须整定) */
/** @{ */
#define SL_OBS_DEFAULT_RS_OHM     (13.0f)                     /**< 定子相电阻,单位 ohm。 */
#define SL_OBS_DEFAULT_LS_H       (13.0e-3f)                  /**< 定子相电感,单位 H(表贴式取 Ld=Lq)。 */
#define SL_OBS_DEFAULT_FLUX_WB    (0.02f)                    /**< 永磁体磁链,单位 Wb。 */
#define SL_OBS_DEFAULT_POLE_PAIRS (2u)                       /**< 极对数。 */
/** @} */

/** @name 滑模电流观测器增益 */
/** @{ */
/**
 * @brief 滑模增益 k_slide,单位 V。
 *
 * 需大于工作转速范围内反电动势峰值 E = psi_f * omega_e,才能保证滑模到达条件。
 * 例:psi_f=0.02、开环 50 Hz 电频(omega_e≈314 rad/s)时 E≈6.3 V,默认 20 V 留足裕量。
 * 转速更高时应上调。
 */
#define SL_OBS_DEFAULT_K_SLIDE    (20.0f)

/**
 * @brief 边界层宽度 delta,单位 A(与电流误差同量纲)。
 *
 * 抗抖振核心参数:开关函数用连续的 s/(|s|+delta) 取代硬 sign(s)。delta 越大越平滑
 * 但带宽损失越多;delta 越小越接近理想滑模但抖振越强。默认 1.0 A。
 */
#define SL_OBS_DEFAULT_BOUNDARY_DELTA (0.08f)
/** @} */

/** @name 反电动势低通滤波 */
/** @{ */
/**
 * @brief 反电动势一阶低通截止角频率 wc,单位 rad/s。
 *
 * 抗抖振第二层:平滑滑模等效控制(切换量)得到反电动势估计。需高于反电动势基频、
 * 低于开关纹波。默认 1000 rad/s(≈159 Hz)。离散增益 wc*Ts 必须 < 1。
 */
#define SL_OBS_DEFAULT_EMF_LPF_WC (1000.0f)

/** @brief 是否补偿反电动势 LPF 引入的相位滞后(1 使能)。 */
#define SL_OBS_DEFAULT_COMP_PHASE_LAG (1)
/** @} */

/** @name PLL 环路 PI 增益 */
/** @{ */
/**
 * @brief PLL 环路 PI 比例/积分增益。
 *
 * 归一化鉴相器输出 eps≈sin(theta_err),PI 输出即估计电角速度 omega_e。
 * 环路自然频率 wn≈sqrt(ki),阻尼 zeta≈kp/(2*sqrt(ki))。默认 kp=150、ki=8000
 * 对应 wn≈89 rad/s、zeta≈0.84。
 */
#define SL_OBS_DEFAULT_PLL_KP     (15.0f)
#define SL_OBS_DEFAULT_PLL_KI     (800.0f)                  /**< @see SL_OBS_DEFAULT_PLL_KP */

/** @brief PLL 输出(估计电角速度)对称限幅,单位 rad/s。 */
#define SL_OBS_DEFAULT_OMEGA_MAX_RAD_S (3000.0f)
/** @} */

/** @name 观测有效性阈值 */
/** @{ */
/** @brief 反电动势幅值有效阈值,单位 V;低于该值判定为低速失效区。 */
#define SL_OBS_DEFAULT_EMF_VALID_MIN   (0.2f)

/** @brief 估计电角速度有效阈值,单位 rad/s;低于该值判定观测不可信。 */
#define SL_OBS_DEFAULT_OMEGA_VALID_MIN (5.0f)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* SL_OBSERVER_CONFIG_H */
