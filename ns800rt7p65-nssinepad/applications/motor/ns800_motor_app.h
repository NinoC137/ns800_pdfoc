/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_MOTOR_APP_H__
#define __NS800_MOTOR_APP_H__

#include <rtthread.h>

#include "ns800_motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机应用层运行状态。
 *
 * 该结构用于 FinSH 查询和调试观测。ISR 会更新计数、故障状态和最近一拍输出。
 */
typedef struct
{
    /**< EPWM2 控制中断进入次数。 */
    volatile rt_uint32_t isr_count;
    /**< ADC DMA 帧未更新或不可用次数。 */
    volatile rt_uint32_t adc_miss_count;
    /**< 转子角度/速度反馈无效次数。 */
    volatile rt_uint32_t feedback_miss_count;
    /**< 最近一次电机控制状态位。 */
    volatile rt_uint32_t fault_flags;
    /**< 最近读取到的 ADC 帧序号。 */
    volatile rt_uint32_t adc_seq;
    /**< 当前电机控制模式。 */
    volatile ns800_motor_mode_t mode;
    /**< 最近一拍控制输出。 */
    ns800_motor_output_t last_output;
} ns800_motor_app_status_t;

/**
 * @brief 启动电机控制应用层。
 *
 * 初始化默认电机参数、开环配置、EPWM2 控制中断，默认进入开环模式。
 *
 * @return 0 表示成功。
 */
int ns800_motor_app_start(void);

/**
 * @brief 停止电机控制应用层。
 *
 * 关闭 EPWM2 控制中断并停止 EPWM8~13 PWM 输出。
 *
 * @return 0 表示成功。
 */
int ns800_motor_app_stop(void);

/**
 * @brief 切换电机控制模式。
 *
 * 切入速度闭环前会检查反馈接口是否有效，无反馈时拒绝切换。
 *
 * @param mode 目标模式。
 * @return 0 表示成功；负值表示失败。
 */
int ns800_motor_app_set_mode(ns800_motor_mode_t mode);

/**
 * @brief 获取电机应用层状态。
 *
 * @return 状态结构只读指针。
 */
const ns800_motor_app_status_t *ns800_motor_app_get_status(void);

/**
 * @brief 获取转子机械角和机械角速度。
 *
 * 默认弱实现返回无效。接入编码器或霍尔后，可在其他应用层文件中实现同名函数覆盖。
 *
 * @param theta_m_rad 输出机械角，单位 rad。
 * @param omega_m_rad_s 输出机械角速度，单位 rad/s。
 * @return RT_TRUE 表示反馈有效；RT_FALSE 表示无有效反馈。
 */
rt_bool_t ns800_motor_feedback_get(float *theta_m_rad, float *omega_m_rad_s);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_MOTOR_APP_H__ */
