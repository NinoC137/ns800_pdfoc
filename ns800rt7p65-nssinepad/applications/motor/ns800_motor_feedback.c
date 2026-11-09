/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    ns800_motor_feedback.c
 * @brief   用无感观测器实现转子反馈接口(覆盖 `ns800_motor_app.c` 的 weak 符号)。
 *
 * 观测器在控制 ISR 中先于控制计算更新，因此这里读到的角度/转速属于"本拍"。
 * 观测器无效(低速/反电动势过小)时返回 RT_FALSE，控制层据此拒绝进入或退出
 * 速度闭环，而不是拿 0 角度硬跑。
 *
 * 机械角由电角度除以极对数得到；`ns800_motor_mech_to_elec_angle()` 会再乘回去
 * 并叠加零位偏置，因此角度链路保持自洽。
 */

#include <rtthread.h>

#include "ns800_motor_app.h"
#include "ns800_motor_config.h"
#include "sl_observer_app.h"

rt_bool_t ns800_motor_feedback_get(float *theta_m_rad, float *omega_m_rad_s)
{
    const sl_observer_output_t *obs = sl_observer_app_get_output();
    float pole_pairs;

    if ((obs == RT_NULL) || (obs->valid != true))
    {
        return RT_FALSE;
    }

    pole_pairs = (float)NS800_MOTOR_DEFAULT_POLE_PAIRS;
    if (pole_pairs < 1.0f)
    {
        pole_pairs = 1.0f;
    }

    if (theta_m_rad != RT_NULL)
    {
        *theta_m_rad = obs->theta_e_rad / pole_pairs;
    }
    if (omega_m_rad_s != RT_NULL)
    {
        *omega_m_rad_s = obs->omega_m_rad_s;
    }

    return RT_TRUE;
}
