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

typedef struct
{
    volatile rt_uint32_t isr_count;
    volatile rt_uint32_t adc_miss_count;
    volatile rt_uint32_t feedback_miss_count;
    volatile rt_uint32_t fault_flags;
    volatile rt_uint32_t adc_seq;
    volatile ns800_motor_mode_t mode;
    ns800_motor_output_t last_output;
} ns800_motor_app_status_t;

int ns800_motor_app_start(void);
int ns800_motor_app_stop(void);
int ns800_motor_app_set_mode(ns800_motor_mode_t mode);
const ns800_motor_app_status_t *ns800_motor_app_get_status(void);
rt_bool_t ns800_motor_feedback_get(float *theta_m_rad, float *omega_m_rad_s);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_MOTOR_APP_H__ */
