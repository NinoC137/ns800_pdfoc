/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_PWM_APP_H__
#define __NS800_PWM_APP_H__

#include <rtthread.h>
#include "svm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NS800_PWM_APP_MIN_EPWM_ID          8U
#define NS800_PWM_APP_MAX_EPWM_ID          13U
#define NS800_PWM_APP_CHANNEL_A            1U
#define NS800_PWM_APP_CHANNEL_B            2U
#define NS800_PWM_APP_DUTY_PERMILLE_MAX    1000U
#define NS800_PWM_APP_DEFAULT_DUTY         500U
#define NS800_PWM_APP_TBPRD_30KHZ          6667U

int ns800_pwm_app_start(void);
int ns800_pwm_app_stop(void);
int ns800_pwm_app_set_duty(rt_uint32_t epwm_id,
                           rt_uint32_t channel,
                           rt_uint32_t duty_permille);
rt_uint32_t ns800_pwm_app_get_duty(rt_uint32_t epwm_id,
                                   rt_uint32_t channel);
void ns800_pwm_app_write_svm(const svm_dual_out_t *duty);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_PWM_APP_H__ */
