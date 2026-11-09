/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_PWM_APP_H__
#define __NS800_PWM_APP_H__

#include <rtthread.h>
#include "ns800_motor_config.h"
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
#define NS800_PWM_APP_FREQ_HZ              20000U
#define NS800_PWM_APP_TBPRD                (NS800_MOTOR_PWM_TBCLK_HZ / (2U * NS800_PWM_APP_FREQ_HZ))

int ns800_pwm_app_start(void);
int ns800_pwm_app_stop(void);
int ns800_pwm_app_resume(void);
int ns800_pwm_app_set_duty(rt_uint32_t epwm_id,
                           rt_uint32_t channel,
                           rt_uint32_t duty_permille);
rt_uint32_t ns800_pwm_app_get_duty(rt_uint32_t epwm_id,
                                   rt_uint32_t channel);
void ns800_pwm_app_write_svm(const svm_dual_out_t *duty);

/**
 * @brief 配置六路功率 PWM 的硬件跳闸通路(ADC X-BAR TRIP -> 数字比较 -> TZ)。
 *
 * 把 `ns800_protect_config.h` 里约定的 TRIP 线接到 EPWM8~13 的 DCAEVT1，
 * 并把 one-shot 动作设为强制输出低。可重复调用。
 *
 * @return 0 表示成功。
 */
int ns800_pwm_app_trip_config(void);

/**
 * @brief 立即强制一次 one-shot trip(软件快路)。
 *
 * 只写 TZFRC，不停止计数器；输出在下一次 TZ 动作点变成低电平。可在 ISR 调用。
 */
void ns800_pwm_app_trip_force(void);

/**
 * @brief 清除六路 PWM 的 one-shot trip 锁存。
 */
void ns800_pwm_app_trip_clear(void);

/**
 * @brief 查询六路 PWM 是否处于 trip 锁存状态。
 *
 * @return RT_TRUE 表示至少一路 PWM 被 TZ 封锁。
 */
rt_bool_t ns800_pwm_app_trip_active(void);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_PWM_APP_H__ */
