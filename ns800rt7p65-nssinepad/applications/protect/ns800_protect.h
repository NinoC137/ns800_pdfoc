/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    ns800_protect.h
 * @brief   过流/硬件跳闸保护：ADC PPB 硬件快通路 + 20 kHz 软件兜底。
 *
 * 三层响应时间：
 *   1. 硬件：ADC PPB 比较 -> X-BAR -> EPWM TZ，纳秒级，不经过 CPU；
 *   2. 软件快路：控制 ISR 内比较三相电流，超限直接 `EPWM_forceTripZoneEvent()`；
 *   3. 慢速兜底：motguard 线程收事件后停控制中断与 PWM，见 `ns800_motor_app.c`。
 *
 * 硬件通路在 `ns800_protect_init()` 中配置，可用 @ref ns800_protect_arm 运行时开关。
 */

#ifndef __NS800_PROTECT_H__
#define __NS800_PROTECT_H__

#include <rtthread.h>

#include "ns800_protect_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 保护链状态位。 */
#define NS800_PROTECT_FLAG_HW_TRIP      (1U << 0)   /**< 硬件通路已跳闸(EPWM TZ 锁存)。 */
#define NS800_PROTECT_FLAG_SW_TRIP      (1U << 1)   /**< 软件电流阈值触发跳闸。 */
#define NS800_PROTECT_FLAG_ARMED        (1U << 2)   /**< PPB/X-BAR 硬件通路当前使能。 */

/** @brief 保护链诊断状态。 */
typedef struct
{
    volatile rt_uint32_t flags;         /**< @ref NS800_PROTECT_FLAG_* 组合。 */
    volatile rt_uint32_t hw_trip_count; /**< 硬件 TZ 锁存被观测到的次数。 */
    volatile rt_uint32_t sw_trip_count; /**< 软件阈值跳闸次数。 */
    rt_uint16_t ppb_high_counts;        /**< PPB 上限阈值(ADC counts)。 */
    rt_uint16_t ppb_low_counts;         /**< PPB 下限阈值(ADC counts)。 */
    float hw_trip_peak_a;               /**< 硬件阈值折算电流，单位 A。 */
    float sw_trip_peak_a;               /**< 软件阈值，单位 A。 */
} ns800_protect_status_t;

/**
 * @brief 配置保护链：ADC PPB1 限值、X-BAR 路由、EPWM8~13 的 TZ 动作。
 *
 * 必须先于大电流工况调用。函数本身不启动 PWM，也不清零 EPWM 现有配置。
 *
 * @return 0 表示成功；负值表示参数越界。
 */
int ns800_protect_init(void);

/**
 * @brief 运行时使能/关闭硬件快通路(PPB 事件与 X-BAR mux)。
 *
 * 关闭后软件兜底仍然有效，可用于 bring-up 阶段排除 PPB 误触发。
 *
 * @param enabled RT_TRUE 使能；RT_FALSE 关闭。
 */
void ns800_protect_arm(rt_bool_t enabled);

/**
 * @brief 查询硬件快通路是否使能。
 *
 * @return RT_TRUE 表示已使能。
 */
rt_bool_t ns800_protect_is_armed(void);

/**
 * @brief 控制 ISR 内的三相电流阈值检查(软件快路)。
 *
 * 超限时立即触发 EPWM one-shot trip 并锁存软件跳闸标志。函数无阻塞、可重入。
 *
 * @param ia A 相电流，单位 A。
 * @param ib B 相电流，单位 A。
 * @param ic C 相电流，单位 A。
 * @return RT_TRUE 表示本次调用触发了跳闸。
 */
rt_bool_t ns800_protect_check_currents(float ia, float ib, float ic);

/**
 * @brief 查询当前是否处于跳闸状态(硬件锁存或软件标志)。
 *
 * @return RT_TRUE 表示已跳闸。
 */
rt_bool_t ns800_protect_is_tripped(void);

/**
 * @brief 清除硬件 TZ 锁存与软件标志。
 *
 * 只在确认过流原因已消除后调用；调用方还需恢复 PWM 与控制中断。
 */
void ns800_protect_clear(void);

/**
 * @brief 获取保护状态(只读)。
 *
 * @return 状态结构只读指针，永不为空。
 */
const ns800_protect_status_t *ns800_protect_get_status(void);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_PROTECT_H__ */
