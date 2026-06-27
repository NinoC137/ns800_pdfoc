/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_BUTTON_APP_H__
#define __NS800_BUTTON_APP_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 功率分配系数，电机控制中限制到 0.0~1.0。 */
extern volatile float ns800_param_xi;
/** 目标机械转速，单位 rpm。 */
extern volatile rt_int32_t ns800_param_speed;
/** 力矩/电流调节量，当前作为 q 轴电流限幅，单位 mA。 */
extern volatile rt_int32_t ns800_param_force_ma;

/**
 * @brief 启动 MultiButton 按键扫描线程。
 *
 * @return 0 表示成功；负值表示线程创建失败。
 */
int ns800_button_app_start(void);

/**
 * @brief 将按键控制参数恢复默认值。
 *
 * 默认值为 xi=0.5、speed=2000 rpm、F=500 mA，并递增 reset 事件计数。
 */
void ns800_button_app_reset_params(void);

/**
 * @brief 获取 reset 事件计数。
 *
 * 电机控制 ISR 通过该计数判断是否需要清零 PI 状态。
 *
 * @return reset 事件累计次数。
 */
rt_uint32_t ns800_button_app_reset_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_BUTTON_APP_H__ */
