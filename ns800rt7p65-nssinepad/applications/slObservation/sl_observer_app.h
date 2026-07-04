/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    sl_observer_app.h
 * @brief   无感观测器应用层/ISR 胶水接口。
 *
 * 本层持有观测器的静态 config/state/output 实例,向电机控制 ISR 提供一个
 * 无阻塞的单拍更新入口 @ref sl_observer_app_update,并向 FinSH/上层提供只读
 * 结果查询。设计约束:
 *
 * - @ref sl_observer_app_update 在 EPWM2 控制 ISR(10 kHz)内调用,内部只做
 *   Clarke 变换与一次 @ref sl_observer_step,禁止 printf/动态内存/阻塞;
 * - 查询接口只读取已算好的静态结果,不触碰 EMATH,可在任意线程上下文调用;
 * - 观测器全程运行(含开环模式),但仅当 @ref sl_observer_output_t::valid 为
 *   真时角度/转速才可信。
 *
 * 纯算法内核见 @ref sl_observer.h。
 */

#ifndef SL_OBSERVER_APP_H
#define SL_OBSERVER_APP_H

#include "sl_observer.h"
#include "svm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化并启动无感观测器应用层。
 *
 * 载入默认配置、清零状态与输出。可安全重复调用(等效于复位)。
 *
 * @return 0 表示成功。
 */
int sl_observer_app_start(void);

/**
 * @brief 单拍更新无感观测器(在电机控制 ISR 内调用)。
 *
 * 内部先用 @ref svm_clarke 把三相电流变换到 alpha/beta,再执行一次
 * @ref sl_observer_step,并刷新静态输出。入参为空指针时直接返回。
 *
 * @param v_ab  施加相电压矢量(alpha/beta),单位 V;取自控制器 voltage_cmd_ab。
 * @param i_abc 实测三相电流,单位 A;取自 ADC 采样。
 */
void sl_observer_app_update(const svm_ab_f32_t *v_ab, const svm_abc_f32_t *i_abc);

/**
 * @brief 获取观测器最近一拍输出(只读)。
 *
 * @return 静态输出结构只读指针,永不为空。
 */
const sl_observer_output_t *sl_observer_app_get_output(void);

/**
 * @brief 获取估计电角度。
 *
 * @return 估计电角度,单位 rad,范围 [0, 2*pi)。
 */
float sl_observer_app_get_theta_e(void);

/**
 * @brief 获取估计机械转速。
 *
 * @return 估计机械转速,单位 rpm(有效性另见 @ref sl_observer_app_get_output)。
 */
float sl_observer_app_get_speed_rpm(void);

#ifdef __cplusplus
}
#endif

#endif /* SL_OBSERVER_APP_H */
