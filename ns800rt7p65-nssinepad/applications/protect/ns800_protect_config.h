/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    ns800_protect_config.h
 * @brief   故障停机保护链的编译期配置：ADC PPB、ePWM X-BAR、EPWM 数字比较。
 *
 * 保护链(纯硬件快通路，不经过 CPU)：
 *
 *   ADCA SOC5(相电流 A) --> ADC PPB1 与 TRIPHI/TRIPLO 比较
 *        --> ADCAEVT1 --> ePWM X-BAR TRIP5 --> EPWM8~13 DCTRIPSEL(TRIPIN5)
 *        --> DCxEVT1 --> TZ one-shot 动作(LOW) --> 六路 PWM 立即封低
 *
 * 软件兜底：
 *   - 电机控制 ISR(20 kHz)对三相电流再做一次阈值比较，超限直接 TZFRC 强制 OST；
 *   - motguard 线程收到事件后停控制中断与 PWM，并锁存故障。
 *
 * @note 这里绑定的 SOC 必须与 `ns800_adc_background.c` 的通道映射一致：
 *       SOC5 -> ADC 通道 5 -> GPIOG11 -> A 相电流。改动采样链时同步改这里。
 */

#ifndef __NS800_PROTECT_CONFIG_H__
#define __NS800_PROTECT_CONFIG_H__

#include "adc.h"
#include "epwm.h"
#include "xbar.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 硬件快通路比较所用的 ADC 实例。 */
#define NS800_PROTECT_ADC                  ADCA
/** @brief 硬件快通路比较所用的 PPB。 */
#define NS800_PROTECT_PPB                  ADC_PPB_NUMBER1
/** @brief PPB 绑定的 SOC(必须对应 A 相电流采样)。 */
#define NS800_PROTECT_PPB_SOC              ADC_SOC_NUMBER5

/** @brief ADC PPB 事件位(EVTSEL bit0/bit1/bit2)，与驱动 drv_adc.h 保持一致。 */
#define NS800_PROTECT_PPB_EVT_TRIPHI       (1U << 0)
#define NS800_PROTECT_PPB_EVT_TRIPLO       (1U << 1)
#define NS800_PROTECT_PPB_EVT_ZERO         (1U << 2)
#define NS800_PROTECT_PPB_EVT_ALL          (NS800_PROTECT_PPB_EVT_TRIPHI | \
                                            NS800_PROTECT_PPB_EVT_TRIPLO | \
                                            NS800_PROTECT_PPB_EVT_ZERO)

/** @brief PPB 事件在 ePWM X-BAR 上的输入源：PPB1 -> ADCAEVT1。 */
#define NS800_PROTECT_XBAR_MUX_SIGNAL      XBAR_EPWM_MUX00_ADCAEVT1
/** @brief 占用的 ePWM X-BAR TRIP 线(mux0)。 */
#define NS800_PROTECT_XBAR_TRIP            XBAR_TRIPA5
/** @brief 对应的 mux 使能位。 */
#define NS800_PROTECT_XBAR_MUX             XBAR_MUX00
/** @brief EPWM 数字比较 A 的输入：必须与上面的 X-BAR TRIP 线一致。 */
#define NS800_PROTECT_DC_TRIP_INPUT        EPWM_DC_TRIP_TRIPIN5

/**
 * @brief 硬件快通路峰值电流阈值，单位 A。
 *
 * 40 A 约为默认 20 A 电流限幅的 2 倍，只用于"灾难级"过流；正常过流由 20 kHz
 * ISR 的软件阈值(22 A)先响应。阈值过小会因采样噪声误触发。
 */
#define NS800_PROTECT_HW_TRIP_PEAK_A       (40.0f)

/**
 * @brief 软件兜底峰值电流阈值，单位 A。
 *
 * 控制 ISR 每拍检查三相电流，超限立即 force one-shot trip。
 */
#define NS800_PROTECT_SW_TRIP_PEAK_A       (22.0f)

/**
 * @brief 上电后是否自动使能硬件快通路(0 = 只配置不使能)。
 *
 * bring-up 阶段如果怀疑 PPB 通道映射不对，可改成 0，先用 `protect_arm on` 手动验证。
 */
#define NS800_PROTECT_AUTO_ARM             (1)

#ifdef __cplusplus
}
#endif

#endif /* __NS800_PROTECT_CONFIG_H__ */
