/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_MOTOR_CONFIG_H__
#define __NS800_MOTOR_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#define NS800_MOTOR_EPWM_TBCLK_HZ                  200000000U
#define NS800_MOTOR_CONTROL_FREQ_HZ                20000U
#define NS800_MOTOR_CONTROL_TBPRD                  (NS800_MOTOR_EPWM_TBCLK_HZ / (2U * NS800_MOTOR_CONTROL_FREQ_HZ))
#define NS800_MOTOR_CONTROL_PERIOD_S               (1.0f / (float)NS800_MOTOR_CONTROL_FREQ_HZ)

#define NS800_MOTOR_DEFAULT_POLE_PAIRS             7U
#define NS800_MOTOR_DEFAULT_OPEN_LOOP_RPM          (500.0f)
#define NS800_MOTOR_DEFAULT_OPEN_LOOP_FREQ_HZ      ((NS800_MOTOR_DEFAULT_OPEN_LOOP_RPM / 60.0f) * \
                                                    (float)NS800_MOTOR_DEFAULT_POLE_PAIRS)
#define NS800_MOTOR_DEFAULT_OPEN_LOOP_VOLTAGE_V    (24.0f)
#define NS800_MOTOR_DEFAULT_HVDC_PORT_VOLTAGE_V    (32.0f)
#define NS800_MOTOR_DEFAULT_LVDC_PORT_VOLTAGE_V    (24.0f)

#define NS800_MOTOR_DEFAULT_PHASE_RESISTANCE_OHM   (1.0f)
#define NS800_MOTOR_DEFAULT_TORQUE_CONSTANT_NM_A   (0.38f)
#define NS800_MOTOR_DEFAULT_FLUX_WB                (0.036f)
#define NS800_MOTOR_DEFAULT_LD_H                   (0.4e-3f)
#define NS800_MOTOR_DEFAULT_LQ_H                   (0.5e-3f)

#define NS800_MOTOR_DEFAULT_ID_REF_A               (0.0f)
#define NS800_MOTOR_DEFAULT_CURRENT_KP             (2.0f)
#define NS800_MOTOR_DEFAULT_CURRENT_KI             (800.0f)
#define NS800_MOTOR_DEFAULT_SPEED_KP               (0.02f)
#define NS800_MOTOR_DEFAULT_SPEED_KI               (2.0f)
#define NS800_MOTOR_DEFAULT_MAX_CURRENT_A          (20.0f)
#define NS800_MOTOR_DEFAULT_MAX_TORQUE_NM          (2.0f)
#define NS800_MOTOR_DEFAULT_MAX_SPEED_RAD_S        (600.0f)
#define NS800_MOTOR_DEFAULT_MAX_VOLTAGE_V          (24.0f)
#define NS800_MOTOR_DEFAULT_MIN_DC_VOLTAGE_V       (1.0f)

#define NS800_MOTOR_MA_TO_A                        (0.001f)

#define NS800_MOTOR_PWM_TBCLK_HZ                   NS800_MOTOR_EPWM_TBCLK_HZ
#define NS800_MOTOR_PWM_DEADTIME_NS                2000U
#define NS800_MOTOR_PWM_DEADTIME_TICKS             ((rt_uint16_t)(((rt_uint64_t)NS800_MOTOR_PWM_TBCLK_HZ * \
                                                                    NS800_MOTOR_PWM_DEADTIME_NS + 999999999ULL) / \
                                                                   1000000000ULL))

#ifdef __cplusplus
}
#endif

#endif /* __NS800_MOTOR_CONFIG_H__ */
