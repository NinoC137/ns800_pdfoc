/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    ns800_motor_app.c
 * @brief   电机应用层：ADC 采样接入、无感起动状态机、故障停机守护线程。
 *
 * 闭环数据通路(全部来自 ADC)：
 *
 *   ADC 相电流 + 上一拍施加电压 --> 滑模观测器 --> 电角度/转速
 *        --> 速度 PI --> 电流 PI --> 逆 Park --> 双端口 SVM --> EPWM8~13
 *
 *   同时 ADC 通道 0/2 的母线电压直接参与 SVM 占空比解算，ADC 无效时回退常量。
 *
 * 故障停机三层：
 *   1. ADC PPB -> X-BAR -> EPWM TZ(硬件，见 `ns800_protect.c`)；
 *   2. 控制 ISR 三相电流阈值 -> `EPWM_forceTripZoneEvent`(软件快路)；
 *   3. `motguard` 线程收事件 -> 停控制中断与 PWM(慢速兜底)。
 */

#include "ns800_motor_app.h"

#include <stdlib.h>

#include <board.h>

#include "epwm.h"
#include "interrupt.h"

#include "ns800_adc_background.h"
#include "ns800_adc_scale.h"
#include "ns800_button_app.h"
#include "ns800_motor_config.h"
#include "ns800_protect.h"
#include "ns800_pwm_app.h"
#include "sl_observer_app.h"

#ifdef RT_USING_FINSH
#include "finsh.h"
#endif

#define NS800_MOTOR_EPWM_BASE                 EPWM2
#define NS800_MOTOR_EPWM_IRQn                 EPWM2_INT_IRQn
#define NS800_MOTOR_WAVE_THREAD_STACK         1024U
#define NS800_MOTOR_WAVE_THREAD_PRIO          20U
#define NS800_MOTOR_WAVE_THREAD_TICK          10U
#define NS800_MOTOR_WAVE_DEFAULT_PERIOD_MS    1U
#define NS800_MOTOR_WAVE_MAX_PERIOD_MS        1000U
#define NS800_MOTOR_GUARD_THREAD_STACK        1024U
#define NS800_MOTOR_GUARD_THREAD_PRIO         8U
#define NS800_MOTOR_GUARD_THREAD_TICK         10U
#define NS800_MOTOR_GUARD_POLL_MS             10U

/** @brief 母线电压可信下限，单位 V；低于该值回退到配置常量。 */
#define NS800_MOTOR_BUS_MIN_V                 5.0f
/** @brief 连续 ADC 丢帧多少拍判为采样故障。 */
#define NS800_MOTOR_ADC_MISS_LIMIT            5U
/** @brief 观测器连续满足切换条件的拍数(20 kHz 下 400 拍 = 20 ms)。 */
#define NS800_MOTOR_LOCK_CYCLES               400U
/** @brief 闭环中观测器连续失锁拍数(1000 拍 = 50 ms)。 */
#define NS800_MOTOR_UNLOCK_CYCLES             1000U
/** @brief I/F 强拖最长等待拍数(200000 拍 = 10 s)，超时判起动失败。 */
#define NS800_MOTOR_IF_TIMEOUT_CYCLES         200000U
/** @brief 切换允许的最小转速，单位 rpm。 */
#define NS800_MOTOR_LOCK_MIN_RPM              50.0f
/** @brief 切换转速容差：比例 + 固定下限，单位 rpm。 */
#define NS800_MOTOR_LOCK_TOL_RATIO            0.10f
#define NS800_MOTOR_LOCK_TOL_MIN_RPM          30.0f
#define NS800_MOTOR_RAD_S_TO_RPM              9.5492965855f
#define NS800_MOTOR_RPM_TO_RAD_S              0.1047197551f

/** @brief motguard 事件位。 */
#define NS800_MOTOR_EVT_ADC_MISS              (1U << 0)
#define NS800_MOTOR_EVT_SAMPLE_FAULT          (1U << 1)
#define NS800_MOTOR_EVT_FEEDBACK_FAULT        (1U << 2)
#define NS800_MOTOR_EVT_LOW_DC                (1U << 3)
#define NS800_MOTOR_EVT_OVERMOD               (1U << 4)
#define NS800_MOTOR_EVT_OVERCURRENT           (1U << 5)
#define NS800_MOTOR_EVT_HW_TRIP               (1U << 6)
#define NS800_MOTOR_EVT_START_TIMEOUT         (1U << 7)
#define NS800_MOTOR_EVT_ALL                   (NS800_MOTOR_EVT_ADC_MISS | \
                                               NS800_MOTOR_EVT_SAMPLE_FAULT | \
                                               NS800_MOTOR_EVT_FEEDBACK_FAULT | \
                                               NS800_MOTOR_EVT_LOW_DC | \
                                               NS800_MOTOR_EVT_OVERMOD | \
                                               NS800_MOTOR_EVT_OVERCURRENT | \
                                               NS800_MOTOR_EVT_HW_TRIP | \
                                               NS800_MOTOR_EVT_START_TIMEOUT)
/** @brief 需要立即停机的致命事件。 */
#define NS800_MOTOR_EVT_FATAL                 (NS800_MOTOR_EVT_ADC_MISS | \
                                               NS800_MOTOR_EVT_SAMPLE_FAULT | \
                                               NS800_MOTOR_EVT_FEEDBACK_FAULT | \
                                               NS800_MOTOR_EVT_LOW_DC | \
                                               NS800_MOTOR_EVT_OVERCURRENT | \
                                               NS800_MOTOR_EVT_HW_TRIP | \
                                               NS800_MOTOR_EVT_START_TIMEOUT)

/** @brief 无感起动状态。 */
typedef enum
{
    /**< 未起动。 */
    NS800_MOTOR_START_IDLE = 0,
    /**< I/F 强拖阶段，等待观测器锁定。 */
    NS800_MOTOR_START_IF_RAMP,
    /**< 速度闭环运行中。 */
    NS800_MOTOR_START_SPEED,
} ns800_motor_start_state_t;

static ns800_motor_config_t motor_cfg;
static ns800_motor_params_t motor_params;
static ns800_motor_state_t motor_state;
static ns800_motor_app_status_t motor_status;
static volatile rt_bool_t motor_running = RT_FALSE;
static volatile rt_bool_t motor_reset_pending = RT_FALSE;
static volatile rt_bool_t motor_tripped = RT_FALSE;
static volatile ns800_motor_mode_t motor_mode = NS800_MOTOR_MODE_OPEN_LOOP;
static volatile ns800_motor_start_state_t motor_start_state = NS800_MOTOR_START_IDLE;
static rt_uint32_t motor_last_adc_seq = 0U;
static rt_uint32_t motor_last_reset_count = 0U;
static rt_uint32_t motor_adc_miss_streak = 0U;
static rt_bool_t motor_adc_seen = RT_FALSE;
static rt_uint32_t motor_lock_count = 0U;
static rt_uint32_t motor_unlock_count = 0U;
static rt_uint32_t motor_if_cycles = 0U;
static volatile rt_uint32_t motor_latched_events = 0U;
static volatile rt_uint32_t motor_event_sent = 0U;
static svm_ab_f32_t motor_prev_voltage_ab;
static rt_thread_t motor_wave_thread = RT_NULL;
static volatile rt_bool_t motor_wave_enabled = RT_FALSE;
static volatile rt_uint32_t motor_wave_period_ms = NS800_MOTOR_WAVE_DEFAULT_PERIOD_MS;
static struct rt_event motor_event;
static rt_thread_t motor_guard_thread = RT_NULL;
static rt_bool_t motor_event_ready = RT_FALSE;
static rt_bool_t motor_guard_started = RT_FALSE;

/**
 * @brief 获取转子机械角和机械角速度的默认弱实现。
 *
 * 本工程由 `ns800_motor_feedback.c` 提供强符号覆盖(无感观测器)。接入编码器/
 * 霍尔时也可以换成该文件的实现，接口保持不变。
 *
 * @param theta_m_rad 输出机械角，单位 rad。
 * @param omega_m_rad_s 输出机械角速度，单位 rad/s。
 * @return RT_FALSE 表示反馈无效。
 */
rt_bool_t __attribute__((weak)) ns800_motor_feedback_get(float *theta_m_rad, float *omega_m_rad_s)
{
    if (theta_m_rad != RT_NULL)
    {
        *theta_m_rad = 0.0f;
    }
    if (omega_m_rad_s != RT_NULL)
    {
        *omega_m_rad_s = 0.0f;
    }

    return RT_FALSE;
}

/**
 * @brief 计算 float 绝对值。
 *
 * @param value 输入值。
 * @return 非负绝对值。
 */
static float motor_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 将输入值裁剪到闭区间。
 *
 * @param value 输入值。
 * @param min_value 下限。
 * @param max_value 上限。
 * @return 裁剪后的值。
 */
static float motor_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

static int motor_float_to_i32(float value)
{
    if (value >= 0.0f)
    {
        return (int)(value + 0.5f);
    }

    return (int)(value - 0.5f);
}

static int motor_milli(float value)
{
    return motor_float_to_i32(value * 1000.0f);
}

static rt_uint32_t motor_wave_clamp_period(rt_uint32_t period_ms)
{
    if (period_ms == 0U)
    {
        return NS800_MOTOR_WAVE_DEFAULT_PERIOD_MS;
    }
    if (period_ms > NS800_MOTOR_WAVE_MAX_PERIOD_MS)
    {
        return NS800_MOTOR_WAVE_MAX_PERIOD_MS;
    }

    return period_ms;
}

static void motor_wave_snapshot(svm_dual_out_t *duty, float *hvdc_v, float *lvdc_v, rt_bool_t *running)
{
    if ((duty == RT_NULL) || (hvdc_v == RT_NULL) || (lvdc_v == RT_NULL) || (running == RT_NULL))
    {
        return;
    }

    rt_enter_critical();
    *duty = motor_status.last_output.duty;
    if (motor_status.measured_hvdc_v > (motor_status.measured_lvdc_v + NS800_MOTOR_BUS_MIN_V))
    {
        *hvdc_v = motor_status.measured_hvdc_v;
        *lvdc_v = motor_status.measured_lvdc_v;
    }
    else
    {
        *hvdc_v = motor_cfg.hvdc_port_voltage_v;
        *lvdc_v = motor_cfg.lvdc_port_voltage_v;
    }
    *running = motor_running;
    rt_exit_critical();
}

static float motor_svm_upper_voltage(float hvdc_v, float lvdc_v)
{
    return hvdc_v - lvdc_v;
}

static void motor_wave_thread_entry(void *parameter)
{
    svm_dual_out_t duty;
    float hvdc_v;
    float lvdc_v;
    float upper_v;
    float va_f;
    float vb_f;
    float vc_f;
    int vab;
    int vbc;
    int vca;

    RT_UNUSED(parameter);

    while (1)
    {
        rt_bool_t running = RT_FALSE;

        if (motor_wave_enabled == RT_TRUE)
        {
            motor_wave_snapshot(&duty, &hvdc_v, &lvdc_v, &running);
            if ((running == RT_TRUE) && (hvdc_v > lvdc_v))
            {
                upper_v = motor_svm_upper_voltage(hvdc_v, lvdc_v);
                va_f = duty.upper.ta * upper_v + duty.lower.ta * lvdc_v;
                vb_f = duty.upper.tb * upper_v + duty.lower.tb * lvdc_v;
                vc_f = duty.upper.tc * upper_v + duty.lower.tc * lvdc_v;

                vab = motor_float_to_i32((va_f - vb_f) * 1000.0f);
                vbc = motor_float_to_i32((vb_f - vc_f) * 1000.0f);
                vca = motor_float_to_i32((vc_f - va_f) * 1000.0f);
                rt_kprintf("wave:%d,%d,%d\r\n", vab, vbc, vca);
            }
        }

        rt_thread_mdelay(motor_wave_period_ms);
    }
}

static int motor_wave_thread_start(void)
{
    if (motor_wave_thread != RT_NULL)
    {
        return 0;
    }

    motor_wave_thread = rt_thread_create("m_wave",
                                         motor_wave_thread_entry,
                                         RT_NULL,
                                         NS800_MOTOR_WAVE_THREAD_STACK,
                                         NS800_MOTOR_WAVE_THREAD_PRIO,
                                         NS800_MOTOR_WAVE_THREAD_TICK);
    if (motor_wave_thread == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_thread_startup(motor_wave_thread);
    return 0;
}

/**
 * @brief 初始化 EPWM2 为电机控制定时中断。
 *
 * EPWM2 只作为控制节拍，不输出 PWM；功率 PWM 仍由 EPWM8~13 负责。
 */
static void ns800_motor_epwm_init(void)
{
    EPWM_setClockPrescaler(NS800_MOTOR_EPWM_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(NS800_MOTOR_EPWM_BASE, NS800_MOTOR_CONTROL_TBPRD);
    EPWM_setTimeBaseCounter(NS800_MOTOR_EPWM_BASE, 0U);
    EPWM_setTimeBaseCounterMode(NS800_MOTOR_EPWM_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(NS800_MOTOR_EPWM_BASE);
    EPWM_setPhaseShift(NS800_MOTOR_EPWM_BASE, 0U);
    EPWM_disableInterrupt(NS800_MOTOR_EPWM_BASE);
    EPWM_setInterruptSource(NS800_MOTOR_EPWM_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_setInterruptEventCount(NS800_MOTOR_EPWM_BASE, 1U);
    EPWM_clearEventTriggerInterruptFlag(NS800_MOTOR_EPWM_BASE);
    EPWM_enableInterrupt(NS800_MOTOR_EPWM_BASE);
}

/**
 * @brief 组装一拍电机控制输入。
 *
 * 该函数运行在 EPWM2 ISR 路径中，只做无阻塞数据读取和简单换算：
 * ADC CH5/7/9 映射为 A/B/C 三相电流，CH0/CH2 映射为高压/低压母线电压，
 * 按键全局变量映射为 xi、speed、Iq 限幅。
 *
 * @param in 输出控制输入结构。
 */
static void ns800_motor_fill_input(ns800_motor_input_t *in)
{
    const rt_uint16_t *frame;
    rt_uint32_t seq;
    rt_uint32_t reset_count;

    frame = ns800_adc_background_latest(&seq);
    if ((frame == RT_NULL) || (seq == motor_last_adc_seq))
    {
        motor_status.adc_miss_count++;
        if (motor_adc_miss_streak < 0xFFFFU)
        {
            motor_adc_miss_streak++;
        }
        in->sample.adc_valid = RT_FALSE;
    }
    else
    {
        motor_last_adc_seq = seq;
        motor_status.adc_seq = seq;
        motor_adc_miss_streak = 0U;
        motor_adc_seen = RT_TRUE;
        in->sample.adc_valid = RT_TRUE;
    }

    in->sample.bus_valid = RT_FALSE;
    if (frame != RT_NULL)
    {
        float hvdc_v = ns800_adc_dc_voltage(frame[0]);
        float lvdc_v = ns800_adc_dc_voltage(frame[2]);

        motor_status.measured_hvdc_v = hvdc_v;
        motor_status.measured_lvdc_v = lvdc_v;

        /* 只有两路母线都可信、且高压侧确实高于低压侧时才用实测值。 */
        if ((hvdc_v > NS800_MOTOR_BUS_MIN_V) &&
            (lvdc_v > NS800_MOTOR_BUS_MIN_V) &&
            (hvdc_v > (lvdc_v + NS800_MOTOR_BUS_MIN_V)))
        {
            in->sample.hvdc_v = hvdc_v;
            in->sample.lvdc_v = lvdc_v;
            in->sample.bus_valid = RT_TRUE;
        }
    }

    if (frame != RT_NULL)
    {
        in->sample.phase_current_abc.a = ns800_adc_ac_current(frame[5]);
        in->sample.phase_current_abc.b = ns800_adc_ac_current(frame[7]);
        in->sample.phase_current_abc.c = ns800_adc_ac_current(frame[9]);
    }
    else
    {
        in->sample.phase_current_abc.a = 0.0f;
        in->sample.phase_current_abc.b = 0.0f;
        in->sample.phase_current_abc.c = 0.0f;
    }

    reset_count = ns800_button_app_reset_count();
    in->command.reset = (reset_count != motor_last_reset_count) || (motor_reset_pending == RT_TRUE);
    motor_last_reset_count = reset_count;
    motor_reset_pending = RT_FALSE;

    in->command.mode = motor_mode;
    in->command.xi = motor_clamp(ns800_param_xi, 0.0f, 1.0f);
    in->command.speed_ref_rpm = (float)ns800_param_speed;
    in->command.iq_limit_a = motor_abs((float)ns800_param_force_ma) * NS800_MOTOR_MA_TO_A;
    in->command.iq_ref_a = (float)ns800_param_force_ma * NS800_MOTOR_MA_TO_A;
}

/**
 * @brief 用观测器(或编码器)填充本拍转子反馈。
 *
 * @param in 单拍输入，feedback 字段被写入。
 */
static void ns800_motor_fill_feedback(ns800_motor_input_t *in)
{
    float theta_m = 0.0f;
    float omega_m = 0.0f;

    in->sample.feedback_valid = ns800_motor_feedback_get(&theta_m, &omega_m);
    if (in->sample.feedback_valid != RT_TRUE)
    {
        if ((in->command.mode == NS800_MOTOR_MODE_SPEED) ||
            (in->command.mode == NS800_MOTOR_MODE_TORQUE))
        {
            motor_status.feedback_miss_count++;
        }
        theta_m = 0.0f;
        omega_m = 0.0f;
    }

    in->sample.theta_m_rad = theta_m;
    in->sample.omega_m_rad_s = omega_m;
}

/**
 * @brief 无感起动状态机：I/F 强拖 -> 观测器锁定 -> 速度闭环。
 *
 * 在控制 ISR 内、每次 `ns800_motor_step` 之前调用，可直接改写本拍命令模式。
 *
 * @param in 单拍输入/输出，切换成功时把 command.mode 改成 SPEED。
 * @return 本拍需要上送 motguard 的事件位。
 */
static rt_uint32_t ns800_motor_start_state_step(ns800_motor_input_t *in)
{
    const sl_observer_output_t *obs = sl_observer_app_get_output();
    rt_uint32_t events = 0U;
    float ref_rpm;
    float ramp_rpm;
    float tolerance;
    rt_bool_t obs_valid;

    obs_valid = ((obs != RT_NULL) && (obs->valid == true)) ? RT_TRUE : RT_FALSE;

    if (motor_start_state == NS800_MOTOR_START_IF_RAMP)
    {
        ref_rpm = motor_abs(in->command.speed_ref_rpm);
        ramp_rpm = motor_state.if_ramp_rpm;
        tolerance = (NS800_MOTOR_LOCK_TOL_RATIO * ref_rpm) + NS800_MOTOR_LOCK_TOL_MIN_RPM;

        motor_if_cycles++;
        if (motor_if_cycles >= NS800_MOTOR_IF_TIMEOUT_CYCLES)
        {
            /* 强拖超时：电流一直给着但观测器不锁定，判起动失败并停机。 */
            events |= NS800_MOTOR_EVT_START_TIMEOUT;
        }

        if ((obs_valid == RT_TRUE) &&
            (ref_rpm > NS800_MOTOR_LOCK_MIN_RPM) &&
            (motor_abs(obs->speed_rpm - ramp_rpm) <= tolerance))
        {
            motor_lock_count++;
            if (motor_lock_count >= NS800_MOTOR_LOCK_CYCLES)
            {
                /* 无扰切换：让速度 PI 的初始输出等于当前 I/F 力矩指令。 */
                float error = (in->command.speed_ref_rpm * NS800_MOTOR_RPM_TO_RAD_S) -
                              obs->omega_m_rad_s;
                float torque = motor_params.torque_constant_nm_a *
                               motor_status.last_output.current_ref_dq.q;

                ns800_motor_speed_preload(&motor_state, &motor_cfg, torque, error);

                motor_mode = NS800_MOTOR_MODE_SPEED;
                motor_start_state = NS800_MOTOR_START_SPEED;
                motor_lock_count = 0U;
                motor_unlock_count = 0U;
                motor_if_cycles = 0U;
                motor_reset_pending = RT_FALSE;
            }
        }
        else
        {
            motor_lock_count = 0U;
        }
    }
    else if (motor_start_state == NS800_MOTOR_START_SPEED)
    {
        if (obs_valid != RT_TRUE)
        {
            motor_unlock_count++;
            if (motor_unlock_count >= NS800_MOTOR_UNLOCK_CYCLES)
            {
                events |= NS800_MOTOR_EVT_FEEDBACK_FAULT;
            }
        }
        else
        {
            motor_unlock_count = 0U;
        }
    }
    else
    {
        /* IDLE：不干预。 */
    }

    motor_status.lock_count = motor_lock_count;
    motor_status.start_state = (rt_uint32_t)motor_start_state;
    in->command.mode = motor_mode;

    return events;
}

/**
 * @brief 把控制状态位翻译成 motguard 事件位。
 *
 * @param status 本拍状态位。
 * @return 事件位。
 */
static rt_uint32_t ns800_motor_events_from_status(rt_uint32_t status)
{
    rt_uint32_t events = 0U;

    if ((status & (rt_uint32_t)NS800_MOTOR_STATUS_SAMPLE_FAULT) != 0U)
    {
        events |= NS800_MOTOR_EVT_SAMPLE_FAULT;
    }
    if ((status & (rt_uint32_t)NS800_MOTOR_STATUS_FEEDBACK_FAULT) != 0U)
    {
        events |= NS800_MOTOR_EVT_FEEDBACK_FAULT;
    }
    if ((status & (rt_uint32_t)NS800_MOTOR_STATUS_LOW_DC_VOLTAGE) != 0U)
    {
        events |= NS800_MOTOR_EVT_LOW_DC;
    }
    if ((status & (rt_uint32_t)NS800_MOTOR_STATUS_OVERMODULATION) != 0U)
    {
        events |= NS800_MOTOR_EVT_OVERMOD;
    }

    return events;
}

/**
 * @brief 向 motguard 上送事件(去重，避免 ISR 每拍重复发送)。
 *
 * @param events 事件位。
 */
static void ns800_motor_post_events(rt_uint32_t events)
{
    rt_uint32_t fresh;

    if ((events == 0U) || (motor_event_ready != RT_TRUE))
    {
        return;
    }

    fresh = events & ~motor_event_sent;
    if (fresh == 0U)
    {
        return;
    }

    motor_event_sent |= fresh;
    (void)rt_event_send(&motor_event, fresh);
}

/**
 * @brief EPWM2 电机控制中断服务函数。
 *
 * 顺序很关键：先喂观测器(上一拍电压 + 本拍电流)，再取反馈、跑控制、写 PWM。
 * 中断内禁止 printf、动态内存和任何可能阻塞的操作。
 */
void ns800_motor_isr(void)
{
    ns800_motor_input_t in;
    ns800_motor_status_t status;
    rt_uint32_t events = 0U;
    const sl_observer_output_t *obs;

    if (motor_running == RT_FALSE)
    {
        EPWM_clearEventTriggerInterruptFlag(NS800_MOTOR_EPWM_BASE);
        __DSB();
        return;
    }

    motor_status.isr_count++;
    ns800_motor_fill_input(&in);

    /* 观测器单拍更新：用上一拍实际施加的电压矢量和本拍 ADC 电流。 */
    sl_observer_app_update(&motor_prev_voltage_ab, &in.sample.phase_current_abc);

    ns800_motor_fill_feedback(&in);
    events |= ns800_motor_start_state_step(&in);

    status = ns800_motor_step(&motor_state, &motor_cfg, &motor_params, &in, &motor_status.last_output);
    motor_status.fault_flags = motor_status.last_output.status;
    motor_status.mode = motor_mode;

    obs = sl_observer_app_get_output();
    if (obs != RT_NULL)
    {
        motor_status.observer_speed_rpm = obs->speed_rpm;
        motor_status.observer_valid = (obs->valid == true) ? RT_TRUE : RT_FALSE;
    }

    ns800_pwm_app_write_svm(&motor_status.last_output.duty);
    motor_prev_voltage_ab = motor_status.last_output.voltage_cmd_ab;

    /* 软件快路：三相电流超限直接 force trip。 */
    if (ns800_protect_check_currents(in.sample.phase_current_abc.a,
                                     in.sample.phase_current_abc.b,
                                     in.sample.phase_current_abc.c) == RT_TRUE)
    {
        events |= (NS800_MOTOR_EVT_OVERCURRENT | NS800_MOTOR_EVT_HW_TRIP);
    }
    if (ns800_pwm_app_trip_active() == RT_TRUE)
    {
        events |= NS800_MOTOR_EVT_HW_TRIP;
    }

    /* 只有确认 ADC 曾经出过帧之后，连续丢帧才判故障，避免上电时序误停机。 */
    if ((motor_adc_seen == RT_TRUE) &&
        (motor_adc_miss_streak >= NS800_MOTOR_ADC_MISS_LIMIT))
    {
        events |= NS800_MOTOR_EVT_ADC_MISS;
    }

    events |= ns800_motor_events_from_status((rt_uint32_t)status);
    ns800_motor_post_events(events);

    RT_UNUSED(status);
    EPWM_clearEventTriggerInterruptFlag(NS800_MOTOR_EPWM_BASE);
    __DSB();
}

/**
 * @brief motguard 线程入口：事件 -> 停机 -> 锁存。
 *
 * 硬件 TZ 跳闸不产生事件，因此线程同时以 10 ms 周期轮询 EPWM 的 trip 标志。
 */
static void motor_guard_thread_entry(void *parameter)
{
    rt_uint32_t recved;

    RT_UNUSED(parameter);

    while (1)
    {
        recved = 0U;
        if (rt_event_recv(&motor_event,
                          NS800_MOTOR_EVT_ALL,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          rt_tick_from_millisecond(NS800_MOTOR_GUARD_POLL_MS),
                          &recved) != RT_EOK)
        {
            recved = 0U;
        }

        if (ns800_protect_is_tripped() == RT_TRUE)
        {
            recved |= NS800_MOTOR_EVT_HW_TRIP;
        }

        if (recved == 0U)
        {
            continue;
        }

        motor_latched_events |= recved;
        motor_status.guard_fault_flags = motor_latched_events;
        motor_status.protect_flags = ns800_protect_get_status()->flags;

        if ((recved & NS800_MOTOR_EVT_FATAL) == 0U)
        {
            continue;
        }
        if (motor_tripped == RT_TRUE)
        {
            continue;
        }

        motor_tripped = RT_TRUE;
        motor_status.tripped = RT_TRUE;
        motor_status.guard_action_tick = (rt_uint32_t)rt_tick_get();

        rt_kprintf("motguard: trip evt=0x%04x status=0x%08x protect=0x%02x isr=%u seq=%u\r\n",
                   (unsigned int)recved,
                   (unsigned int)motor_status.fault_flags,
                   (unsigned int)motor_status.protect_flags,
                   (unsigned int)motor_status.isr_count,
                   (unsigned int)motor_status.adc_seq);

        (void)ns800_motor_app_stop();
        motor_status.tripped = RT_TRUE;
    }
}

static int motor_guard_thread_start(void)
{
    if (motor_guard_started == RT_TRUE)
    {
        return 0;
    }

    if (motor_event_ready != RT_TRUE)
    {
        if (rt_event_init(&motor_event, "mot_evt", RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            return -RT_ERROR;
        }
        motor_event_ready = RT_TRUE;
    }

    motor_guard_thread = rt_thread_create("motguard",
                                          motor_guard_thread_entry,
                                          RT_NULL,
                                          NS800_MOTOR_GUARD_THREAD_STACK,
                                          NS800_MOTOR_GUARD_THREAD_PRIO,
                                          NS800_MOTOR_GUARD_THREAD_TICK);
    if (motor_guard_thread == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_thread_startup(motor_guard_thread);
    motor_guard_started = RT_TRUE;
    return 0;
}

/**
 * @brief 用实际电机参数与真实控制周期重配无感观测器。
 */
static void ns800_motor_observer_config(void)
{
    float ls_h = 0.5f * (motor_params.ld_h + motor_params.lq_h);

    (void)sl_observer_app_config(motor_params.phase_resistance_ohm,
                                 ls_h,
                                 motor_params.flux_wb,
                                 motor_params.pole_pairs,
                                 motor_cfg.sample_time_s);
}

/**
 * @brief 启动电机应用层。
 *
 * 默认进入开环模式，便于先验证 EPWM8~13 和功率分配 SVM 输出链路；随后可
 * `motor_speed` 触发 I/F 起动 -> 无感速度闭环。
 *
 * @return 0 表示成功。
 */
int ns800_motor_app_start(void)
{
    rt_memset((void *)&motor_status, 0, sizeof(motor_status));
    motor_last_adc_seq = 0U;
    motor_last_reset_count = ns800_button_app_reset_count();
    motor_adc_miss_streak = 0U;
    motor_lock_count = 0U;
    motor_unlock_count = 0U;
    motor_latched_events = 0U;
    motor_event_sent = 0U;
    motor_prev_voltage_ab.alpha = 0.0f;
    motor_prev_voltage_ab.beta = 0.0f;
    motor_mode = NS800_MOTOR_MODE_OPEN_LOOP;
    motor_start_state = NS800_MOTOR_START_IDLE;
    motor_tripped = RT_FALSE;
    motor_running = RT_FALSE;
    motor_reset_pending = RT_TRUE;

    ns800_motor_default_config(&motor_cfg);
    ns800_motor_default_params(&motor_params);
    ns800_motor_init(&motor_state, &motor_cfg);
    ns800_motor_observer_config();

    ns800_motor_epwm_init();
    Interrupt_register(NS800_MOTOR_EPWM_IRQn, &ns800_motor_isr);
    Interrupt_enable(NS800_MOTOR_EPWM_IRQn);

    (void)motor_guard_thread_start();
    (void)ns800_protect_init();

    motor_status.mode = motor_mode;
    motor_status.start_state = (rt_uint32_t)motor_start_state;
    motor_status.tripped = RT_FALSE;

    /* 若保护链在上电自检时已经跳闸(例如 PPB 通道映射不符)，保持输出封锁。 */
    if (ns800_protect_is_tripped() == RT_TRUE)
    {
        motor_tripped = RT_TRUE;
        motor_status.tripped = RT_TRUE;
        motor_running = RT_FALSE;
        rt_kprintf("motor: protection tripped at startup, outputs locked (motor_clear to reset)\r\n");
    }
    else
    {
        motor_running = RT_TRUE;
    }

    motor_wave_thread_start();
    return 0;
}

/**
 * @brief 停止电机应用层并关闭 PWM 输出。
 *
 * @return 0 表示成功。
 */
int ns800_motor_app_stop(void)
{
    motor_running = RT_FALSE;
    motor_mode = NS800_MOTOR_MODE_STOP;
    motor_start_state = NS800_MOTOR_START_IDLE;
    motor_lock_count = 0U;
    motor_unlock_count = 0U;
    Interrupt_disable(NS800_MOTOR_EPWM_IRQn);
    EPWM_disableInterrupt(NS800_MOTOR_EPWM_BASE);
    EPWM_setTimeBaseCounterMode(NS800_MOTOR_EPWM_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
    ns800_pwm_app_stop();
    motor_status.mode = motor_mode;
    motor_status.start_state = (rt_uint32_t)motor_start_state;

    return 0;
}

/**
 * @brief 切换电机控制模式。
 *
 * 切入速度闭环前会检查反馈接口是否有效，无反馈时拒绝切换。
 *
 * @param mode 目标模式。
 * @return 0 表示切换成功；负值表示切换失败。
 */
int ns800_motor_app_set_mode(ns800_motor_mode_t mode)
{
    float theta_m;
    float omega_m;

    if (mode == NS800_MOTOR_MODE_SPEED)
    {
        if (ns800_motor_feedback_get(&theta_m, &omega_m) == RT_FALSE)
        {
            motor_status.fault_flags |= (rt_uint32_t)NS800_MOTOR_STATUS_FEEDBACK_FAULT;
            return -RT_ERROR;
        }
    }

    motor_mode = mode;
    if (mode == NS800_MOTOR_MODE_SPEED)
    {
        motor_start_state = NS800_MOTOR_START_SPEED;
    }
    else if (mode != NS800_MOTOR_MODE_IF_START)
    {
        motor_start_state = NS800_MOTOR_START_IDLE;
    }
    motor_reset_pending = RT_TRUE;
    return 0;
}

int ns800_motor_app_start_speed(void)
{
    float theta_m;
    float omega_m;

    if (motor_tripped == RT_TRUE)
    {
        return -RT_ERROR;
    }

    if (motor_mode == NS800_MOTOR_MODE_SPEED)
    {
        return 0;
    }

    motor_reset_pending = RT_TRUE;
    motor_lock_count = 0U;
    motor_unlock_count = 0U;
    motor_if_cycles = 0U;

    if (ns800_motor_feedback_get(&theta_m, &omega_m) == RT_TRUE)
    {
        motor_mode = NS800_MOTOR_MODE_SPEED;
        motor_start_state = NS800_MOTOR_START_SPEED;
    }
    else
    {
        motor_state.if_ramp_rpm = 0.0f;
        motor_mode = NS800_MOTOR_MODE_IF_START;
        motor_start_state = NS800_MOTOR_START_IF_RAMP;
    }

    return 0;
}

int ns800_motor_app_clear_trip(void)
{
    if ((motor_tripped == RT_FALSE) &&
        (ns800_protect_is_tripped() == RT_FALSE) &&
        (motor_running == RT_TRUE))
    {
        return 0;
    }

    ns800_protect_clear();
    ns800_pwm_app_resume();

    ns800_motor_reset(&motor_state);
    motor_mode = NS800_MOTOR_MODE_STOP;
    motor_start_state = NS800_MOTOR_START_IDLE;
    motor_reset_pending = RT_TRUE;
    motor_latched_events = 0U;
    motor_event_sent = 0U;
    motor_adc_miss_streak = 0U;
    motor_lock_count = 0U;
    motor_unlock_count = 0U;
    motor_if_cycles = 0U;
    motor_prev_voltage_ab.alpha = 0.0f;
    motor_prev_voltage_ab.beta = 0.0f;
    motor_status.guard_fault_flags = 0U;
    motor_status.guard_action_tick = 0U;
    motor_status.protect_flags = 0U;
    motor_status.fault_flags = 0U;
    motor_status.tripped = RT_FALSE;

    ns800_motor_observer_config();

    EPWM_setTimeBaseCounterMode(NS800_MOTOR_EPWM_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enableInterrupt(NS800_MOTOR_EPWM_BASE);
    Interrupt_enable(NS800_MOTOR_EPWM_IRQn);

    motor_status.mode = motor_mode;
    motor_status.start_state = (rt_uint32_t)motor_start_state;
    motor_tripped = RT_FALSE;
    motor_running = RT_TRUE;

    return 0;
}

rt_bool_t ns800_motor_app_is_tripped(void)
{
    return motor_tripped;
}

/**
 * @brief 获取电机应用层状态。
 *
 * @return 状态结构只读指针。
 */
const ns800_motor_app_status_t *ns800_motor_app_get_status(void)
{
    return &motor_status;
}

#ifdef RT_USING_FINSH
/**
 * @brief FinSH 命令：切换到开环电机测试模式。
 */
static int motor_ol_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    ns800_motor_app_set_mode(NS800_MOTOR_MODE_OPEN_LOOP);
    rt_kprintf("motor open-loop: 24V peak, %d rpm, hv/lv from ADC when valid\r\n",
               (int)NS800_MOTOR_DEFAULT_OPEN_LOOP_RPM);
    return 0;
}

/**
 * @brief FinSH 命令：启动无感速度闭环(I/F -> 观测器锁定 -> SPEED)。
 */
static int motor_speed_cmd(int argc, char **argv)
{
    int result;

    RT_UNUSED(argc);
    RT_UNUSED(argv);

    if (ns800_param_speed <= 0)
    {
        rt_kprintf("motor speed: speed param must be > 0 (use motor_status to check)\r\n");
        return -RT_EINVAL;
    }

    result = ns800_motor_app_start_speed();
    if (result != 0)
    {
        rt_kprintf("motor speed: tripped, run motor_clear first\r\n");
        return result;
    }

    rt_kprintf("motor sensorless start: target=%d rpm iq=%d mA xi=%d/1000\r\n",
               (int)ns800_param_speed,
               (int)ns800_param_force_ma,
               (int)(ns800_param_xi * 1000.0f));
    return 0;
}

/**
 * @brief FinSH 命令：切换到停止模式。
 */
static int motor_stop_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    ns800_motor_app_set_mode(NS800_MOTOR_MODE_STOP);
    rt_kprintf("motor stop\r\n");
    return 0;
}

/**
 * @brief FinSH 命令：清除故障锁存并恢复 PWM/控制中断。
 */
static int motor_clear_cmd(int argc, char **argv)
{
    int result;

    RT_UNUSED(argc);
    RT_UNUSED(argv);

    result = ns800_motor_app_clear_trip();
    rt_kprintf("motor clear: result=%d tripped=%d\r\n",
               result,
               (int)ns800_motor_app_is_tripped());
    return result;
}

/**
 * @brief FinSH 命令：打印最近一拍电机控制状态。
 */
static int motor_status_cmd(int argc, char **argv)
{
    const ns800_motor_app_status_t *status = ns800_motor_app_get_status();
    const ns800_motor_output_t *out = &status->last_output;
    const ns800_protect_status_t *protect = ns800_protect_get_status();

    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("motor: mode=%u start=%u isr=%u adc_miss=%u fb_miss=%u fault=0x%08x seq=%u trip=%d\r\n",
               (unsigned int)status->mode,
               (unsigned int)status->start_state,
               (unsigned int)status->isr_count,
               (unsigned int)status->adc_miss_count,
               (unsigned int)status->feedback_miss_count,
               (unsigned int)status->fault_flags,
               (unsigned int)status->adc_seq,
               (int)status->tripped);
    rt_kprintf("obs: valid=%d rpm=%d lock=%u guard=0x%04x protect=0x%02x hw_cnt=%u sw_cnt=%u\r\n",
               (int)status->observer_valid,
               motor_float_to_i32(status->observer_speed_rpm),
               (unsigned int)status->lock_count,
               (unsigned int)status->guard_fault_flags,
               (unsigned int)protect->flags,
               (unsigned int)protect->hw_trip_count,
               (unsigned int)protect->sw_trip_count);
    rt_kprintf("bus: hv=%dmV lv=%dmV measured, window=[%u,%u]\r\n",
               motor_milli(status->measured_hvdc_v),
               motor_milli(status->measured_lvdc_v),
               (unsigned int)protect->ppb_low_counts,
               (unsigned int)protect->ppb_high_counts);
    rt_kprintf("cmd: xi=%d/1000 speed=%d rpm F=%d mA theta=%d mrad\r\n",
               (int)(out->xi * 1000.0f),
               (int)ns800_param_speed,
               (int)ns800_param_force_ma,
               motor_milli(out->theta_e_rad));
    rt_kprintf("loop: iq_ref=%dmA iq=%dmA omega=%dmrad/s\r\n",
               motor_milli(out->current_ref_dq.q),
               motor_milli(out->current_dq.q),
               motor_milli(out->omega_m_rad_s));
    rt_kprintf("duty: ua=%d ub=%d uc=%d la=%d lb=%d lc=%d permille\r\n",
               (int)(out->duty.upper.ta * 1000.0f),
               (int)(out->duty.upper.tb * 1000.0f),
               (int)(out->duty.upper.tc * 1000.0f),
               (int)(out->duty.lower.ta * 1000.0f),
               (int)(out->duty.lower.tb * 1000.0f),
               (int)(out->duty.lower.tc * 1000.0f));
    rt_kprintf("output power=%d.%02d W\r\n",
               (int)out->output_power_w,
               (int)(motor_abs(out->output_power_w - (float)((int)out->output_power_w)) * 100.0f));

    return 0;
}

/**
 * @brief FinSH 命令：按 WaveView 协议输出三相线电压。
 */
static int motor_wave_cmd(int argc, char **argv)
{
    rt_uint32_t period_ms = motor_wave_period_ms;

    if ((argc >= 2) &&
        ((rt_strcmp(argv[1], "off") == 0) || (rt_strcmp(argv[1], "stop") == 0) ||
         (rt_strcmp(argv[1], "0") == 0)))
    {
        motor_wave_enabled = RT_FALSE;
        rt_kprintf("motor_wave: off\r\n");
        return 0;
    }

    if ((argc >= 3) &&
        ((rt_strcmp(argv[1], "on") == 0) || (rt_strcmp(argv[1], "start") == 0)))
    {
        period_ms = (rt_uint32_t)strtoul(argv[2], RT_NULL, 0);
    }
    else if ((argc >= 2) &&
             ((rt_strcmp(argv[1], "on") != 0) && (rt_strcmp(argv[1], "start") != 0)))
    {
        period_ms = (rt_uint32_t)strtoul(argv[1], RT_NULL, 0);
    }

    motor_wave_period_ms = motor_wave_clamp_period(period_ms);
    if (motor_wave_thread_start() != 0)
    {
        rt_kprintf("motor_wave: thread create failed\r\n");
        return -RT_ERROR;
    }

    motor_wave_enabled = RT_TRUE;
    rt_kprintf("motor_wave: on period=%u ms format=wave:vab,vbc,vca mV\r\n",
               (unsigned int)motor_wave_period_ms);
    return 0;
}

MSH_CMD_EXPORT_ALIAS(motor_ol_cmd, motor_ol, start ns800 motor open-loop test);
MSH_CMD_EXPORT_ALIAS(motor_speed_cmd, motor_speed, start sensorless closed-loop speed control);
MSH_CMD_EXPORT_ALIAS(motor_stop_cmd, motor_stop, stop ns800 motor pwm);
MSH_CMD_EXPORT_ALIAS(motor_clear_cmd, motor_clear, clear latched motor trip and resume pwm);
MSH_CMD_EXPORT_ALIAS(motor_status_cmd, motor_status, show ns800 motor status);
MSH_CMD_EXPORT_ALIAS(motor_wave_cmd, motor_wave, stream motor line voltage to WaveView);
#endif
