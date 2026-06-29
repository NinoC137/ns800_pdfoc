/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_motor_app.h"

#include <stdlib.h>

#include <board.h>

#include "epwm.h"
#include "interrupt.h"

#include "ns800_adc_background.h"
#include "ns800_button_app.h"
#include "ns800_pwm_app.h"

#ifdef RT_USING_FINSH
#include "finsh.h"
#endif

#define NS800_MOTOR_EPWM_BASE                 EPWM2
#define NS800_MOTOR_EPWM_IRQn                 EPWM2_INT_IRQn
#define NS800_MOTOR_CONTROL_FREQ_HZ           10000U
#define NS800_MOTOR_CONTROL_TBPRD             20000U
#define NS800_MOTOR_ADC_CURRENT_ZERO          (2048.0f)
#define NS800_MOTOR_ADC_CURRENT_GAIN_A_COUNT  (0.001f)
#define NS800_MOTOR_MA_TO_A                   (0.001f)
#define NS800_MOTOR_WAVE_THREAD_STACK         1536U
#define NS800_MOTOR_WAVE_THREAD_PRIO          20U
#define NS800_MOTOR_WAVE_THREAD_TICK          10U
#define NS800_MOTOR_WAVE_DEFAULT_RATE_HZ      50U
#define NS800_MOTOR_WAVE_MAX_RATE_HZ          100U
#define NS800_MOTOR_WAVE_MV_PER_V             1000.0f
#define NS800_MOTOR_RAD_TO_DEG                (57.2957795131f)
#define NS800_MOTOR_PI_OVER_4                 (0.7853981634f)

typedef struct
{
    rt_uint32_t seq;
    ns800_motor_output_t output;
} ns800_motor_wave_snapshot_t;

static ns800_motor_config_t motor_cfg;
static ns800_motor_params_t motor_params;
static ns800_motor_state_t motor_state;
static ns800_motor_app_status_t motor_status;
static volatile rt_bool_t motor_running = RT_FALSE;
static volatile rt_bool_t motor_reset_pending = RT_FALSE;
static volatile ns800_motor_mode_t motor_mode = NS800_MOTOR_MODE_OPEN_LOOP;
static rt_uint32_t motor_last_adc_seq = 0U;
static rt_uint32_t motor_last_reset_count = 0U;
static volatile rt_bool_t motor_wave_enable = RT_FALSE;
static volatile rt_uint32_t motor_wave_rate_hz = NS800_MOTOR_WAVE_DEFAULT_RATE_HZ;
static volatile rt_uint32_t motor_wave_decim = NS800_MOTOR_CONTROL_FREQ_HZ / NS800_MOTOR_WAVE_DEFAULT_RATE_HZ;
static volatile rt_bool_t motor_wave_snapshot_ready = RT_FALSE;
static rt_thread_t motor_wave_thread = RT_NULL;
static ns800_motor_wave_snapshot_t motor_wave_snapshot;

/**
 * @brief 获取转子机械角和机械角速度的默认弱实现。
 *
 * 当前项目还没有接入编码器/霍尔反馈，因此默认返回无效。后续新增真实反馈时，
 * 在应用层提供同名非 weak 函数即可覆盖该实现。
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

/**
 * @brief 将浮点值转换为四舍五入整数。
 *
 * @param value 输入值。
 * @return 四舍五入后的 int。
 */
static int motor_round_to_int(float value)
{
    return (int)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

/**
 * @brief 调试遥测用平方根近似，只在线程上下文使用。
 *
 * @param value 非负输入。
 * @return 平方根近似值。
 */
static float motor_sqrt_approx(float value)
{
    float x;
    rt_uint32_t i;

    if (value <= 0.0f)
    {
        return 0.0f;
    }

    x = (value >= 1.0f) ? value : 1.0f;
    for (i = 0U; i < 6U; i++)
    {
        x = 0.5f * (x + (value / x));
    }

    return x;
}

/**
 * @brief atan 近似，最大误差满足调试显示需求。
 *
 * @param x 输入斜率。
 * @return 角度，单位 rad。
 */
static float motor_atan_approx(float x)
{
    float ax = motor_abs(x);
    float base;
    float result;

    if (ax <= 1.0f)
    {
        result = (NS800_MOTOR_PI_OVER_4 * x) - (x * (ax - 1.0f) * (0.2447f + (0.0663f * ax)));
    }
    else
    {
        base = NS800_MOTOR_PI_OVER_4 * (1.0f / x);
        result = (x > 0.0f) ? (NS800_MOTOR_PI / 2.0f) : (-NS800_MOTOR_PI / 2.0f);
        result -= base - ((1.0f / x) * ((1.0f / ax) - 1.0f) * (0.2447f + (0.0663f / ax)));
    }

    return result;
}

/**
 * @brief atan2 近似，返回 [-pi, pi]。
 *
 * @param y beta 分量。
 * @param x alpha 分量。
 * @return 角度，单位 rad。
 */
static float motor_atan2_approx(float y, float x)
{
    float angle;

    if (x > 0.0f)
    {
        return motor_atan_approx(y / x);
    }
    if (x < 0.0f)
    {
        angle = motor_atan_approx(y / x);
        return (y >= 0.0f) ? (angle + NS800_MOTOR_PI) : (angle - NS800_MOTOR_PI);
    }
    if (y > 0.0f)
    {
        return NS800_MOTOR_PI / 2.0f;
    }
    if (y < 0.0f)
    {
        return -NS800_MOTOR_PI / 2.0f;
    }

    return 0.0f;
}

/**
 * @brief 将 alpha/beta 矢量转换为 WaveView 需要的角度和长度。
 *
 * @param ab alpha/beta 矢量。
 * @param angle_deg 输出角度，单位 deg，[0, 360)。
 * @param length_mv 输出长度，单位 mV。
 */
static void motor_wave_vector_metrics(const svm_ab_f32_t *ab, int *angle_deg, int *length_mv)
{
    float angle;
    float length_v;

    if ((ab == RT_NULL) || (angle_deg == RT_NULL) || (length_mv == RT_NULL))
    {
        return;
    }

    angle = motor_atan2_approx(ab->beta, ab->alpha) * NS800_MOTOR_RAD_TO_DEG;
    if (angle < 0.0f)
    {
        angle += 360.0f;
    }
    length_v = motor_sqrt_approx((ab->alpha * ab->alpha) + (ab->beta * ab->beta));

    *angle_deg = motor_round_to_int(angle);
    *length_mv = motor_round_to_int(length_v * NS800_MOTOR_WAVE_MV_PER_V);
}

/**
 * @brief 按配置抽样锁存一帧 WaveView 遥测快照。
 *
 * 该函数在 ISR 中调用，只复制最近控制输出，不做串口发送和复杂计算。
 *
 * @param out 最近一拍控制输出。
 */
static void motor_wave_capture_isr(const ns800_motor_output_t *out)
{
    rt_uint32_t decim = motor_wave_decim;

    if ((motor_wave_enable == RT_FALSE) || (out == RT_NULL) || (decim == 0U))
    {
        return;
    }
    if ((motor_status.isr_count % decim) != 0U)
    {
        return;
    }

    motor_wave_snapshot.seq = motor_status.isr_count;
    motor_wave_snapshot.output = *out;
    motor_wave_snapshot_ready = RT_TRUE;
}

/**
 * @brief 输出一帧 WaveView 文本协议。
 *
 * @param snapshot ISR 锁存的遥测快照。
 */
static void motor_wave_send_snapshot(const ns800_motor_wave_snapshot_t *snapshot)
{
    const ns800_motor_output_t *out;
    int total_angle;
    int total_length;
    int upper_angle;
    int upper_length;
    int lower_angle;
    int lower_length;

    if (snapshot == RT_NULL)
    {
        return;
    }

    out = &snapshot->output;
    motor_wave_vector_metrics(&out->voltage_cmd_ab, &total_angle, &total_length);
    motor_wave_vector_metrics(&out->voltage_split_ab.upper, &upper_angle, &upper_length);
    motor_wave_vector_metrics(&out->voltage_split_ab.lower, &lower_angle, &lower_length);

    rt_kprintf("svm:total,%d,%d\r\n", total_angle, total_length);
    rt_kprintf("svm:upper,%d,%d\r\n", upper_angle, upper_length);
    rt_kprintf("svm:lower,%d,%d\r\n", lower_angle, lower_length);
    rt_kprintf("wave:%d,%d,%d\r\n",
               motor_round_to_int(out->voltage_cmd_abc.a * NS800_MOTOR_WAVE_MV_PER_V),
               motor_round_to_int(out->voltage_cmd_abc.b * NS800_MOTOR_WAVE_MV_PER_V),
               motor_round_to_int(out->voltage_cmd_abc.c * NS800_MOTOR_WAVE_MV_PER_V));
}

/**
 * @brief WaveView 发送线程入口。
 *
 * @param parameter RT-Thread 线程参数，未使用。
 */
static void motor_wave_thread_entry(void *parameter)
{
    ns800_motor_wave_snapshot_t snapshot;
    rt_base_t level;

    RT_UNUSED(parameter);

    while (1)
    {
        if ((motor_wave_enable == RT_TRUE) && (motor_wave_snapshot_ready == RT_TRUE))
        {
            level = rt_hw_interrupt_disable();
            snapshot = motor_wave_snapshot;
            motor_wave_snapshot_ready = RT_FALSE;
            rt_hw_interrupt_enable(level);
            motor_wave_send_snapshot(&snapshot);
        }
        else
        {
            rt_thread_mdelay(10);
        }
    }
}

/**
 * @brief 创建 WaveView 发送线程。
 *
 * @return 0 表示成功。
 */
static int motor_wave_thread_start(void)
{
    if (motor_wave_thread != RT_NULL)
    {
        return 0;
    }

    motor_wave_thread = rt_thread_create("mwave",
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
 * @brief 将 ADC 原始值转换为相电流。
 *
 * 当前为占位标定：2048 作为零点，1 count = 1 mA。后续应按硬件采样比例更新。
 *
 * @param raw ADC 原始采样值。
 * @return 相电流，单位 A。
 */
static float motor_adc_current(rt_uint16_t raw)
{
    return (((float)(raw & 0x0fffU)) - NS800_MOTOR_ADC_CURRENT_ZERO) *
           NS800_MOTOR_ADC_CURRENT_GAIN_A_COUNT;
}

/**
 * @brief 初始化 EPWM2 为 10 kHz 电机控制定时中断。
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
 * ADC CH5/7/9 映射为 A/B/C 三相电流，按键全局变量映射为 xi、speed、Iq 限幅。
 *
 * @param in 输出控制输入结构。
 */
static void ns800_motor_fill_input(ns800_motor_input_t *in)
{
    const rt_uint16_t *frame;
    rt_uint32_t seq;
    rt_uint32_t reset_count;
    float theta_m;
    float omega_m;

    frame = ns800_adc_background_latest(&seq);
    if ((frame == RT_NULL) || (seq == motor_last_adc_seq))
    {
        motor_status.adc_miss_count++;
        in->sample.adc_valid = RT_FALSE;
    }
    else
    {
        motor_last_adc_seq = seq;
        motor_status.adc_seq = seq;
        in->sample.adc_valid = RT_TRUE;
    }

    if (frame != RT_NULL)
    {
        in->sample.phase_current_abc.a = motor_adc_current(frame[5]);
        in->sample.phase_current_abc.b = motor_adc_current(frame[7]);
        in->sample.phase_current_abc.c = motor_adc_current(frame[9]);
    }
    else
    {
        in->sample.phase_current_abc.a = 0.0f;
        in->sample.phase_current_abc.b = 0.0f;
        in->sample.phase_current_abc.c = 0.0f;
    }

    in->sample.feedback_valid = ns800_motor_feedback_get(&theta_m, &omega_m);
    if (!in->sample.feedback_valid)
    {
        motor_status.feedback_miss_count++;
        theta_m = 0.0f;
        omega_m = 0.0f;
    }
    in->sample.theta_m_rad = theta_m;
    in->sample.omega_m_rad_s = omega_m;

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
 * @brief EPWM2 电机控制中断服务函数。
 *
 * 中断内完成采样锁存、控制计算和 PWM shadow compare 更新。此路径禁止 printf、
 * 动态内存和任何可能阻塞的操作。
 */
void ns800_motor_isr(void)
{
    ns800_motor_input_t in;
    ns800_motor_status_t status;

    if (motor_running == RT_FALSE)
    {
        EPWM_clearEventTriggerInterruptFlag(NS800_MOTOR_EPWM_BASE);
        __DSB();
        return;
    }

    motor_status.isr_count++;
    ns800_motor_fill_input(&in);

    status = ns800_motor_step(&motor_state, &motor_cfg, &motor_params, &in, &motor_status.last_output);
    motor_status.fault_flags = motor_status.last_output.status;
    motor_status.mode = motor_mode;

    ns800_pwm_app_write_svm(&motor_status.last_output.duty);
    motor_wave_capture_isr(&motor_status.last_output);

    RT_UNUSED(status);
    EPWM_clearEventTriggerInterruptFlag(NS800_MOTOR_EPWM_BASE);
    __DSB();
}

/**
 * @brief 启动电机应用层。
 *
 * 默认进入开环模式，便于先验证 EPWM8~13 和功率分配 SVM 输出链路。
 *
 * @return 0 表示成功。
 */
int ns800_motor_app_start(void)
{
    rt_memset((void *)&motor_status, 0, sizeof(motor_status));
    motor_last_adc_seq = 0U;
    motor_last_reset_count = ns800_button_app_reset_count();
    motor_mode = NS800_MOTOR_MODE_OPEN_LOOP;
    motor_running = RT_FALSE;
    motor_reset_pending = RT_TRUE;
    motor_wave_enable = RT_FALSE;
    motor_wave_snapshot_ready = RT_FALSE;

    ns800_motor_default_config(&motor_cfg);
    ns800_motor_default_params(&motor_params);
    ns800_motor_init(&motor_state, &motor_cfg);

    ns800_motor_epwm_init();
    Interrupt_register(NS800_MOTOR_EPWM_IRQn, &ns800_motor_isr);
    Interrupt_enable(NS800_MOTOR_EPWM_IRQn);

    motor_running = RT_TRUE;
    motor_status.mode = motor_mode;
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
    Interrupt_disable(NS800_MOTOR_EPWM_IRQn);
    EPWM_disableInterrupt(NS800_MOTOR_EPWM_BASE);
    EPWM_setTimeBaseCounterMode(NS800_MOTOR_EPWM_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
    ns800_pwm_app_stop();
    motor_status.mode = motor_mode;

    return 0;
}

/**
 * @brief 切换电机控制模式。
 *
 * 速度闭环依赖真实转子反馈；如果反馈接口无效，则拒绝切入 SPEED 模式。
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
    motor_reset_pending = RT_TRUE;
    return 0;
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
    rt_kprintf("motor open-loop: 12V peak, 50Hz, hv=48V, lv=24V\r\n");
    return 0;
}

/**
 * @brief FinSH 命令：尝试切换到速度闭环模式。
 */
static int motor_speed_cmd(int argc, char **argv)
{
    int result;

    RT_UNUSED(argc);
    RT_UNUSED(argv);

    result = ns800_motor_app_set_mode(NS800_MOTOR_MODE_SPEED);
    if (result != 0)
    {
        rt_kprintf("motor speed: feedback invalid, keep previous mode\r\n");
        return result;
    }

    rt_kprintf("motor speed closed-loop: speed=%d rpm iq_limit=%d mA xi=%d/1000\r\n",
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
 * @brief FinSH 命令：打印最近一拍电机控制状态。
 */
static int motor_status_cmd(int argc, char **argv)
{
    const ns800_motor_app_status_t *status = ns800_motor_app_get_status();
    const ns800_motor_output_t *out = &status->last_output;

    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("motor: mode=%u isr=%u adc_miss=%u fb_miss=%u fault=0x%08x seq=%u\r\n",
               (unsigned int)status->mode,
               (unsigned int)status->isr_count,
               (unsigned int)status->adc_miss_count,
               (unsigned int)status->feedback_miss_count,
               (unsigned int)status->fault_flags,
               (unsigned int)status->adc_seq);
    rt_kprintf("cmd: xi=%d/1000 speed=%d rpm F=%d mA theta=%d mrad\r\n",
               (int)(out->xi * 1000.0f),
               (int)ns800_param_speed,
               (int)ns800_param_force_ma,
               (int)(out->theta_e_rad * 1000.0f));
    rt_kprintf("duty: ua=%d ub=%d uc=%d la=%d lb=%d lc=%d permille\r\n",
               (int)(out->duty.upper.ta * 1000.0f),
               (int)(out->duty.upper.tb * 1000.0f),
               (int)(out->duty.upper.tc * 1000.0f),
               (int)(out->duty.lower.ta * 1000.0f),
               (int)(out->duty.lower.tb * 1000.0f),
               (int)(out->duty.lower.tc * 1000.0f));

    return 0;
}

/**
 * @brief FinSH 命令：开启/关闭 WaveView 遥测输出。
 */
static int motor_wave_cmd(int argc, char **argv)
{
    rt_uint32_t rate_hz;

    if (argc < 2)
    {
        rt_kprintf("usage: motor_wave <on|off|status> [rate_hz]\r\n");
        return -RT_EINVAL;
    }

    if ((argv[1][0] == 's') || (argv[1][0] == 'S'))
    {
        rt_kprintf("motor_wave: %s rate=%uHz decim=%u ready=%u\r\n",
                   (motor_wave_enable == RT_TRUE) ? "on" : "off",
                   (unsigned int)motor_wave_rate_hz,
                   (unsigned int)motor_wave_decim,
                   (unsigned int)motor_wave_snapshot_ready);
        return 0;
    }

    if ((argv[1][0] == 'o') || (argv[1][0] == 'O'))
    {
        if ((argv[1][1] == 'n') || (argv[1][1] == 'N'))
        {
            rate_hz = motor_wave_rate_hz;
            if (argc >= 3)
            {
                rate_hz = (rt_uint32_t)strtoul(argv[2], RT_NULL, 0);
            }
            if (rate_hz == 0U)
            {
                rate_hz = NS800_MOTOR_WAVE_DEFAULT_RATE_HZ;
            }
            if (rate_hz > NS800_MOTOR_WAVE_MAX_RATE_HZ)
            {
                rate_hz = NS800_MOTOR_WAVE_MAX_RATE_HZ;
            }

            motor_wave_rate_hz = rate_hz;
            motor_wave_decim = NS800_MOTOR_CONTROL_FREQ_HZ / rate_hz;
            if (motor_wave_decim == 0U)
            {
                motor_wave_decim = 1U;
            }
            motor_wave_snapshot_ready = RT_FALSE;
            motor_wave_enable = RT_TRUE;
            rt_kprintf("motor_wave: on rate=%uHz decim=%u\r\n",
                       (unsigned int)motor_wave_rate_hz,
                       (unsigned int)motor_wave_decim);
            return 0;
        }

        if ((argv[1][1] == 'f') || (argv[1][1] == 'F'))
        {
            motor_wave_enable = RT_FALSE;
            motor_wave_snapshot_ready = RT_FALSE;
            rt_kprintf("motor_wave: off\r\n");
            return 0;
        }
    }

    rt_kprintf("usage: motor_wave <on|off|status> [rate_hz]\r\n");
    return -RT_EINVAL;
}

MSH_CMD_EXPORT_ALIAS(motor_ol_cmd, motor_ol, start ns800 motor open-loop test);
MSH_CMD_EXPORT_ALIAS(motor_speed_cmd, motor_speed, start ns800 motor speed loop);
MSH_CMD_EXPORT_ALIAS(motor_stop_cmd, motor_stop, stop ns800 motor pwm);
MSH_CMD_EXPORT_ALIAS(motor_status_cmd, motor_status, show ns800 motor status);
MSH_CMD_EXPORT_ALIAS(motor_wave_cmd, motor_wave, stream ns800 motor WaveView data);
#endif
