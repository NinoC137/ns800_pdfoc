/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_motor_app.h"

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

static ns800_motor_config_t motor_cfg;
static ns800_motor_params_t motor_params;
static ns800_motor_state_t motor_state;
static ns800_motor_app_status_t motor_status;
static volatile rt_bool_t motor_running = RT_FALSE;
static volatile rt_bool_t motor_reset_pending = RT_FALSE;
static volatile ns800_motor_mode_t motor_mode = NS800_MOTOR_MODE_OPEN_LOOP;
static rt_uint32_t motor_last_adc_seq = 0U;
static rt_uint32_t motor_last_reset_count = 0U;

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

static float motor_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

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

static float motor_adc_current(rt_uint16_t raw)
{
    return (((float)(raw & 0x0fffU)) - NS800_MOTOR_ADC_CURRENT_ZERO) *
           NS800_MOTOR_ADC_CURRENT_GAIN_A_COUNT;
}

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

    RT_UNUSED(status);
    EPWM_clearEventTriggerInterruptFlag(NS800_MOTOR_EPWM_BASE);
    __DSB();
}

int ns800_motor_app_start(void)
{
    rt_memset((void *)&motor_status, 0, sizeof(motor_status));
    motor_last_adc_seq = 0U;
    motor_last_reset_count = ns800_button_app_reset_count();
    motor_mode = NS800_MOTOR_MODE_OPEN_LOOP;
    motor_running = RT_FALSE;
    motor_reset_pending = RT_TRUE;

    ns800_motor_default_config(&motor_cfg);
    ns800_motor_default_params(&motor_params);
    ns800_motor_init(&motor_state, &motor_cfg);

    ns800_motor_epwm_init();
    Interrupt_register(NS800_MOTOR_EPWM_IRQn, &ns800_motor_isr);
    Interrupt_enable(NS800_MOTOR_EPWM_IRQn);

    motor_running = RT_TRUE;
    motor_status.mode = motor_mode;
    return 0;
}

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

const ns800_motor_app_status_t *ns800_motor_app_get_status(void)
{
    return &motor_status;
}

#ifdef RT_USING_FINSH
static int motor_ol_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    ns800_motor_app_set_mode(NS800_MOTOR_MODE_OPEN_LOOP);
    rt_kprintf("motor open-loop: 12V peak, 50Hz, hv=48V, lv=24V\r\n");
    return 0;
}

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

static int motor_stop_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    ns800_motor_app_set_mode(NS800_MOTOR_MODE_STOP);
    rt_kprintf("motor stop\r\n");
    return 0;
}

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

MSH_CMD_EXPORT_ALIAS(motor_ol_cmd, motor_ol, start ns800 motor open-loop test);
MSH_CMD_EXPORT_ALIAS(motor_speed_cmd, motor_speed, start ns800 motor speed loop);
MSH_CMD_EXPORT_ALIAS(motor_stop_cmd, motor_stop, stop ns800 motor pwm);
MSH_CMD_EXPORT_ALIAS(motor_status_cmd, motor_status, show ns800 motor status);
#endif
