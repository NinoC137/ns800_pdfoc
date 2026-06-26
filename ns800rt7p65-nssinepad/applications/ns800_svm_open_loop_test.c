/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_svm_open_loop_test.h"

#include <stdlib.h>

#include <board.h>

#include "epwm.h"
#include "interrupt.h"
#include "ns800_pwm_app.h"
#include "svm_power_alloc.h"
#include "svm_transform.h"

#ifdef RT_USING_FINSH
#include "finsh.h"
#endif

#define NS800_SVM_OL_EPWM_BASE          EPWM2
#define NS800_SVM_OL_EPWM_IRQn          EPWM2_INT_IRQn
#define NS800_SVM_OL_CONTROL_FREQ_HZ    10000U
#define NS800_SVM_OL_TBPRD              20000U
#define NS800_SVM_OL_SAMPLE_TIME_S      (1.0e-4f)
#define NS800_SVM_OL_DEFAULT_HVDC       (48.0f)
#define NS800_SVM_OL_DEFAULT_LVDC       (24.0f)
#define NS800_SVM_OL_DEFAULT_XI         (0.63f)
#define NS800_SVM_OL_DEFAULT_VPEAK      (12.0f)
#define NS800_SVM_OL_DEFAULT_FREQ       (50.0f)
#define NS800_SVM_OL_PRINT_STACK        1024U
#define NS800_SVM_OL_PRINT_PRIO         20U
#define NS800_SVM_OL_PRINT_TICK         10U
#define NS800_SVM_OL_PRINT_PERIOD_MS    10U
#define NS800_SVM_OL_PRINT_SCALE        (1000.0f)

static volatile rt_bool_t ol_running = RT_FALSE;
static svm_nco_t ol_nco;
static svm_svm_config_t ol_svm_cfg;
static ns800_svm_open_loop_status_t ol_status;
static rt_thread_t ol_print_thread = RT_NULL;

static float ol_clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static void ns800_svm_open_loop_epwm_init(void)
{
    EPWM_setClockPrescaler(NS800_SVM_OL_EPWM_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(NS800_SVM_OL_EPWM_BASE, NS800_SVM_OL_TBPRD);
    EPWM_setTimeBaseCounter(NS800_SVM_OL_EPWM_BASE, 0U);
    EPWM_setTimeBaseCounterMode(NS800_SVM_OL_EPWM_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(NS800_SVM_OL_EPWM_BASE);
    EPWM_setPhaseShift(NS800_SVM_OL_EPWM_BASE, 0U);
    EPWM_disableInterrupt(NS800_SVM_OL_EPWM_BASE);
    EPWM_setInterruptSource(NS800_SVM_OL_EPWM_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_setInterruptEventCount(NS800_SVM_OL_EPWM_BASE, 1U);
    EPWM_clearEventTriggerInterruptFlag(NS800_SVM_OL_EPWM_BASE);
    EPWM_enableInterrupt(NS800_SVM_OL_EPWM_BASE);
}

void ns800_svm_open_loop_test_isr(void)
{
    svm_sincos_f32_t sc;
    svm_ab_f32_t u_ab;
    svm_abc_f32_t u_abc;
    svm_status_t status;

    if (ol_running == RT_FALSE)
    {
        EPWM_clearEventTriggerInterruptFlag(NS800_SVM_OL_EPWM_BASE);
        __DSB();
        return;
    }

    svm_lut_sincos(ol_nco.angle_rad, &sc);
    u_ab.alpha = ol_status.voltage_peak_v * sc.cos_theta;
    u_ab.beta = ol_status.voltage_peak_v * sc.sin_theta;
    svm_inverse_clarke(&u_ab, &u_abc);

    status = svm_compute_dual(&u_ab,
                              ol_status.xi,
                              ol_status.hvdc_v,
                              ol_status.lvdc_v,
                              &ol_svm_cfg,
                              &ol_status.last_duty);
    ol_status.fault_flags = (rt_uint32_t)status;
    ol_status.angle_rad = ol_nco.angle_rad;
    ol_status.last_ac_voltage_abc = u_abc;
    ol_status.isr_count++;

    ns800_pwm_app_write_svm(&ol_status.last_duty);
    svm_nco_step(&ol_nco);

    EPWM_clearEventTriggerInterruptFlag(NS800_SVM_OL_EPWM_BASE);
    __DSB();
}

static int ol_float_to_i32(float value)
{
    if (value >= 0.0f)
    {
        return (int)(value + 0.5f);
    }
    return (int)(value - 0.5f);
}

static void ns800_svm_open_loop_print_entry(void *parameter)
{
    int va;
    int vb;
    int vc;

    RT_UNUSED(parameter);

    while (1)
    {
        if (ol_running == RT_TRUE)
        {
            // va = ol_float_to_i32(ol_status.last_ac_voltage_abc.a * NS800_SVM_OL_PRINT_SCALE);
            // vb = ol_float_to_i32(ol_status.last_ac_voltage_abc.b * NS800_SVM_OL_PRINT_SCALE);
            // vc = ol_float_to_i32(ol_status.last_ac_voltage_abc.c * NS800_SVM_OL_PRINT_SCALE);

            va = ol_float_to_i32((ol_status.last_duty.upper.ta * ol_status.hvdc_v -
                      ol_status.last_duty.lower.ta * ol_status.lvdc_v) * NS800_SVM_OL_PRINT_SCALE);
            vb = ol_float_to_i32((ol_status.last_duty.upper.tb * ol_status.hvdc_v -
                                ol_status.last_duty.lower.tb * ol_status.lvdc_v) * NS800_SVM_OL_PRINT_SCALE);
            vc = ol_float_to_i32((ol_status.last_duty.upper.tc * ol_status.hvdc_v -
                                ol_status.last_duty.lower.tc * ol_status.lvdc_v) * NS800_SVM_OL_PRINT_SCALE);

            rt_kprintf("%d,%d,%d\r\n", va, vb, vc);
        }
        rt_thread_mdelay(1);
    }
}

static void ns800_svm_open_loop_print_start(void)
{
    if (ol_print_thread != RT_NULL)
    {
        return;
    }

    ol_print_thread = rt_thread_create("svm_plot",
                                       ns800_svm_open_loop_print_entry,
                                       RT_NULL,
                                       NS800_SVM_OL_PRINT_STACK,
                                       NS800_SVM_OL_PRINT_PRIO,
                                       NS800_SVM_OL_PRINT_TICK);
    if (ol_print_thread != RT_NULL)
    {
        rt_thread_startup(ol_print_thread);
    }
}

int ns800_svm_open_loop_test_set(float hvdc_v,
                                 float lvdc_v,
                                 float xi,
                                 float voltage_peak_v,
                                 float output_freq_hz)
{
    ol_status.hvdc_v = hvdc_v;
    ol_status.lvdc_v = lvdc_v;
    ol_status.xi = ol_clamp(xi, 0.0f, 1.0f);
    ol_status.voltage_peak_v = voltage_peak_v;
    ol_status.output_freq_hz = output_freq_hz;
    svm_nco_set_frequency(&ol_nco, output_freq_hz, NS800_SVM_OL_SAMPLE_TIME_S);
    return 0;
}

int ns800_svm_open_loop_test_start(void)
{
    rt_memset((void *)&ol_status, 0, sizeof(ol_status));

    ol_svm_cfg = svm_svm_default_config();
    ol_svm_cfg.sample_time_s = NS800_SVM_OL_SAMPLE_TIME_S;
    ol_svm_cfg.min_dc_voltage = 0.0f;
    svm_nco_init(&ol_nco, NS800_SVM_OL_DEFAULT_FREQ, NS800_SVM_OL_SAMPLE_TIME_S);
    ns800_svm_open_loop_test_set(NS800_SVM_OL_DEFAULT_HVDC,
                                 NS800_SVM_OL_DEFAULT_LVDC,
                                 NS800_SVM_OL_DEFAULT_XI,
                                 NS800_SVM_OL_DEFAULT_VPEAK,
                                 NS800_SVM_OL_DEFAULT_FREQ);

    ns800_svm_open_loop_epwm_init();
    Interrupt_register(NS800_SVM_OL_EPWM_IRQn, &ns800_svm_open_loop_test_isr);
    Interrupt_enable(NS800_SVM_OL_EPWM_IRQn);
    ol_running = RT_TRUE;
    ns800_svm_open_loop_print_start();
    return 0;
}

int ns800_svm_open_loop_test_stop(void)
{
    ol_running = RT_FALSE;
    Interrupt_disable(NS800_SVM_OL_EPWM_IRQn);
    EPWM_disableInterrupt(NS800_SVM_OL_EPWM_BASE);
    EPWM_setTimeBaseCounterMode(NS800_SVM_OL_EPWM_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
    return 0;
}

const ns800_svm_open_loop_status_t *ns800_svm_open_loop_test_get_status(void)
{
    return &ol_status;
}

#ifdef RT_USING_FINSH
static void svm_ol_print_status(void)
{
    const ns800_svm_open_loop_status_t *st = ns800_svm_open_loop_test_get_status();

    rt_kprintf("svm_ol: cnt=%u fault=0x%08x angle=%d.%03d hvdc=%d.%03d lvdc=%d.%03d xi=%d.%03d vpk=%d.%03d freq=%d.%03d\r\n",
               (unsigned int)st->isr_count,
               (unsigned int)st->fault_flags,
               (int)st->angle_rad, (int)((st->angle_rad - (float)((int)st->angle_rad)) * 1000.0f),
               (int)st->hvdc_v, (int)((st->hvdc_v - (float)((int)st->hvdc_v)) * 1000.0f),
               (int)st->lvdc_v, (int)((st->lvdc_v - (float)((int)st->lvdc_v)) * 1000.0f),
               (int)st->xi, (int)((st->xi - (float)((int)st->xi)) * 1000.0f),
               (int)st->voltage_peak_v, (int)((st->voltage_peak_v - (float)((int)st->voltage_peak_v)) * 1000.0f),
               (int)st->output_freq_hz, (int)((st->output_freq_hz - (float)((int)st->output_freq_hz)) * 1000.0f));
    rt_kprintf("upper: sector=%u ta=%d.%03d tb=%d.%03d tc=%d.%03d status=0x%02x\r\n",
               st->last_duty.upper.sector,
               (int)st->last_duty.upper.ta, (int)((st->last_duty.upper.ta - (float)((int)st->last_duty.upper.ta)) * 1000.0f),
               (int)st->last_duty.upper.tb, (int)((st->last_duty.upper.tb - (float)((int)st->last_duty.upper.tb)) * 1000.0f),
               (int)st->last_duty.upper.tc, (int)((st->last_duty.upper.tc - (float)((int)st->last_duty.upper.tc)) * 1000.0f),
               st->last_duty.upper.status);
    rt_kprintf("lower: sector=%u ta=%d.%03d tb=%d.%03d tc=%d.%03d status=0x%02x\r\n",
               st->last_duty.lower.sector,
               (int)st->last_duty.lower.ta, (int)((st->last_duty.lower.ta - (float)((int)st->last_duty.lower.ta)) * 1000.0f),
               (int)st->last_duty.lower.tb, (int)((st->last_duty.lower.tb - (float)((int)st->last_duty.lower.tb)) * 1000.0f),
               (int)st->last_duty.lower.tc, (int)((st->last_duty.lower.tc - (float)((int)st->last_duty.lower.tc)) * 1000.0f),
               st->last_duty.lower.status);
}

static int svm_ol_start_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return ns800_svm_open_loop_test_start();
}
MSH_CMD_EXPORT_ALIAS(svm_ol_start_cmd, svm_ol_start, start open-loop SVM PWM test);

static int svm_ol_stop_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    return ns800_svm_open_loop_test_stop();
}
MSH_CMD_EXPORT_ALIAS(svm_ol_stop_cmd, svm_ol_stop, stop open-loop SVM PWM test);

static int svm_ol_status_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    svm_ol_print_status();
    return 0;
}
MSH_CMD_EXPORT_ALIAS(svm_ol_status_cmd, svm_ol_status, print open-loop SVM PWM status);

static int svm_ol_set_cmd(int argc, char **argv)
{
    if (argc != 6)
    {
        rt_kprintf("usage: svm_ol_set <hvdc> <lvdc> <xi> <v_peak> <freq_hz>\r\n");
        return -RT_EINVAL;
    }

    ns800_svm_open_loop_test_set((float)atof(argv[1]),
                                 (float)atof(argv[2]),
                                 (float)atof(argv[3]),
                                 (float)atof(argv[4]),
                                 (float)atof(argv[5]));
    svm_ol_print_status();
    return 0;
}
MSH_CMD_EXPORT_ALIAS(svm_ol_set_cmd, svm_ol_set, set open-loop SVM PWM parameters);
#endif
