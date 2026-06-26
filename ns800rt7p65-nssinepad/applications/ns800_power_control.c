/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_power_control.h"

#include <board.h>

#include "interrupt.h"
#include "epwm.h"

#include "ns800_adc_background.h"
#include "ns800_pwm_app.h"

#define NS800_POWER_CONTROL_EPWM_BASE          EPWM2
#define NS800_POWER_CONTROL_EPWM_IRQn          EPWM2_INT_IRQn
#define NS800_POWER_CONTROL_FREQ_HZ            10000U
#define NS800_POWER_CONTROL_TBPRD              20000U

static svm_control_config_t power_cfg;
static svm_control_state_t power_state;
static ns800_power_control_status_t power_status;
static volatile rt_bool_t power_running = RT_FALSE;
static rt_uint32_t power_last_adc_seq = 0U;

static void ns800_power_control_epwm_init(void)
{
    EPWM_setClockPrescaler(NS800_POWER_CONTROL_EPWM_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(NS800_POWER_CONTROL_EPWM_BASE, NS800_POWER_CONTROL_TBPRD);
    EPWM_setTimeBaseCounter(NS800_POWER_CONTROL_EPWM_BASE, 0U);
    EPWM_setTimeBaseCounterMode(NS800_POWER_CONTROL_EPWM_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(NS800_POWER_CONTROL_EPWM_BASE);
    EPWM_setPhaseShift(NS800_POWER_CONTROL_EPWM_BASE, 0U);
    EPWM_disableInterrupt(NS800_POWER_CONTROL_EPWM_BASE);
    EPWM_setInterruptSource(NS800_POWER_CONTROL_EPWM_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_setInterruptEventCount(NS800_POWER_CONTROL_EPWM_BASE, 1U);
    EPWM_clearEventTriggerInterruptFlag(NS800_POWER_CONTROL_EPWM_BASE);
    EPWM_enableInterrupt(NS800_POWER_CONTROL_EPWM_BASE);
}

void ns800_power_control_isr(void)
{
    const rt_uint16_t *frame;
    rt_uint32_t seq;
    svm_status_t status;

    if (power_running == RT_FALSE)
    {
        EPWM_clearEventTriggerInterruptFlag(NS800_POWER_CONTROL_EPWM_BASE);
        __DSB();
        return;
    }

    power_status.isr_count++;

    frame = ns800_adc_background_latest(&seq);
    if (frame == RT_NULL)
    {
        power_status.adc_miss_count++;
    }
    else if (seq == power_last_adc_seq)
    {
        power_status.adc_miss_count++;
    }
    else
    {
        power_last_adc_seq = seq;
    }

    status = svm_control_isr_step(&power_state, &power_cfg, &power_status.last_output);
    if (status != SVM_STATUS_OK)
    {
        power_status.fault_flags = power_status.last_output.status;
    }
    else
    {
        power_status.fault_flags = power_status.last_output.status;
    }

    EPWM_clearEventTriggerInterruptFlag(NS800_POWER_CONTROL_EPWM_BASE);
    __DSB();
}

int ns800_power_control_start(void)
{
    rt_memset((void *)&power_status, 0, sizeof(power_status));
    power_last_adc_seq = 0U;
    svm_control_default_config(&power_cfg);
    power_cfg.sample_time_s = 1.0e-4f;
    power_cfg.output_frequency_hz = 50.0f;
    power_cfg.power_factor = 0.5f;
    power_cfg.svm.sample_time_s = 1.0e-4f;
    power_cfg.svm.min_dc_voltage = 1.0f;
    svm_control_init(&power_state, &power_cfg);

    ns800_power_control_epwm_init();
    Interrupt_register(NS800_POWER_CONTROL_EPWM_IRQn, &ns800_power_control_isr);
    Interrupt_enable(NS800_POWER_CONTROL_EPWM_IRQn);
    power_running = RT_TRUE;
    return 0;
}

int ns800_power_control_stop(void)
{
    power_running = RT_FALSE;
    Interrupt_disable(NS800_POWER_CONTROL_EPWM_IRQn);
    EPWM_disableInterrupt(NS800_POWER_CONTROL_EPWM_BASE);
    EPWM_setTimeBaseCounterMode(NS800_POWER_CONTROL_EPWM_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
    return 0;
}

const ns800_power_control_status_t *ns800_power_control_get_status(void)
{
    return &power_status;
}
