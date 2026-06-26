/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "svm_port.h"

#include <rtthread.h>

#include "ns800_adc_background.h"
#include "ns800_pwm_app.h"

void svm_port_enter_critical(void)
{
    rt_enter_critical();
}

void svm_port_exit_critical(void)
{
    rt_exit_critical();
}

bool svm_port_read_adc(svm_adc_sample_t *sample)
{
    const rt_uint16_t *frame;
    rt_uint32_t seq;

    if (sample == RT_NULL) return false;

    frame = ns800_adc_background_latest(&seq);
    if (frame == RT_NULL) return false;

    sample->ac_voltage_abc.a = (float)frame[4];
    sample->ac_voltage_abc.b = (float)frame[6];
    sample->ac_voltage_abc.c = (float)frame[8];
    sample->ac_current_abc.a = (float)frame[5];
    sample->ac_current_abc.b = (float)frame[7];
    sample->ac_current_abc.c = (float)frame[9];
    sample->dc_upper_voltage = (float)frame[0];
    sample->dc_upper_current = (float)frame[1];
    sample->dc_lower_voltage = (float)frame[2];
    sample->dc_lower_current = (float)frame[3];
    (void)seq;
    return true;
}

void svm_port_write_pwm_compare(const svm_dual_out_t *duty)
{
    if (duty == RT_NULL) return;
    ns800_pwm_app_write_svm(duty);
}

void svm_port_report_fault(uint32_t fault_flags)
{
    rt_kprintf("svm fault: 0x%08x\r\n", (unsigned int)fault_flags);
}
