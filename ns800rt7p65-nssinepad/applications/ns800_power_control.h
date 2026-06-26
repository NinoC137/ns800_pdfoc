/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_POWER_CONTROL_H__
#define __NS800_POWER_CONTROL_H__

#include <rtthread.h>
#include "svm_control.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    volatile rt_uint32_t isr_count;
    volatile rt_uint32_t adc_miss_count;
    volatile rt_uint32_t fault_flags;
    svm_control_output_t last_output;
} ns800_power_control_status_t;

int ns800_power_control_start(void);
int ns800_power_control_stop(void);
const ns800_power_control_status_t *ns800_power_control_get_status(void);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_POWER_CONTROL_H__ */
