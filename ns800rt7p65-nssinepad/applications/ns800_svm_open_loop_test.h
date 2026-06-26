/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_SVM_OPEN_LOOP_TEST_H__
#define __NS800_SVM_OPEN_LOOP_TEST_H__

#include <rtthread.h>
#include "svm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float hvdc_v;
    float lvdc_v;
    float xi;
    float voltage_peak_v;
    float output_freq_hz;
    volatile rt_uint32_t isr_count;
    volatile rt_uint32_t fault_flags;
    volatile float angle_rad;
    svm_dual_out_t last_duty;
} ns800_svm_open_loop_status_t;

int ns800_svm_open_loop_test_start(void);
int ns800_svm_open_loop_test_stop(void);
int ns800_svm_open_loop_test_set(float hvdc_v,
                                 float lvdc_v,
                                 float xi,
                                 float voltage_peak_v,
                                 float output_freq_hz);
const ns800_svm_open_loop_status_t *ns800_svm_open_loop_test_get_status(void);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_SVM_OPEN_LOOP_TEST_H__ */
