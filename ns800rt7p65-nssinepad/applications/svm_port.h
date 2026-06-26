/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SVM_PORT_H
#define SVM_PORT_H

#include <stdbool.h>
#include "svm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    svm_abc_f32_t ac_voltage_abc;
    svm_abc_f32_t ac_current_abc;
    float dc_upper_voltage;
    float dc_upper_current;
    float dc_lower_voltage;
    float dc_lower_current;
} svm_adc_sample_t;

void svm_port_enter_critical(void);
void svm_port_exit_critical(void);
bool svm_port_read_adc(svm_adc_sample_t *sample);
void svm_port_write_pwm_compare(const svm_dual_out_t *duty);
void svm_port_report_fault(uint32_t fault_flags);

#ifdef __cplusplus
}
#endif

#endif /* SVM_PORT_H */
