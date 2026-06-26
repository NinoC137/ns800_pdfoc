/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SVM_TYPES_H
#define SVM_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float a;
    float b;
    float c;
} svm_abc_f32_t;

typedef struct {
    float alpha;
    float beta;
} svm_ab_f32_t;

typedef struct {
    float d;
    float q;
} svm_dq_f32_t;

typedef struct {
    float sin_theta;
    float cos_theta;
} svm_sincos_f32_t;

typedef enum {
    SVM_PORT_UPPER = 0,
    SVM_PORT_LOWER = 1
} svm_port_select_t;

typedef enum {
    SVM_STATUS_OK = 0,
    SVM_STATUS_NULL_POINTER = 1,
    SVM_STATUS_LOW_DC_VOLTAGE = 2,
    SVM_STATUS_OVERMODULATION = 4,
    SVM_STATUS_CLAMPED = 8
} svm_status_t;

typedef struct {
    float ta;
    float tb;
    float tc;
    uint8_t sector;
    uint8_t status;
} svm_svm_out_t;

typedef struct {
    svm_svm_out_t upper;
    svm_svm_out_t lower;
} svm_dual_out_t;

typedef struct {
    svm_ab_f32_t upper;
    svm_ab_f32_t lower;
} svm_split_ab_f32_t;

typedef struct {
    uint8_t gate[12u];
} svm_pwm_logic_t;

#ifdef __cplusplus
}
#endif

#endif /* SVM_TYPES_H */
