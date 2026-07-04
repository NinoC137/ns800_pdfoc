/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_FILTER_H__
#define __NS800_FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float alpha;
    float y;
    unsigned char initialized;
} ns800_lpf1_t;

typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} ns800_iir2_coeff_t;

typedef struct
{
    ns800_iir2_coeff_t coeff;
    float x1;
    float x2;
    float y1;
    float y2;
} ns800_iir2_t;

void ns800_lpf1_init(ns800_lpf1_t *filter, float alpha, float initial_value);
void ns800_lpf1_init_tau(ns800_lpf1_t *filter,
                         float sample_time_s,
                         float time_constant_s,
                         float initial_value);
void ns800_lpf1_reset(ns800_lpf1_t *filter, float value);
float ns800_lpf1_update(ns800_lpf1_t *filter, float input);

void ns800_iir2_init(ns800_iir2_t *filter, const ns800_iir2_coeff_t *coeff);
void ns800_iir2_reset(ns800_iir2_t *filter, float value);
float ns800_iir2_update(ns800_iir2_t *filter, float input);

void ns800_iir2_set_coeff(ns800_iir2_t *filter, const ns800_iir2_coeff_t *coeff);
ns800_iir2_coeff_t ns800_iir2_coeff_normalized(float b0,
                                               float b1,
                                               float b2,
                                               float a0,
                                               float a1,
                                               float a2);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_FILTER_H__ */
