/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_filter.h"

static float ns800_filter_clamp(float value, float min_value, float max_value)
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

void ns800_lpf1_init(ns800_lpf1_t *filter, float alpha, float initial_value)
{
    if (filter == 0)
    {
        return;
    }

    filter->alpha = ns800_filter_clamp(alpha, 0.0f, 1.0f);
    filter->y = initial_value;
    filter->initialized = 1U;
}

void ns800_lpf1_init_tau(ns800_lpf1_t *filter,
                         float sample_time_s,
                         float time_constant_s,
                         float initial_value)
{
    float alpha;

    if (time_constant_s <= 0.0f)
    {
        alpha = 1.0f;
    }
    else if (sample_time_s <= 0.0f)
    {
        alpha = 0.0f;
    }
    else
    {
        alpha = sample_time_s / (time_constant_s + sample_time_s);
    }

    ns800_lpf1_init(filter, alpha, initial_value);
}

void ns800_lpf1_reset(ns800_lpf1_t *filter, float value)
{
    if (filter == 0)
    {
        return;
    }

    filter->y = value;
    filter->initialized = 1U;
}

float ns800_lpf1_update(ns800_lpf1_t *filter, float input)
{
    if (filter == 0)
    {
        return input;
    }

    if (filter->initialized == 0U)
    {
        ns800_lpf1_reset(filter, input);
        return input;
    }

    filter->y += filter->alpha * (input - filter->y);
    return filter->y;
}

void ns800_iir2_init(ns800_iir2_t *filter, const ns800_iir2_coeff_t *coeff)
{
    if ((filter == 0) || (coeff == 0))
    {
        return;
    }

    filter->coeff = *coeff;
    ns800_iir2_reset(filter, 0.0f);
}

void ns800_iir2_reset(ns800_iir2_t *filter, float value)
{
    if (filter == 0)
    {
        return;
    }

    filter->x1 = value;
    filter->x2 = value;
    filter->y1 = value;
    filter->y2 = value;
}

float ns800_iir2_update(ns800_iir2_t *filter, float input)
{
    float output;

    if (filter == 0)
    {
        return input;
    }

    output = filter->coeff.b0 * input +
             filter->coeff.b1 * filter->x1 +
             filter->coeff.b2 * filter->x2 -
             filter->coeff.a1 * filter->y1 -
             filter->coeff.a2 * filter->y2;

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

void ns800_iir2_set_coeff(ns800_iir2_t *filter, const ns800_iir2_coeff_t *coeff)
{
    if ((filter == 0) || (coeff == 0))
    {
        return;
    }

    filter->coeff = *coeff;
}

ns800_iir2_coeff_t ns800_iir2_coeff_normalized(float b0,
                                               float b1,
                                               float b2,
                                               float a0,
                                               float a1,
                                               float a2)
{
    ns800_iir2_coeff_t coeff;

    if (a0 == 0.0f)
    {
        a0 = 1.0f;
    }

    coeff.b0 = b0 / a0;
    coeff.b1 = b1 / a0;
    coeff.b2 = b2 / a0;
    coeff.a1 = a1 / a0;
    coeff.a2 = a2 / a0;
    return coeff;
}
