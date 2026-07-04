/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_ADC_SCALE_H__
#define __NS800_ADC_SCALE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NS800_ADC_AC_ZERO_COUNT        (1986.0f)
#define NS800_ADC_FULL_SCALE_COUNT     (4096.0f)
#define NS800_ADC_VREF_V               (3.3f)
#define NS800_ADC_AC_VOLTAGE_SCALE     (NS800_ADC_VREF_V / 5.0f / 100.0f * 20000.0f)
#define NS800_ADC_AC_CURRENT_SCALE     (NS800_ADC_VREF_V * 2000.0f / 51.0f * 5.0f)
#define NS800_ADC_DC_CURRENT_OFFSET    (6.0f)
#define NS800_ADC_DC_VOLTAGE_SCALE     (NS800_ADC_VREF_V * 100.0f / 4.7f)
#define NS800_ADC_DC_CURRENT_SCALE     (NS800_ADC_VREF_V / 0.5f)

static inline float ns800_adc_ac_voltage(uint16_t raw)
{
    return (((float)(raw & 0x0fffU)) - NS800_ADC_AC_ZERO_COUNT) *
           NS800_ADC_AC_VOLTAGE_SCALE / NS800_ADC_FULL_SCALE_COUNT;
}

static inline float ns800_adc_ac_current(uint16_t raw)
{
    return (((float)(raw & 0x0fffU)) - NS800_ADC_AC_ZERO_COUNT) *
           NS800_ADC_AC_CURRENT_SCALE / NS800_ADC_FULL_SCALE_COUNT;
}

static inline float ns800_adc_dc_voltage(uint16_t raw)
{
    return ((float)(raw & 0x0fffU)) *
           NS800_ADC_DC_VOLTAGE_SCALE / NS800_ADC_FULL_SCALE_COUNT;
}

static inline float ns800_adc_dc_current(uint16_t raw)
{
    float current = (((float)(raw & 0x0fffU)) - NS800_ADC_DC_CURRENT_OFFSET) *
                    NS800_ADC_DC_CURRENT_SCALE / NS800_ADC_FULL_SCALE_COUNT;

    return (current < 0.0f) ? 0.0f : current;
}

#ifdef __cplusplus
}
#endif

#endif /* __NS800_ADC_SCALE_H__ */
