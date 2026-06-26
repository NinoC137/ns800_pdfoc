/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_ADC_BACKGROUND_H__
#define __NS800_ADC_BACKGROUND_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NS800_ADC_BACKGROUND_CHANNELS     10U
#define NS800_ADC_BACKGROUND_FRAME_COUNT  32U

int ns800_adc_background_start(void);
int ns800_adc_background_stop(void);
const rt_uint16_t *ns800_adc_background_latest(rt_uint32_t *seq);
int ns800_adc_background_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_ADC_BACKGROUND_H__ */
