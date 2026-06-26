/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_BUTTON_APP_H__
#define __NS800_BUTTON_APP_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile float ns800_param_xi;
extern volatile rt_int32_t ns800_param_speed;
extern volatile rt_int32_t ns800_param_force_ma;

int ns800_button_app_start(void);
void ns800_button_app_reset_params(void);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_BUTTON_APP_H__ */
