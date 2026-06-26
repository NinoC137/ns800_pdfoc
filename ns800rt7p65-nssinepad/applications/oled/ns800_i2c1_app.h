/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_I2C1_APP_H__
#define __NS800_I2C1_APP_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

int ns800_i2c1_app_start(void);
int ns800_i2c1_app_write(rt_uint8_t addr7, const rt_uint8_t *data, rt_uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_I2C1_APP_H__ */
