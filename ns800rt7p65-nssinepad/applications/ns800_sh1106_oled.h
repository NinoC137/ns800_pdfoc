/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_SH1106_OLED_H__
#define __NS800_SH1106_OLED_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NS800_SH1106_WIDTH   128U
#define NS800_SH1106_HEIGHT  64U

int ns800_sh1106_oled_start(void);
void ns800_sh1106_oled_clear(void);
void ns800_sh1106_oled_flush(void);
void ns800_sh1106_oled_draw_pixel(rt_uint32_t x, rt_uint32_t y, rt_bool_t on);
void ns800_sh1106_oled_draw_string(rt_uint32_t x, rt_uint32_t y, const char *text);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_SH1106_OLED_H__ */
