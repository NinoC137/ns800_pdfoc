/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_display_app.h"

#include "ns800_button_app.h"
#include "ns800_sh1106_oled.h"

#include <rtthread.h>

#define NS800_DISPLAY_THREAD_STACK  1024U
#define NS800_DISPLAY_THREAD_PRIO   12U
#define NS800_DISPLAY_THREAD_TICK   10U
#define NS800_DISPLAY_REFRESH_MS    200U

static rt_thread_t display_thread = RT_NULL;

static int ns800_abs_i32(int value)
{
    return (value < 0) ? -value : value;
}

static void ns800_i32_to_dec(char *buf, int value)
{
    char tmp[12];
    int pos = 0;
    int out = 0;
    unsigned int v;

    if (value < 0)
    {
        buf[out++] = '-';
        v = (unsigned int)(-value);
    }
    else
    {
        v = (unsigned int)value;
    }

    do
    {
        tmp[pos++] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v != 0U);

    while (pos > 0)
    {
        buf[out++] = tmp[--pos];
    }
    buf[out] = '\0';
}

static void ns800_make_xi(char *buf)
{
    int xi100 = (int)(ns800_param_xi * 100.0f);
    int abs_xi100;

    if ((ns800_param_xi >= 0.0f) && (((float)xi100 / 100.0f) < ns800_param_xi))
    {
        xi100++;
    }
    if ((ns800_param_xi < 0.0f) && (((float)xi100 / 100.0f) > ns800_param_xi))
    {
        xi100--;
    }

    abs_xi100 = ns800_abs_i32(xi100);
    buf[0] = 'x';
    buf[1] = 'i';
    buf[2] = ':';
    if (xi100 < 0)
    {
        buf[3] = '-';
        buf[4] = (char)('0' + ((abs_xi100 / 100) % 10));
        buf[5] = '.';
        buf[6] = (char)('0' + ((abs_xi100 / 10) % 10));
        buf[7] = (char)('0' + (abs_xi100 % 10));
        buf[8] = '\0';
    }
    else
    {
        buf[3] = (char)('0' + ((abs_xi100 / 100) % 10));
        buf[4] = '.';
        buf[5] = (char)('0' + ((abs_xi100 / 10) % 10));
        buf[6] = (char)('0' + (abs_xi100 % 10));
        buf[7] = '\0';
    }
}

static void ns800_draw_status_page(void)
{
    char line[24];
    char num[12];

    ns800_sh1106_oled_clear();
    ns800_sh1106_oled_draw_string(0, 0, "NS800 PD FOC");

    ns800_make_xi(line);
    ns800_sh1106_oled_draw_string(0, 16, line);

    ns800_i32_to_dec(num, ns800_param_speed);
    line[0] = 's';
    line[1] = 'p';
    line[2] = 'e';
    line[3] = 'e';
    line[4] = 'd';
    line[5] = ':';
    rt_strncpy(&line[6], num, sizeof(line) - 6U);
    line[sizeof(line) - 1U] = '\0';
    ns800_sh1106_oled_draw_string(0, 32, line);

    ns800_i32_to_dec(num, ns800_param_force_ma);
    line[0] = 'F';
    line[1] = '(';
    line[2] = 'm';
    line[3] = 'A';
    line[4] = ')';
    line[5] = ':';
    rt_strncpy(&line[6], num, sizeof(line) - 6U);
    line[sizeof(line) - 1U] = '\0';
    ns800_sh1106_oled_draw_string(0, 48, line);

    ns800_sh1106_oled_flush();
}

static void ns800_display_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);

    if (ns800_sh1106_oled_start() != 0)
    {
        rt_kprintf("ns800 oled start failed\r\n");
    }

    while (1)
    {
        ns800_draw_status_page();
        rt_thread_mdelay(NS800_DISPLAY_REFRESH_MS);
    }
}

int ns800_display_app_start(void)
{
    if (display_thread != RT_NULL)
    {
        return 0;
    }

    display_thread = rt_thread_create("oled",
                                      ns800_display_thread_entry,
                                      RT_NULL,
                                      NS800_DISPLAY_THREAD_STACK,
                                      NS800_DISPLAY_THREAD_PRIO,
                                      NS800_DISPLAY_THREAD_TICK);
    if (display_thread == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_thread_startup(display_thread);
    return 0;
}
