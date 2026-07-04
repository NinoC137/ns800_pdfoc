/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_display_app.h"

#include "ns800_adc_background.h"
#include "ns800_adc_scale.h"
#include "ns800_button_app.h"
#include "ns800_motor_app.h"
#include "ns800_sh1106_oled.h"
#include "svm_power_alloc.h"

#include <rtthread.h>

#define NS800_DISPLAY_THREAD_STACK  1024U
#define NS800_DISPLAY_THREAD_PRIO   12U
#define NS800_DISPLAY_THREAD_TICK   10U
#define NS800_DISPLAY_REFRESH_MS    100U

static rt_thread_t display_thread = RT_NULL;

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

static void ns800_append_ma_unit(char *buf, rt_size_t size)
{
    rt_size_t len;

    if (size < 3U)
    {
        return;
    }

    len = rt_strlen(buf);
    if ((len + 2U) >= size)
    {
        return;
    }

    buf[len++] = 'm';
    buf[len++] = 'A';
    buf[len] = '\0';
}

static void ns800_display_sample_power(float *hv_power, float *lv_power, float *total_power)
{
    const rt_uint16_t *frame;
    float hv_v;
    float hv_i;
    float lv_v;
    float lv_i;
    float va;
    float ia;
    float vb;
    float ib;
    float vc;
    float ic;

    frame = ns800_adc_background_latest(RT_NULL);
    if (frame == RT_NULL)
    {
        *hv_power = 0.0f;
        *lv_power = 0.0f;
        *total_power = 0.0f;
        return;
    }

    hv_v = ns800_adc_dc_voltage(frame[0]);
    hv_i = ns800_adc_dc_current(frame[1]);
    lv_v = ns800_adc_dc_voltage(frame[2]);
    lv_i = ns800_adc_dc_current(frame[3]);
    va = ns800_adc_ac_voltage(frame[4]);
    ia = ns800_adc_ac_current(frame[5]);
    vb = ns800_adc_ac_voltage(frame[6]);
    ib = ns800_adc_ac_current(frame[7]);
    vc = ns800_adc_ac_voltage(frame[8]);
    ic = ns800_adc_ac_current(frame[9]);

    *hv_power = hv_v * hv_i;
    *lv_power = lv_v * lv_i;
    *total_power = (va * ia) + (vb * ib) + (vc * ic);
}

static void ns800_draw_status_page(void)
{
    char line[24];
    char num[12];
    const ns800_motor_app_status_t *motor_status;
    float hv_power;
    float lv_power;
    float total_power;
    float output_power;
    float display_xi;

    ns800_display_sample_power(&hv_power, &lv_power, &total_power);
    motor_status = ns800_motor_app_get_status();
    output_power = motor_status->last_output.output_power_w;
    display_xi = svm_clamp_power_factor(ns800_param_xi);

    ns800_sh1106_oled_clear();
    ns800_sh1106_oled_show_string(0, 0, "HV:");
    ns800_sh1106_oled_show_float(18, 0, hv_power);
    ns800_sh1106_oled_show_string(60, 0, "Output P:");

    ns800_sh1106_oled_show_string(0, 16, "LV:");
    ns800_sh1106_oled_show_float(18, 16, lv_power);
    ns800_sh1106_oled_show_float(60, 16, output_power);

    ns800_sh1106_oled_show_string(0, 32, "xi:");
    ns800_sh1106_oled_show_float(18, 32, display_xi);
    ns800_sh1106_oled_show_string(60, 32, "TP:");
    ns800_sh1106_oled_show_float(78, 32, total_power);

    ns800_i32_to_dec(num, ns800_param_speed);
    line[0] = 'S';
    line[1] = ':';
    rt_strncpy(&line[2], num, sizeof(line) - 2U);
    line[sizeof(line) - 1U] = '\0';
    ns800_sh1106_oled_show_string(0, 48, line);

    ns800_i32_to_dec(num, ns800_param_force_ma);
    line[0] = 'F';
    line[1] = ':';
    rt_strncpy(&line[2], num, sizeof(line) - 5U);
    line[sizeof(line) - 1U] = '\0';
    ns800_append_ma_unit(line, sizeof(line));
    ns800_sh1106_oled_show_string(54, 48, line);

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
