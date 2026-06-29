/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_display_app.h"

#include "ns800_adc_background.h"
#include "ns800_button_app.h"
#include "ns800_sh1106_oled.h"

#include <rtthread.h>

#define NS800_DISPLAY_THREAD_STACK  1024U
#define NS800_DISPLAY_THREAD_PRIO   12U
#define NS800_DISPLAY_THREAD_TICK   10U
#define NS800_DISPLAY_REFRESH_MS    100U
#define NS800_DISPLAY_PHASE_I_ZERO  (2048.0f)
#define NS800_DISPLAY_ADC_VREF      (3.3f)
#define NS800_DISPLAY_ADC_FULL      (4095.0f)
#define NS800_DISPLAY_HV_V_GAIN     (1.0f)
#define NS800_DISPLAY_HV_I_GAIN     (0.001f)
#define NS800_DISPLAY_LV_V_GAIN     (1.0f)
#define NS800_DISPLAY_LV_I_GAIN     (0.001f)
#define NS800_DISPLAY_PHASE_V_GAIN  (1.0f)
#define NS800_DISPLAY_PHASE_I_GAIN  (0.001f)

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

static float ns800_adc_voltage(rt_uint16_t raw, float gain)
{
    return ((float)(raw & 0x0fffU)) * NS800_DISPLAY_ADC_VREF * gain / NS800_DISPLAY_ADC_FULL;
}

static float ns800_adc_bipolar_current(rt_uint16_t raw, float gain)
{
    return (((float)(raw & 0x0fffU)) - NS800_DISPLAY_PHASE_I_ZERO) * gain;
}

static float ns800_adc_unipolar_current(rt_uint16_t raw, float gain)
{
    return ((float)(raw & 0x0fffU)) * gain;
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

    hv_v = ns800_adc_voltage(frame[0], NS800_DISPLAY_HV_V_GAIN);
    hv_i = ns800_adc_unipolar_current(frame[1], NS800_DISPLAY_HV_I_GAIN);
    lv_v = ns800_adc_voltage(frame[2], NS800_DISPLAY_LV_V_GAIN);
    lv_i = ns800_adc_unipolar_current(frame[3], NS800_DISPLAY_LV_I_GAIN);
    va = ns800_adc_voltage(frame[4], NS800_DISPLAY_PHASE_V_GAIN);
    ia = ns800_adc_bipolar_current(frame[5], NS800_DISPLAY_PHASE_I_GAIN);
    vb = ns800_adc_voltage(frame[6], NS800_DISPLAY_PHASE_V_GAIN);
    ib = ns800_adc_bipolar_current(frame[7], NS800_DISPLAY_PHASE_I_GAIN);
    vc = ns800_adc_voltage(frame[8], NS800_DISPLAY_PHASE_V_GAIN);
    ic = ns800_adc_bipolar_current(frame[9], NS800_DISPLAY_PHASE_I_GAIN);

    *hv_power = hv_v * hv_i;
    *lv_power = lv_v * lv_i;
    *total_power = (va * ia) + (vb * ib) + (vc * ic);
}

static void ns800_draw_status_page(void)
{
    char line[24];
    char num[12];
    float hv_power;
    float lv_power;
    float total_power;

    ns800_display_sample_power(&hv_power, &lv_power, &total_power);

    ns800_sh1106_oled_clear();
    ns800_sh1106_oled_show_string(0, 0, "HVDC P:");
    ns800_sh1106_oled_show_float(80, 0, hv_power);

    ns800_sh1106_oled_show_string(0, 16, "LVDC P:");
    ns800_sh1106_oled_show_float(80, 16, lv_power);

    ns800_sh1106_oled_show_string(0, 32, "xi:");
    ns800_sh1106_oled_show_float(18, 32, ns800_param_xi);
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
