/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_app.h"

#include "ns800_adc_background.h"
#include "ns800_button_app.h"
#include "ns800_display_app.h"
#include "ns800_led_app.h"
#include "ns800_pwm_app.h"
#include "ns800_power_control.h"

#include <rtthread.h>

#define NS800_ADC_BG_THREAD_STACK  1024U
#define NS800_ADC_BG_THREAD_PRIO   3U
#define NS800_ADC_BG_THREAD_TICK   20U
#define NS800_POWER_CTRL_STACK     1024U
#define NS800_POWER_CTRL_PRIO      4U
#define NS800_POWER_CTRL_TICK      10U

static rt_thread_t adc_bg_thread = RT_NULL;
static rt_thread_t power_ctrl_thread = RT_NULL;

static void ns800_adc_bg_thread_entry(void *parameter)
{
    int result;

    RT_UNUSED(parameter);

    result = ns800_adc_background_start();
    rt_kprintf("ns800 adc background start: %d\r\n", result);

    while (1)
    {
        rt_thread_mdelay(1000);
    }
}

static int ns800_adc_bg_thread_start(void)
{
    if (adc_bg_thread != RT_NULL)
    {
        return 0;
    }

    adc_bg_thread = rt_thread_create("adc_bg",
                                     ns800_adc_bg_thread_entry,
                                     RT_NULL,
                                     NS800_ADC_BG_THREAD_STACK,
                                     NS800_ADC_BG_THREAD_PRIO,
                                     NS800_ADC_BG_THREAD_TICK);
    if (adc_bg_thread == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_thread_startup(adc_bg_thread);
    return 0;
}

static void ns800_power_ctrl_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);
    ns800_power_control_start();
    while (1)
    {
        rt_thread_mdelay(1000);
    }
}

static int ns800_power_ctrl_thread_start(void)
{
    if (power_ctrl_thread != RT_NULL)
    {
        return 0;
    }

    power_ctrl_thread = rt_thread_create("pwr_ctrl",
                                         ns800_power_ctrl_thread_entry,
                                         RT_NULL,
                                         NS800_POWER_CTRL_STACK,
                                         NS800_POWER_CTRL_PRIO,
                                         NS800_POWER_CTRL_TICK);
    if (power_ctrl_thread == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_thread_startup(power_ctrl_thread);
    return 0;
}

int ns800_app_main(void)
{
    rt_kprintf("NS800RT7P65 application start\r\n");

    // ns800_button_app_start();
    ns800_display_app_start();
    ns800_pwm_app_start();
    ns800_adc_bg_thread_start();
    ns800_power_ctrl_thread_start();
    
    ns800_led_app_start();
    ns800_led_app_loop();

    return 0;
}
