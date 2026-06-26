/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_led_app.h"

#include <board.h>
#include <rtthread.h>

#define NS800_LED1_THREAD_STACK  512U
#define NS800_LED1_THREAD_PRIO   11U
#define NS800_LED1_THREAD_TICK   20U

static rt_thread_t led1_thread = RT_NULL;

static void ns800_led1_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        GPIO_togglePin(BOARD_LED1_PIN);
        rt_thread_mdelay(500);
    }
}

int ns800_led_app_start(void)
{
    if (led1_thread != RT_NULL)
    {
        return 0;
    }

    LED_init();

    led1_thread = rt_thread_create("led1",
                                   ns800_led1_thread_entry,
                                   RT_NULL,
                                   NS800_LED1_THREAD_STACK,
                                   NS800_LED1_THREAD_PRIO,
                                   NS800_LED1_THREAD_TICK);
    if (led1_thread == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_thread_startup(led1_thread);
    return 0;
}

// #include "ns800_adc_background.h"
void ns800_led_app_loop(void)
{
    while (1)
    {
        // ns800_adc_background_dump();
        GPIO_togglePin(BOARD_LED2_PIN);
        rt_thread_mdelay(1000);
    }
}
