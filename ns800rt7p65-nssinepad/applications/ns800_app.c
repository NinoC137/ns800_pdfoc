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
#include "ns800_motor_app.h"
#include "ns800_pwm_app.h"

#include <rtthread.h>

#define NS800_ADC_BG_THREAD_STACK  1024U
#define NS800_ADC_BG_THREAD_PRIO   3U
#define NS800_ADC_BG_THREAD_TICK   20U
#define NS800_MOTOR_CTRL_STACK     1024U
#define NS800_MOTOR_CTRL_PRIO      4U
#define NS800_MOTOR_CTRL_TICK      10U

static rt_thread_t adc_bg_thread = RT_NULL;
static rt_thread_t motor_ctrl_thread = RT_NULL;

/**
 * @brief ADC 后台线程入口。
 *
 * 线程只负责启动循环 ADC + DMA，启动完成后低频休眠，采样由 EPWM1/ADC/DMA
 * 硬件链路持续完成。
 *
 * @param parameter RT-Thread 线程参数，未使用。
 */
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

/**
 * @brief 创建并启动 ADC 后台线程。
 *
 * @return 0 表示成功；负值表示线程创建失败。
 */
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

/**
 * @brief 电机控制线程入口。
 *
 * 线程只负责初始化电机应用层和 EPWM2 控制中断，实时控制计算在 ISR 中完成。
 *
 * @param parameter RT-Thread 线程参数，未使用。
 */
static void ns800_motor_ctrl_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);
    ns800_motor_app_start();
    while (1)
    {
        rt_thread_mdelay(1000);
    }
}

/**
 * @brief 创建并启动电机控制初始化线程。
 *
 * @return 0 表示成功；负值表示线程创建失败。
 */
static int ns800_motor_ctrl_thread_start(void)
{
    if (motor_ctrl_thread != RT_NULL)
    {
        return 0;
    }

    motor_ctrl_thread = rt_thread_create("motor",
                                         ns800_motor_ctrl_thread_entry,
                                         RT_NULL,
                                         NS800_MOTOR_CTRL_STACK,
                                         NS800_MOTOR_CTRL_PRIO,
                                         NS800_MOTOR_CTRL_TICK);
    if (motor_ctrl_thread == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_thread_startup(motor_ctrl_thread);
    return 0;
}

/**
 * @brief NS800 应用主入口。
 *
 * 启动按键、OLED、PWM、ADC 后台采样和电机控制 ISR，最后进入 LED2 主循环。
 *
 * @return 理论上不返回；保留 0 作为 RT-Thread main 兼容返回值。
 */
int ns800_app_main(void)
{
    rt_kprintf("NS800RT7P65 application start\r\n");

    ns800_button_app_start();
    ns800_display_app_start();
    ns800_pwm_app_start();
    ns800_adc_bg_thread_start();
    ns800_motor_ctrl_thread_start();

    ns800_led_app_start();
    ns800_led_app_loop();

    return 0;
}
