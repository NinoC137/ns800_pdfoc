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
#include "ns800_log.h"
#include "ns800_motor_app.h"
#include "ns800_pwm_app.h"

#include <rtthread.h>

#define NS800_RTT_INIT_THREAD_STACK  1536U
#define NS800_RTT_INIT_THREAD_PRIO   3U
#define NS800_RTT_INIT_THREAD_TICK   20U

static rt_thread_t rtt_init_thread = RT_NULL;

/**
 * @brief RT-Thread 应用设施初始化线程入口。
 *
 * 线程按固定顺序启动 ADC-DMA 背景采样和电机控制 ISR。硬件快路径仍使用
 * SDK/寄存器级实现；该线程只提供确定的启动顺序，完成后直接退出。
 *
 * @param parameter RT-Thread 线程参数，未使用。
 */
static void ns800_rtt_init_thread_entry(void *parameter)
{
    int adc_result;
    int motor_result;

    RT_UNUSED(parameter);

    adc_result = ns800_adc_background_start();
    if (adc_result != 0)
    {
        ns800_log_e("startup", "ns800 adc background start failed: %d", adc_result);
        return;
    }

    ns800_log_i("startup", "ns800 adc background started");

    motor_result = ns800_motor_app_start();
    if (motor_result != 0)
    {
        ns800_log_e("startup", "ns800 motor app start failed: %d", motor_result);
        return;
    }

    ns800_log_i("startup", "ns800 motor app started");
}

/**
 * @brief 创建并启动一次性 RT-Thread 应用初始化线程。
 *
 * @return 0 表示成功；负值表示线程创建失败。
 */
static int ns800_rtt_init_thread_start(void)
{
    if (rtt_init_thread != RT_NULL)
    {
        return 0;
    }

    rtt_init_thread = rt_thread_create("rttinit",
                                       ns800_rtt_init_thread_entry,
                                       RT_NULL,
                                       NS800_RTT_INIT_THREAD_STACK,
                                       NS800_RTT_INIT_THREAD_PRIO,
                                       NS800_RTT_INIT_THREAD_TICK);
    if (rtt_init_thread == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_thread_startup(rtt_init_thread);
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
    ns800_log_i("startup", "NS800RT7P65 application start");

    ns800_button_app_start();
    ns800_display_app_start();
    ns800_pwm_app_start();
    if (ns800_rtt_init_thread_start() != 0)
    {
        ns800_log_e("startup", "rtt init thread create failed");
    }

    ns800_led_app_start();
    ns800_led_app_loop();

    return 0;
}
