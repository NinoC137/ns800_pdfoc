/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_button_app.h"

#include <stdint.h>

#include <board.h>

#include "gpio.h"
#include "multi_button.h"

#define NS800_BUTTON_COUNT          8U
#define NS800_BUTTON_THREAD_STACK   1024U
#define NS800_BUTTON_THREAD_PRIO    4U
#define NS800_BUTTON_THREAD_TICK    10U
#define NS800_BUTTON_SCAN_MS        5U
#define NS800_XI_STEP               (0.05f)
#define NS800_SPEED_STEP            100
#define NS800_FORCE_STEP_MA         50

volatile float ns800_param_xi = 0.5f;
volatile rt_int32_t ns800_param_speed = 2000;
volatile rt_int32_t ns800_param_force_ma = 500;
static volatile rt_uint32_t button_reset_count = 0U;

typedef enum
{
    NS800_BTN_XI_INC = 0,
    NS800_BTN_XI_DEC,
    NS800_BTN_SPEED_INC,
    NS800_BTN_SPEED_DEC,
    NS800_BTN_FORCE_INC,
    NS800_BTN_FORCE_DEC,
    NS800_BTN_RESET,
    NS800_BTN_IO12_13_TOGGLE,
} ns800_button_id_t;

struct ns800_button_pin
{
    GPIO_TypeDef *port;
    GPIO_PinNum pin;
};

static const struct ns800_button_pin button_pins[NS800_BUTTON_COUNT] =
{
    {GPIOA, GPIO_PIN_12},
    {GPIOA, GPIO_PIN_13},
    {GPIOA, GPIO_PIN_14},
    {GPIOA, GPIO_PIN_15},
    {GPIOA, GPIO_PIN_16},
    {GPIOA, GPIO_PIN_17},
    {GPIOA, GPIO_PIN_18},
    {GPIOB, GPIO_PIN_9},
};

static Button buttons[NS800_BUTTON_COUNT];
static rt_thread_t button_thread = RT_NULL;
static rt_bool_t io12_13_uart_mode = RT_TRUE;

/**
 * @brief 将 GPIO12/13 切回 SCIA 串口功能。
 */
static void ns800_io12_13_to_uart(void)
{
    GPIO_setPinConfig(GPIO_12_SCIA_TX);
    GPIO_setPinConfig(GPIO_13_SCIA_RX);

    GPIO_setAnalogMode(GPIO_12, GPIO_ANALOG_DISABLED);
    GPIO_setPadConfig(GPIO_12, GPIO_PIN_TYPE_STD);
    GPIO_setDriveLevel(GPIO_12, GPIO_DRV_LOW);
    GPIO_setPin(GPIO_12);
    GPIO_setDirectionMode(GPIO_12, GPIO_DIR_MODE_OUT);

    GPIO_setAnalogMode(GPIO_13, GPIO_ANALOG_DISABLED);
    GPIO_setPadConfig(GPIO_13, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(GPIO_13, GPIO_QUAL_SYNC);
    GPIO_setDirectionMode(GPIO_13, GPIO_DIR_MODE_IN);

    io12_13_uart_mode = RT_TRUE;
}

/**
 * @brief 将 GPIO12/13 切换为普通按键输入。
 */
static void ns800_io12_13_to_buttons(void)
{
    GPIO_setPinConfig(GPIOA, GPIO_PIN_12, ALT0_FUNCTION);
    GPIO_setPinConfig(GPIOA, GPIO_PIN_13, ALT0_FUNCTION);

    GPIO_setAnalogMode(GPIO_12, GPIO_ANALOG_DISABLED);
    GPIO_setPadConfig(GPIO_12, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(GPIO_12, GPIO_QUAL_SYNC);
    GPIO_setDirectionMode(GPIO_12, GPIO_DIR_MODE_IN);

    GPIO_setAnalogMode(GPIO_13, GPIO_ANALOG_DISABLED);
    GPIO_setPadConfig(GPIO_13, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(GPIO_13, GPIO_QUAL_SYNC);
    GPIO_setDirectionMode(GPIO_13, GPIO_DIR_MODE_IN);

    io12_13_uart_mode = RT_FALSE;
}

/**
 * @brief 恢复按键控制参数默认值。
 *
 * 同时递增 reset 事件计数，供电机控制 ISR 清零速度环和电流环 PI 状态。
 */
void ns800_button_app_reset_params(void)
{
    ns800_param_xi = 0.5f;
    ns800_param_speed = 2000;
    ns800_param_force_ma = 500;
    button_reset_count++;
}

/**
 * @brief 获取 reset 事件累计次数。
 *
 * @return reset 事件计数。
 */
rt_uint32_t ns800_button_app_reset_count(void)
{
    return button_reset_count;
}

/**
 * @brief 读取指定按键当前电平。
 *
 * @param button_id 按键编号。
 * @return GPIO 电平，0 表示按下。
 */
static uint8_t ns800_button_read_level(uint8_t button_id)
{
    const struct ns800_button_pin *pin = &button_pins[button_id];
    return (uint8_t)GPIO_readPin(pin->port, pin->pin);
}

/**
 * @brief 初始化按键 GPIO。
 *
 * GPIO12/13 可被串口复用，因此由独立切换函数处理。
 */
static void ns800_button_gpio_init(void)
{
    rt_uint32_t i;

    for (i = 0U; i < NS800_BUTTON_COUNT; i++)
    {
        if ((i == NS800_BTN_XI_INC) || (i == NS800_BTN_XI_DEC))
        {
            continue;
        }

        GPIO_setPinConfig(button_pins[i].port, button_pins[i].pin, ALT0_FUNCTION);
        GPIO_setAnalogMode(button_pins[i].port, button_pins[i].pin, GPIO_ANALOG_DISABLED);
        GPIO_setPadConfig(button_pins[i].port, button_pins[i].pin, GPIO_PIN_TYPE_PULLUP);
        GPIO_setQualificationMode(button_pins[i].port, button_pins[i].pin, GPIO_QUAL_SYNC);
        GPIO_setDirectionMode(button_pins[i].port, button_pins[i].pin, GPIO_DIR_MODE_IN);
    }
}

/**
 * @brief MultiButton 单击/长按回调。
 *
 * 根据按键编号调整 xi、speed、F 或触发 reset/串口复用切换。
 *
 * @param handle MultiButton 按键句柄。
 * @param user_data 按键编号。
 */
static void ns800_button_click(Button *handle, void *user_data)
{
    rt_uint32_t id = (rt_uint32_t)(uintptr_t)user_data;

    RT_UNUSED(handle);

    // if (((id == NS800_BTN_XI_INC) || (id == NS800_BTN_XI_DEC)) && (io12_13_uart_mode == RT_TRUE))
    // {
    //     return;
    // }

    switch (id)
    {
    case NS800_BTN_XI_INC:
        if(io12_13_uart_mode == RT_FALSE) {
            ns800_param_xi += NS800_XI_STEP;
        }
        break;
    case NS800_BTN_XI_DEC:
        if(io12_13_uart_mode == RT_FALSE) {
            ns800_param_xi -= NS800_XI_STEP;
        }
        break;
    case NS800_BTN_SPEED_INC:
        ns800_param_speed += NS800_SPEED_STEP;
        break;
    case NS800_BTN_SPEED_DEC:
        ns800_param_speed -= NS800_SPEED_STEP;
        break;
    case NS800_BTN_FORCE_INC:
        ns800_param_force_ma += NS800_FORCE_STEP_MA;
        break;
    case NS800_BTN_FORCE_DEC:
        ns800_param_force_ma -= NS800_FORCE_STEP_MA;
        break;
    case NS800_BTN_RESET:
        ns800_button_app_reset_params();
        break;
    case NS800_BTN_IO12_13_TOGGLE:
        if (io12_13_uart_mode == RT_TRUE)
        {
            rt_kprintf("change to button mode\r\n");
            ns800_io12_13_to_buttons();
        }
        else
        {
            ns800_io12_13_to_uart();
            rt_kprintf("change to uart mode\r\n");
        }
        break;
    default:
        break;
    }
}

/**
 * @brief 初始化所有 MultiButton 对象。
 */
static void ns800_button_init_all(void)
{
    rt_uint32_t i;

    ns800_button_gpio_init();
    ns800_io12_13_to_uart();
    for (i = 0U; i < NS800_BUTTON_COUNT; i++)
    {
        button_init(&buttons[i], ns800_button_read_level, 0U, (uint8_t)i);
        button_attach(&buttons[i], BTN_SINGLE_CLICK, ns800_button_click, (void *)(uintptr_t)i);
        button_attach(&buttons[i], BTN_LONG_PRESS_START, ns800_button_click, (void *)(uintptr_t)i);
        button_start(&buttons[i]);
    }
}

/**
 * @brief 按键扫描线程入口。
 *
 * MultiButton 要求周期性调用 button_ticks()，当前心跳为 5 ms。
 *
 * @param parameter RT-Thread 线程参数，未使用。
 */
static void ns800_button_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);

    ns800_button_init_all();
    while (1)
    {
        button_ticks();
        rt_thread_mdelay(NS800_BUTTON_SCAN_MS);
    }
}

/**
 * @brief 启动按键扫描线程。
 *
 * @return 0 表示成功；负值表示线程创建失败。
 */
int ns800_button_app_start(void)
{
    if (button_thread != RT_NULL)
    {
        return 0;
    }

    ns800_button_app_reset_params();
    button_thread = rt_thread_create("buttons",
                                     ns800_button_thread_entry,
                                     RT_NULL,
                                     NS800_BUTTON_THREAD_STACK,
                                     NS800_BUTTON_THREAD_PRIO,
                                     NS800_BUTTON_THREAD_TICK);
    if (button_thread == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_thread_startup(button_thread);
    return 0;
}
