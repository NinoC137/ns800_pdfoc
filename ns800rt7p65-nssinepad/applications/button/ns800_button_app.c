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

#define NS800_BUTTON_COUNT          7U
#define NS800_BUTTON_THREAD_STACK   1024U
#define NS800_BUTTON_THREAD_PRIO    5U
#define NS800_BUTTON_THREAD_TICK    10U
#define NS800_BUTTON_SCAN_MS        5U
#define NS800_XI_STEP               (0.05f)
#define NS800_SPEED_STEP            100
#define NS800_FORCE_STEP_MA         50

volatile float ns800_param_xi = 0.5f;
volatile rt_int32_t ns800_param_speed = 2000;
volatile rt_int32_t ns800_param_force_ma = 500;

typedef enum
{
    NS800_BTN_XI_INC = 0,
    NS800_BTN_XI_DEC,
    NS800_BTN_SPEED_INC,
    NS800_BTN_SPEED_DEC,
    NS800_BTN_FORCE_INC,
    NS800_BTN_FORCE_DEC,
    NS800_BTN_RESET,
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
};

static Button buttons[NS800_BUTTON_COUNT];
static rt_thread_t button_thread = RT_NULL;

void ns800_button_app_reset_params(void)
{
    ns800_param_xi = 0.5f;
    ns800_param_speed = 2000;
    ns800_param_force_ma = 500;
}

static uint8_t ns800_button_read_level(uint8_t button_id)
{
    const struct ns800_button_pin *pin = &button_pins[button_id];
    return (uint8_t)GPIO_readPin(pin->port, pin->pin);
}

static void ns800_button_gpio_init(void)
{
    rt_uint32_t i;

    for (i = 0U; i < NS800_BUTTON_COUNT; i++)
    {
        GPIO_setPinConfig(button_pins[i].port, button_pins[i].pin, ALT0_FUNCTION);
        GPIO_setAnalogMode(button_pins[i].port, button_pins[i].pin, GPIO_ANALOG_DISABLED);
        GPIO_setPadConfig(button_pins[i].port, button_pins[i].pin, GPIO_PIN_TYPE_PULLUP);
        GPIO_setQualificationMode(button_pins[i].port, button_pins[i].pin, GPIO_QUAL_SYNC);
        GPIO_setDirectionMode(button_pins[i].port, button_pins[i].pin, GPIO_DIR_MODE_IN);
    }
}

static void ns800_button_click(Button *handle, void *user_data)
{
    rt_uint32_t id = (rt_uint32_t)(uintptr_t)user_data;

    RT_UNUSED(handle);

    switch (id)
    {
    case NS800_BTN_XI_INC:
        ns800_param_xi += NS800_XI_STEP;
        break;
    case NS800_BTN_XI_DEC:
        ns800_param_xi -= NS800_XI_STEP;
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
    default:
        break;
    }
}

static void ns800_button_init_all(void)
{
    rt_uint32_t i;

    ns800_button_gpio_init();
    for (i = 0U; i < NS800_BUTTON_COUNT; i++)
    {
        button_init(&buttons[i], ns800_button_read_level, 0U, (uint8_t)i);
        button_attach(&buttons[i], BTN_SINGLE_CLICK, ns800_button_click, (void *)(uintptr_t)i);
        button_attach(&buttons[i], BTN_LONG_PRESS_START, ns800_button_click, (void *)(uintptr_t)i);
        button_start(&buttons[i]);
    }
}

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
