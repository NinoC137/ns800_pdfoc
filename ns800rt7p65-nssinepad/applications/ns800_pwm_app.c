/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_pwm_app.h"

#include <stdlib.h>

#include <board.h>

#include "epwm.h"
#include "gpio.h"
#include "syscon.h"

#ifdef RT_USING_FINSH
#include "finsh.h"
#endif

#define NS800_PWM_APP_EPWM_COUNT  6U

struct ns800_pwm_pin
{
    GPIO_TypeDef *port;
    GPIO_PinNum pin;
    GPIO_AltFunc af;
};

struct ns800_pwm_module
{
    EPWM_TypeDef *epwm;
    struct ns800_pwm_pin pin_a;
    struct ns800_pwm_pin pin_b;
};

static const struct ns800_pwm_module pwm_modules[NS800_PWM_APP_EPWM_COUNT] =
{
    {EPWM8,  {GPIOC, GPIO_PIN_10, ALT1_FUNCTION}, {GPIOC, GPIO_PIN_11, ALT1_FUNCTION}}, /* GPIO74/75 */
    {EPWM9,  {GPIOC, GPIO_PIN_12, ALT1_FUNCTION}, {GPIOC, GPIO_PIN_13, ALT1_FUNCTION}}, /* GPIO76/77 */
    {EPWM10, {GPIOC, GPIO_PIN_14, ALT1_FUNCTION}, {GPIOC, GPIO_PIN_15, ALT1_FUNCTION}}, /* GPIO78/79 */
    {EPWM11, {GPIOC, GPIO_PIN_16, ALT1_FUNCTION}, {GPIOC, GPIO_PIN_17, ALT1_FUNCTION}}, /* GPIO80/81 */
    {EPWM12, {GPIOC, GPIO_PIN_18, ALT1_FUNCTION}, {GPIOC, GPIO_PIN_19, ALT1_FUNCTION}}, /* GPIO82/83 */
    {EPWM13, {GPIOC, GPIO_PIN_21, ALT1_FUNCTION}, {GPIOC, GPIO_PIN_22, ALT1_FUNCTION}}, /* GPIO85/86 */
};

static rt_uint32_t pwm_duty[NS800_PWM_APP_EPWM_COUNT][2];

static rt_uint16_t pwm_cmp_from_permille(rt_uint32_t duty_permille)
{
    return (rt_uint16_t)(((rt_uint64_t)NS800_PWM_APP_TBPRD_30KHZ *
                          (NS800_PWM_APP_DUTY_PERMILLE_MAX - duty_permille) +
                          (NS800_PWM_APP_DUTY_PERMILLE_MAX / 2U)) /
                         NS800_PWM_APP_DUTY_PERMILLE_MAX);
}

static rt_uint16_t pwm_cmp_from_unit(float duty_unit)
{
    if (duty_unit < 0.0f)
    {
        duty_unit = 0.0f;
    }
    else if (duty_unit > 1.0f)
    {
        duty_unit = 1.0f;
    }

    return (rt_uint16_t)((rt_uint32_t)(NS800_PWM_APP_TBPRD_30KHZ * (1.0f - duty_unit)));
}

static void pwm_gpio_init(const struct ns800_pwm_pin *pin)
{
    GPIO_setPinConfig(pin->port, pin->pin, pin->af);
    GPIO_setAnalogMode(pin->port, pin->pin, GPIO_ANALOG_DISABLED);
    GPIO_setPadConfig(pin->port, pin->pin, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(pin->port, pin->pin, GPIO_QUAL_SYNC);
    GPIO_setDirectionMode(pin->port, pin->pin, GPIO_DIR_MODE_OUT);
}

static void pwm_module_init(rt_uint32_t index)
{
    const struct ns800_pwm_module *module = &pwm_modules[index];
    rt_uint16_t cmp = pwm_cmp_from_permille(NS800_PWM_APP_DEFAULT_DUTY);

    pwm_gpio_init(&module->pin_a);
    pwm_gpio_init(&module->pin_b);

    EPWM_setTimeBaseCounterMode(module->epwm, EPWM_COUNTER_MODE_STOP_FREEZE);
    EPWM_setPeriodLoadMode(module->epwm, EPWM_PERIOD_DIRECT_LOAD);
    EPWM_setClockPrescaler(module->epwm, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_disablePhaseShiftLoad(module->epwm);
    EPWM_setPhaseShift(module->epwm, 0U);
    EPWM_setTimeBaseCounter(module->epwm, 0U);
    EPWM_setTimeBasePeriod(module->epwm, NS800_PWM_APP_TBPRD_30KHZ);
    EPWM_setCounterCompareShadowLoadMode(module->epwm, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(module->epwm, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(module->epwm, EPWM_COUNTER_COMPARE_A, cmp);
    EPWM_setCounterCompareValue(module->epwm, EPWM_COUNTER_COMPARE_B, cmp);
    EPWM_setActionQualifierAction(module->epwm, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(module->epwm, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(module->epwm, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(module->epwm, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(module->epwm, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(module->epwm, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(module->epwm, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(module->epwm, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    EPWM_setCounterCompareValue(module->epwm, EPWM_COUNTER_COMPARE_A, cmp);
    EPWM_setCounterCompareValue(module->epwm, EPWM_COUNTER_COMPARE_B, cmp);
    EPWM_setTimeBaseCounterMode(module->epwm, EPWM_COUNTER_MODE_UP_DOWN);

    pwm_duty[index][0] = NS800_PWM_APP_DEFAULT_DUTY;
    pwm_duty[index][1] = NS800_PWM_APP_DEFAULT_DUTY;
}

static int pwm_index(rt_uint32_t epwm_id)
{
    return ((epwm_id >= NS800_PWM_APP_MIN_EPWM_ID) &&
            (epwm_id <= NS800_PWM_APP_MAX_EPWM_ID)) ?
           (int)(epwm_id - NS800_PWM_APP_MIN_EPWM_ID) : -1;
}

int ns800_pwm_app_start(void)
{
    rt_uint32_t i;

    SYSCON_UNLOCK;
    SYSCON_setTbClkSync(SYSCON, false);
    SYSCON_LOCK;

    for (i = 0U; i < NS800_PWM_APP_EPWM_COUNT; i++)
    {
        pwm_module_init(i);
    }

    SYSCON_UNLOCK;
    SYSCON_setTbClkSync(SYSCON, true);
    SYSCON_LOCK;

    return 0;
}

void ns800_pwm_app_write_svm(const svm_dual_out_t *duty)
{
    rt_uint16_t cmp;

    if (duty == RT_NULL)
    {
        return;
    }

    /* Phase A: EPWM8 upper, EPWM9 lower. */
    cmp = pwm_cmp_from_unit(duty->upper.ta);
    EPWM_setCounterCompareValue(EPWM8, EPWM_COUNTER_COMPARE_A, cmp);
    EPWM_setCounterCompareValue(EPWM8, EPWM_COUNTER_COMPARE_B, cmp);
    cmp = pwm_cmp_from_unit(duty->lower.ta);
    EPWM_setCounterCompareValue(EPWM9, EPWM_COUNTER_COMPARE_A, cmp);
    EPWM_setCounterCompareValue(EPWM9, EPWM_COUNTER_COMPARE_B, cmp);

    /* Phase B: EPWM10 upper, EPWM11 lower. */
    cmp = pwm_cmp_from_unit(duty->upper.tb);
    EPWM_setCounterCompareValue(EPWM10, EPWM_COUNTER_COMPARE_A, cmp);
    EPWM_setCounterCompareValue(EPWM10, EPWM_COUNTER_COMPARE_B, cmp);
    cmp = pwm_cmp_from_unit(duty->lower.tb);
    EPWM_setCounterCompareValue(EPWM11, EPWM_COUNTER_COMPARE_A, cmp);
    EPWM_setCounterCompareValue(EPWM11, EPWM_COUNTER_COMPARE_B, cmp);

    /* Phase C: EPWM12 upper, EPWM13 lower. */
    cmp = pwm_cmp_from_unit(duty->upper.tc);
    EPWM_setCounterCompareValue(EPWM12, EPWM_COUNTER_COMPARE_A, cmp);
    EPWM_setCounterCompareValue(EPWM12, EPWM_COUNTER_COMPARE_B, cmp);
    cmp = pwm_cmp_from_unit(duty->lower.tc);
    EPWM_setCounterCompareValue(EPWM13, EPWM_COUNTER_COMPARE_A, cmp);
    EPWM_setCounterCompareValue(EPWM13, EPWM_COUNTER_COMPARE_B, cmp);
}

int ns800_pwm_app_stop(void)
{
    rt_uint32_t i;

    for (i = 0U; i < NS800_PWM_APP_EPWM_COUNT; i++)
    {
        EPWM_setTimeBaseCounterMode(pwm_modules[i].epwm, EPWM_COUNTER_MODE_STOP_FREEZE);
        EPWM_setCounterCompareValue(pwm_modules[i].epwm, EPWM_COUNTER_COMPARE_A, NS800_PWM_APP_TBPRD_30KHZ);
        EPWM_setCounterCompareValue(pwm_modules[i].epwm, EPWM_COUNTER_COMPARE_B, NS800_PWM_APP_TBPRD_30KHZ);
    }

    return 0;
}

int ns800_pwm_app_set_duty(rt_uint32_t epwm_id, rt_uint32_t channel, rt_uint32_t duty_permille)
{
    int index = pwm_index(epwm_id);
    const struct ns800_pwm_module *module;
    rt_uint16_t cmp;

    if ((index < 0) ||
        ((channel != NS800_PWM_APP_CHANNEL_A) && (channel != NS800_PWM_APP_CHANNEL_B)) ||
        (duty_permille > NS800_PWM_APP_DUTY_PERMILLE_MAX))
    {
        return -RT_EINVAL;
    }

    module = &pwm_modules[index];
    pwm_duty[index][channel - 1U] = duty_permille;

    if ((duty_permille == 0U) || (duty_permille == NS800_PWM_APP_DUTY_PERMILLE_MAX))
    {
        return 0;
    }

    cmp = pwm_cmp_from_permille(duty_permille);
    if (channel == NS800_PWM_APP_CHANNEL_B)
    {
        EPWM_setCounterCompareValue(module->epwm, EPWM_COUNTER_COMPARE_B, cmp);
    }
    else
    {
        EPWM_setCounterCompareValue(module->epwm, EPWM_COUNTER_COMPARE_A, cmp);
    }

    return 0;
}

rt_uint32_t ns800_pwm_app_get_duty(rt_uint32_t epwm_id, rt_uint32_t channel)
{
    int index = pwm_index(epwm_id);

    if ((index < 0) || ((channel != NS800_PWM_APP_CHANNEL_A) && (channel != NS800_PWM_APP_CHANNEL_B)))
    {
        return 0U;
    }

    return pwm_duty[index][channel - 1U];
}

#ifdef RT_USING_FINSH
static int pwm_duty_cmd(int argc, char **argv)
{
    rt_uint32_t epwm_id;
    rt_uint32_t channel;
    rt_uint32_t duty;
    int result;

    if (argc != 4)
    {
        rt_kprintf("usage: pwm_duty <8-13> <a|b|1|2> <0-1000>\r\n");
        return -RT_EINVAL;
    }

    epwm_id = (rt_uint32_t)strtoul(argv[1], RT_NULL, 0);
    channel = ((argv[2][0] == 'b') || (argv[2][0] == 'B') || (argv[2][0] == '2')) ?
              NS800_PWM_APP_CHANNEL_B : NS800_PWM_APP_CHANNEL_A;
    duty = (rt_uint32_t)strtoul(argv[3], RT_NULL, 0);

    result = ns800_pwm_app_set_duty(epwm_id, channel, duty);
    rt_kprintf("pwm_duty: epwm%u %c duty=%u result=%d\r\n",
               (unsigned int)epwm_id,
               (channel == NS800_PWM_APP_CHANNEL_B) ? 'B' : 'A',
               (unsigned int)duty,
               result);
    return result;
}

MSH_CMD_EXPORT_ALIAS(pwm_duty_cmd, pwm_duty, set ns800 pwm duty in permille);
#endif
