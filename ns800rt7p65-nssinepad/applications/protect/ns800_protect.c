/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    ns800_protect.c
 * @brief   过流保护链实现：ADC PPB -> ePWM X-BAR -> EPWM TZ + ISR 软件兜底。
 */

#include "ns800_protect.h"

#include <board.h>

#include "ns800_adc_scale.h"
#include "ns800_pwm_app.h"

#ifdef RT_USING_FINSH
#include "finsh.h"
#endif

/** @brief 每 ADC count 对应的电流，单位 A。 */
#define NS800_PROTECT_A_PER_COUNT \
    (NS800_ADC_AC_CURRENT_SCALE / NS800_ADC_FULL_SCALE_COUNT)

/** @brief ADC 12 位满量程。 */
#define NS800_PROTECT_COUNT_MAX        (4095.0f)

static ns800_protect_status_t protect_status;
static rt_bool_t protect_initialized = RT_FALSE;

static float protect_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float protect_max3(float a, float b, float c)
{
    float peak = protect_abs(a);

    if (protect_abs(b) > peak)
    {
        peak = protect_abs(b);
    }
    if (protect_abs(c) > peak)
    {
        peak = protect_abs(c);
    }

    return peak;
}

/**
 * @brief 把峰值电流阈值换算成 ADC counts。
 *
 * @param peak_a 阈值，单位 A。
 * @return counts，已裁剪到 12 位范围。
 */
static rt_uint16_t protect_amp_to_counts(float peak_a)
{
    float counts = peak_a / NS800_PROTECT_A_PER_COUNT;

    if (counts < 0.0f)
    {
        counts = 0.0f;
    }
    if (counts > NS800_PROTECT_COUNT_MAX)
    {
        counts = NS800_PROTECT_COUNT_MAX;
    }

    return (rt_uint16_t)(counts + 0.5f);
}

/**
 * @brief 配置 ADC PPB 限值与事件输出(不使能 X-BAR)。
 */
static void protect_ppb_config(void)
{
    rt_uint16_t half_span;
    rt_uint16_t zero_counts;
    rt_uint16_t high_counts;
    rt_uint16_t low_counts;

    half_span = protect_amp_to_counts(NS800_PROTECT_HW_TRIP_PEAK_A);
    zero_counts = (rt_uint16_t)(NS800_ADC_AC_ZERO_COUNT + 0.5f);

    high_counts = (rt_uint16_t)((rt_uint32_t)zero_counts + half_span);
    if (high_counts > (rt_uint16_t)NS800_PROTECT_COUNT_MAX)
    {
        high_counts = (rt_uint16_t)NS800_PROTECT_COUNT_MAX;
    }
    low_counts = (zero_counts > half_span) ? (rt_uint16_t)(zero_counts - half_span) : 0U;

    ADC_setupPPB(NS800_PROTECT_ADC, NS800_PROTECT_PPB, NS800_PROTECT_PPB_SOC);
    ADC_disablePPBEvent(NS800_PROTECT_ADC, NS800_PROTECT_PPB,
                        (NS800_PROTECT_PPB_EVT_TRIPHI | NS800_PROTECT_PPB_EVT_TRIPLO));
    ADC_disablePPBEventInterrupt(NS800_PROTECT_ADC, NS800_PROTECT_PPB,
                                 (NS800_PROTECT_PPB_EVT_TRIPHI | NS800_PROTECT_PPB_EVT_TRIPLO));
    ADC_setPPBCalibrationOffset(NS800_PROTECT_ADC, NS800_PROTECT_PPB, 0U);
    ADC_setPPBReferenceOffset(NS800_PROTECT_ADC, NS800_PROTECT_PPB, 0U);
    ADC_disablePPBTwosComplement(NS800_PROTECT_ADC, NS800_PROTECT_PPB);

    /* 单次转换就比较：count=1、shift=0、比较源取原始结果。 */
    ADC_setPPBCountLimit(NS800_PROTECT_ADC, NS800_PROTECT_PPB, 1U);
    ADC_setPPBShiftValue(NS800_PROTECT_ADC, NS800_PROTECT_PPB, 0U);
    ADC_selectPPBCompareSource(NS800_PROTECT_ADC, NS800_PROTECT_PPB,
                               ADC_PPB_COMPSOURCE_RESULT);
    ADC_selectPPBSyncInput(NS800_PROTECT_ADC, NS800_PROTECT_PPB, ADC_SYNCIN_DISABLE);

    ADC_setPPBTripLimits(NS800_PROTECT_ADC, NS800_PROTECT_PPB, ADC_PPB_PosSyb,
                         high_counts, low_counts);
    ADC_disablePPBEventCBCClear(NS800_PROTECT_ADC, NS800_PROTECT_PPB);
    ADC_clearPPBEventStatus(NS800_PROTECT_ADC, NS800_PROTECT_PPB,
                            NS800_PROTECT_PPB_EVT_ALL);
    ADC_forcePPBSync(NS800_PROTECT_ADC, NS800_PROTECT_PPB);

    protect_status.ppb_high_counts = high_counts;
    protect_status.ppb_low_counts = low_counts;
}

int ns800_protect_init(void)
{
    if (protect_initialized == RT_TRUE)
    {
        return 0;
    }

    rt_memset(&protect_status, 0, sizeof(protect_status));
    protect_status.hw_trip_peak_a = NS800_PROTECT_HW_TRIP_PEAK_A;
    protect_status.sw_trip_peak_a = NS800_PROTECT_SW_TRIP_PEAK_A;

    protect_ppb_config();
    ns800_pwm_app_trip_config();

    protect_initialized = RT_TRUE;
    ns800_protect_arm((NS800_PROTECT_AUTO_ARM != 0) ? RT_TRUE : RT_FALSE);

    rt_kprintf("protect: ppb1 soc=%u window=[%u,%u] hw=%dmA sw=%dmA armed=%d\r\n",
               (unsigned int)NS800_PROTECT_PPB_SOC,
               (unsigned int)protect_status.ppb_low_counts,
               (unsigned int)protect_status.ppb_high_counts,
               (int)(NS800_PROTECT_HW_TRIP_PEAK_A * 1000.0f),
               (int)(NS800_PROTECT_SW_TRIP_PEAK_A * 1000.0f),
               (int)ns800_protect_is_armed());

    return 0;
}

void ns800_protect_arm(rt_bool_t enabled)
{
    if (enabled == RT_TRUE)
    {
        ADC_clearPPBEventStatus(NS800_PROTECT_ADC, NS800_PROTECT_PPB,
                                (NS800_PROTECT_PPB_EVT_TRIPHI | NS800_PROTECT_PPB_EVT_TRIPLO));
        XBAR_setEPWMMuxConfig(XBAR, NS800_PROTECT_XBAR_TRIP, NS800_PROTECT_XBAR_MUX_SIGNAL);
        XBAR_enableEPWMMux(XBAR, NS800_PROTECT_XBAR_TRIP, NS800_PROTECT_XBAR_MUX);
        ADC_enablePPBEvent(NS800_PROTECT_ADC, NS800_PROTECT_PPB,
                           (NS800_PROTECT_PPB_EVT_TRIPHI | NS800_PROTECT_PPB_EVT_TRIPLO));
        protect_status.flags |= NS800_PROTECT_FLAG_ARMED;
    }
    else
    {
        ADC_disablePPBEvent(NS800_PROTECT_ADC, NS800_PROTECT_PPB,
                            (NS800_PROTECT_PPB_EVT_TRIPHI | NS800_PROTECT_PPB_EVT_TRIPLO));
        XBAR_disableEPWMMux(XBAR, NS800_PROTECT_XBAR_TRIP, NS800_PROTECT_XBAR_MUX);
        protect_status.flags &= ~NS800_PROTECT_FLAG_ARMED;
    }
}

rt_bool_t ns800_protect_is_armed(void)
{
    return ((protect_status.flags & NS800_PROTECT_FLAG_ARMED) != 0U) ? RT_TRUE : RT_FALSE;
}

rt_bool_t ns800_protect_check_currents(float ia, float ib, float ic)
{
    float peak;

    if ((protect_status.flags & NS800_PROTECT_FLAG_SW_TRIP) != 0U)
    {
        return RT_FALSE;
    }

    peak = protect_max3(ia, ib, ic);
    if (peak <= NS800_PROTECT_SW_TRIP_PEAK_A)
    {
        return RT_FALSE;
    }

    protect_status.flags |= NS800_PROTECT_FLAG_SW_TRIP;
    protect_status.sw_trip_count++;
    ns800_pwm_app_trip_force();
    return RT_TRUE;
}

rt_bool_t ns800_protect_is_tripped(void)
{
    if ((protect_status.flags & NS800_PROTECT_FLAG_SW_TRIP) != 0U)
    {
        return RT_TRUE;
    }
    if (ns800_pwm_app_trip_active() == RT_TRUE)
    {
        if ((protect_status.flags & NS800_PROTECT_FLAG_HW_TRIP) == 0U)
        {
            protect_status.hw_trip_count++;
        }
        protect_status.flags |= NS800_PROTECT_FLAG_HW_TRIP;
        return RT_TRUE;
    }

    return RT_FALSE;
}

void ns800_protect_clear(void)
{
    /* 顺序很重要：先清 ADC 侧的事件锁存，再清 EPWM 的 TZ 锁存；否则 ADCAEVT1
     * 仍为高时清 TZ 会被立刻重新触发。若过流仍然存在，下一次转换会再次跳闸。 */
    ADC_clearPPBEventStatus(NS800_PROTECT_ADC, NS800_PROTECT_PPB,
                            NS800_PROTECT_PPB_EVT_ALL);
    ns800_pwm_app_trip_clear();

    protect_status.flags &= (rt_uint32_t)NS800_PROTECT_FLAG_ARMED;
}

const ns800_protect_status_t *ns800_protect_get_status(void)
{
    return &protect_status;
}

#ifdef RT_USING_FINSH
static int protect_status_cmd(int argc, char **argv)
{
    const ns800_protect_status_t *st = &protect_status;

    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("protect: flags=0x%02x armed=%d tripped=%d hw_cnt=%u sw_cnt=%u\r\n",
               (unsigned int)st->flags,
               (int)ns800_protect_is_armed(),
               (int)ns800_protect_is_tripped(),
               (unsigned int)st->hw_trip_count,
               (unsigned int)st->sw_trip_count);
    rt_kprintf("ppb: soc=%u window=[%u,%u] counts, hw=%dmA sw=%dmA\r\n",
               (unsigned int)NS800_PROTECT_PPB_SOC,
               (unsigned int)st->ppb_low_counts,
               (unsigned int)st->ppb_high_counts,
               (int)(st->hw_trip_peak_a * 1000.0f),
               (int)(st->sw_trip_peak_a * 1000.0f));
    return 0;
}

static int protect_arm_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("usage: protect_arm <on|off>\r\n");
        return -1;
    }

    if (rt_strcmp(argv[1], "on") == 0)
    {
        ns800_protect_arm(RT_TRUE);
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        ns800_protect_arm(RT_FALSE);
    }
    else
    {
        rt_kprintf("usage: protect_arm <on|off>\r\n");
        return -1;
    }

    rt_kprintf("protect: armed=%d\r\n", (int)ns800_protect_is_armed());
    return 0;
}

static int protect_clear_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    ns800_protect_clear();
    rt_kprintf("protect: cleared\r\n");
    return 0;
}

MSH_CMD_EXPORT_ALIAS(protect_status_cmd, protect_status, show over-current protection status);
MSH_CMD_EXPORT_ALIAS(protect_arm_cmd, protect_arm, arm hardware trip: protect_arm on|off);
MSH_CMD_EXPORT_ALIAS(protect_clear_cmd, protect_clear, clear latched trip flags);
#endif /* RT_USING_FINSH */
