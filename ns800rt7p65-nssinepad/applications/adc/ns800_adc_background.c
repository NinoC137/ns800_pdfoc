/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_adc_background.h"

#include <stdint.h>

#include <board.h>

#include "adc.h"
#include "dmamux.h"
#include "edma.h"
#include "epwm.h"
#include "gpio.h"
#include "syscon.h"

#define NS800_ADC_BG_DMA_CHANNEL   1U
#define NS800_ADC_BG_CACHE_BYTES   32U
#define NS800_ADC_BG_SAMPLE_WIN    8U
#define NS800_ADC_BG_EPWM_PERIOD   155U
#define NS800_ADC_BG_EPWM_COMPARE  78U

struct ns800_adc_bg_pin
{
    GPIO_TypeDef *port;
    GPIO_PinNum pin;
};

static const struct ns800_adc_bg_pin adc_bg_pins[NS800_ADC_BACKGROUND_CHANNELS] =
{
    {GPIOG, GPIO_PIN_6},
    {GPIOG, GPIO_PIN_7},
    {GPIOG, GPIO_PIN_8},
    {GPIOG, GPIO_PIN_9},
    {GPIOG, GPIO_PIN_10},
    {GPIOG, GPIO_PIN_11},
    {GPIOG, GPIO_PIN_12},
    {GPIOG, GPIO_PIN_13},
    {GPIOG, GPIO_PIN_14},
    {GPIOH, GPIO_PIN_13},
};

static rt_uint16_t adc_bg_frames[NS800_ADC_BACKGROUND_FRAME_COUNT][NS800_ADC_BACKGROUND_CHANNELS]
    __attribute__((aligned(32)));
static rt_uint16_t adc_bg_zero_frame[NS800_ADC_BACKGROUND_CHANNELS]
    __attribute__((aligned(32)));

static volatile rt_bool_t adc_bg_running = RT_FALSE;

static void ns800_adc_bg_gpio_init(void)
{
    rt_uint32_t index;

    for (index = 0U; index < NS800_ADC_BACKGROUND_CHANNELS; index++)
    {
        GPIO_setPinConfig(adc_bg_pins[index].port,
                          adc_bg_pins[index].pin,
                          ALT0_FUNCTION);
        GPIO_setAnalogMode(adc_bg_pins[index].port,
                           adc_bg_pins[index].pin,
                           GPIO_ANALOG_ENABLED);
        GPIO_setPadConfig(adc_bg_pins[index].port,
                          adc_bg_pins[index].pin,
                          GPIO_PIN_TYPE_STD);
        GPIO_setQualificationMode(adc_bg_pins[index].port,
                                  adc_bg_pins[index].pin,
                                  GPIO_QUAL_ASYNC);
        GPIO_setDirectionMode(adc_bg_pins[index].port,
                              adc_bg_pins[index].pin,
                              GPIO_DIR_MODE_IN);
    }
}

static void ns800_adc_bg_adc_init(void)
{
    ADC_setVREF(ADCA, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setPrescaler(ADCA, ADC_CLK_DIV_4);
    ADC_setInterruptPulsePosMode(ADCA, ADC_PULSE_END_OF_CONV);
    ADC_enableConverter(ADCA);

    ADC_disableBurstMode(ADCA);
    ADC_setSOCPriority(ADCA, ADC_PRI_ALL_ROUND_ROBIN);
    ADC_disableInterrupt(ADCA, ADC_INT_NUMBER1);
    ADC_disableContinuousMode(ADCA, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(ADCA, ADC_INT_NUMBER1);
}

static void ns800_adc_bg_config_soc(void)
{
    rt_uint32_t index;

    for (index = 0U; index < NS800_ADC_BACKGROUND_CHANNELS; index++)
    {
        ADC_setupSOC(ADCA,
                     (ADC_SOCNumber)index,
                     ADC_TRIGGER_EPWM1_SOCA,
                     (ADC_Channel)index,
                     NS800_ADC_BG_SAMPLE_WIN);
        ADC_setInterruptSOCTrigger(ADCA,
                                   (ADC_SOCNumber)index,
                                   ADC_INT_SOC_TRIGGER_NONE);
    }
}

static void ns800_adc_bg_set_adc_dma_request(rt_bool_t enabled)
{
    ADCA->DMA.BIT.INT1DMAEN = enabled ? 1U : 0U;
}

static void ns800_adc_bg_dma_clear(void)
{
    EDMA_clearChannelStatusIntFlags(EDMA1, NS800_ADC_BG_DMA_CHANNEL);
    EDMA_clearChannelStatusDoneFlags(EDMA1, NS800_ADC_BG_DMA_CHANNEL);
    EDMA_clearChannelStatusErrorFlags(EDMA1, NS800_ADC_BG_DMA_CHANNEL);
}

static void ns800_adc_bg_dma_stop(void)
{
    ns800_adc_bg_set_adc_dma_request(RT_FALSE);
    ADC_disableInterrupt(ADCA, ADC_INT_NUMBER1);
    ADC_disableContinuousMode(ADCA, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(ADCA, ADC_INT_NUMBER1);

    DMAMUX_disableChannel(DMAMUX1, NS800_ADC_BG_DMA_CHANNEL);
    EDMA_disableChannelRequest(EDMA1, NS800_ADC_BG_DMA_CHANNEL);
    EDMA_disableChannelInterrupts(EDMA1,
                                  NS800_ADC_BG_DMA_CHANNEL,
                                  EDMA_MAJORINTERRUPTENABLE | EDMA_ERRORINTERRUPTENABLE);
    ns800_adc_bg_dma_clear();
}

static void ns800_adc_bg_dma_config(void)
{
    EDMA_CommonConfig common = {0};
    EDMA_TransferConfig transfer = {0};
    EDMA_MinorOffsetConfig minor = {0};

    common.enableHaltOnError = true;
    common.enableEmlm = true;

    transfer.channel = NS800_ADC_BG_DMA_CHANNEL;
    transfer.srcAddr = (uint32_t)&ADCARESULT->RESULT0.WORDVAL;
    transfer.destAddr = EDMA_getAbsAddrForMultiCore((uint32_t)adc_bg_frames[0]);
    transfer.srcTransferSize = EDMA_TRANSFERSIZE2BYTES;
    transfer.destTransferSize = EDMA_TRANSFERSIZE2BYTES;
    transfer.srcOffset = 4;
    transfer.destOffset = 2;
    transfer.minorLoopBytes = NS800_ADC_BACKGROUND_CHANNELS * sizeof(rt_uint16_t);
    transfer.majorLoopCounts = NS800_ADC_BACKGROUND_FRAME_COUNT;
    transfer.slast = -((int32_t)NS800_ADC_BACKGROUND_CHANNELS * 4);
    transfer.dlast = -((int32_t)NS800_ADC_BACKGROUND_CHANNELS *
                       (int32_t)NS800_ADC_BACKGROUND_FRAME_COUNT *
                       (int32_t)sizeof(rt_uint16_t));
    transfer.enMajorInt = false;
    transfer.enDreq = false;
    transfer.enTrigger = true;
    transfer.enErrInt = true;
    transfer.startMode = true;

    EDMA_initialize(EDMA1);
    DMAMUX_configModule(DMAMUX1);
    DMAMUX_setSource(DMAMUX1, NS800_ADC_BG_DMA_CHANNEL, DMAMUX_ADCA1_REQ);
    DMAMUX_enableChannel(DMAMUX1, NS800_ADC_BG_DMA_CHANNEL);

    EDMA_configModule(EDMA1, &common);
    EDMA_configChannel(EDMA1, &transfer);

    minor.enableSrcMinorOffset = true;
    minor.enableDestMinorOffset = false;
    minor.minorOffset = -((int32_t)NS800_ADC_BACKGROUND_CHANNELS * 4);
    EDMA_setMinorOffsetConfig(EDMA1, NS800_ADC_BG_DMA_CHANNEL, &minor);

    ADC_clearInterruptStatus(ADCA, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(ADCA, ADC_INT_NUMBER1);
    ADC_setInterruptSource(ADCA, ADC_INT_NUMBER1, ADC_SOC_NUMBER9);
    ADC_enableContinuousMode(ADCA, ADC_INT_NUMBER1);
    ADC_enableInterrupt(ADCA, ADC_INT_NUMBER1);
    ns800_adc_bg_set_adc_dma_request(RT_TRUE);
}

static void ns800_adc_bg_epwm_stop(void)
{
    EPWM_disableADCTrigger(EPWM1, EPWM_SOC_A);
    EPWM_setTimeBaseCounterMode(EPWM1, EPWM_COUNTER_MODE_STOP_FREEZE);
}

static void ns800_adc_bg_epwm_start(void)
{
    SYSCON_UNLOCK;
    SYSCON_setTbClkSync(SYSCON, false);
    SYSCON_LOCK;

    EPWM_disableADCTrigger(EPWM1, EPWM_SOC_A);
    EPWM_setADCTriggerSource(EPWM1, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerEventPrescale(EPWM1, EPWM_SOC_A, 1U);
    EPWM_setTimeBasePeriod(EPWM1, NS800_ADC_BG_EPWM_PERIOD);
    EPWM_setCounterCompareValue(EPWM1, EPWM_COUNTER_COMPARE_A, NS800_ADC_BG_EPWM_COMPARE);
    EPWM_setClockPrescaler(EPWM1, EPWM_CLOCK_DIVIDER_64, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBaseCounterMode(EPWM1, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enableADCTrigger(EPWM1, EPWM_SOC_A);

    SYSCON_UNLOCK;
    SYSCON_setEpwmSocAEn(SYSCON, SYSCON_EPWM1SOCAEN, true);
    SYSCON_setTbClkSync(SYSCON, true);
    SYSCON_LOCK;
}

static rt_uint32_t ns800_adc_bg_next_write_frame(void)
{
    uintptr_t base = (uintptr_t)EDMA_getAbsAddrForMultiCore((uint32_t)adc_bg_frames[0]);
    uintptr_t daddr = (uintptr_t)EDMA1->TCD[NS800_ADC_BG_DMA_CHANNEL].DADDR.WORDVAL;
    uintptr_t frame_bytes = NS800_ADC_BACKGROUND_CHANNELS * sizeof(rt_uint16_t);
    uintptr_t ring_bytes = NS800_ADC_BACKGROUND_FRAME_COUNT * frame_bytes;
    uintptr_t offset;

    if ((daddr < base) || (daddr >= (base + ring_bytes)))
    {
        return 0U;
    }

    offset = daddr - base;
    return (rt_uint32_t)((offset / frame_bytes) % NS800_ADC_BACKGROUND_FRAME_COUNT);
}

int ns800_adc_background_start(void)
{
    if (adc_bg_running == RT_TRUE)
    {
        return 0;
    }

    ns800_adc_bg_epwm_stop();
    ns800_adc_bg_dma_stop();
    ns800_adc_bg_gpio_init();
    ns800_adc_bg_adc_init();
    ns800_adc_bg_config_soc();

    rt_memset((void *)adc_bg_frames, 0, sizeof(adc_bg_frames));
    rt_memset((void *)adc_bg_zero_frame, 0, sizeof(adc_bg_zero_frame));
    SCB_CleanDCache_by_Addr(adc_bg_frames, sizeof(adc_bg_frames));

    ns800_adc_bg_dma_config();
    ns800_adc_bg_dma_clear();
    EDMA_startTransfer(EDMA1, NS800_ADC_BG_DMA_CHANNEL);

    adc_bg_running = RT_TRUE;
    ns800_adc_bg_epwm_start();

    return 0;
}

int ns800_adc_background_stop(void)
{
    adc_bg_running = RT_FALSE;
    ns800_adc_bg_epwm_stop();
    ns800_adc_bg_dma_stop();

    return 0;
}

const rt_uint16_t *ns800_adc_background_latest(rt_uint32_t *seq)
{
    rt_uint32_t next;
    rt_uint32_t latest;

    if (adc_bg_running == RT_FALSE)
    {
        if (seq != RT_NULL)
        {
            *seq = 0U;
        }
        return adc_bg_zero_frame;
    }

    next = ns800_adc_bg_next_write_frame();
    latest = (next + NS800_ADC_BACKGROUND_FRAME_COUNT - 1U) % NS800_ADC_BACKGROUND_FRAME_COUNT;

    SCB_InvalidateDCache_by_Addr(adc_bg_frames[latest], NS800_ADC_BG_CACHE_BYTES);
    if (seq != RT_NULL)
    {
        rt_uint32_t citer = EDMA1->TCD[NS800_ADC_BG_DMA_CHANNEL].CITER_ELINKNO.BIT.CITER;
        *seq = (NS800_ADC_BACKGROUND_FRAME_COUNT - citer) % NS800_ADC_BACKGROUND_FRAME_COUNT;
    }

    return adc_bg_frames[latest];
}
