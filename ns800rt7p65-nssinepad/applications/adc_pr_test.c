/*
 * PR test commands for the slim NS800 ADC driver.
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include "drv_adc.h"

#ifdef RT_USING_FINSH
#include "finsh.h"
#endif

#ifdef BSP_USING_ADC

#define ADC_PR_DEFAULT_CHANNEL  0U
#define ADC_PR_TIMEOUT_MS       1000U

static struct rt_semaphore adc_pr_irq_sem;
static rt_bool_t adc_pr_irq_sem_inited = RT_FALSE;
static volatile rt_uint32_t adc_pr_irq_count;

static rt_uint32_t adc_pr_parse_u32(const char *text, rt_uint32_t fallback)
{
    char *end = RT_NULL;
    unsigned long value;

    if (text == RT_NULL)
    {
        return fallback;
    }

    value = strtoul(text, &end, 0);
    if (end == text)
    {
        return fallback;
    }

    return (rt_uint32_t)value;
}

static rt_err_t adc_pr_irq_sem_init(void)
{
    if (adc_pr_irq_sem_inited == RT_TRUE)
    {
        return RT_EOK;
    }

    if (rt_sem_init(&adc_pr_irq_sem, "adcirq", 0, RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        return -RT_ERROR;
    }

    adc_pr_irq_sem_inited = RT_TRUE;
    return RT_EOK;
}

static void adc_pr_irq_callback(void *user_data)
{
    RT_UNUSED(user_data);
    rt_kprintf("adc irq callback test.\r\n");
    adc_pr_irq_count++;

    if (adc_pr_irq_sem_inited == RT_TRUE)
    {
        rt_sem_release(&adc_pr_irq_sem);
    }
}

static rt_device_t adc_pr_find_device(void)
{
    rt_device_t device;

    device = rt_device_find("adc0");
    if (device == RT_NULL)
    {
        rt_kprintf("adc0 not found\r\n");
    }

    return device;
}

static int adc_pr_read(int argc, char **argv)
{
    rt_device_t device;
    rt_uint32_t channel = (argc > 1) ? adc_pr_parse_u32(argv[1], ADC_PR_DEFAULT_CHANNEL) : ADC_PR_DEFAULT_CHANNEL;
    rt_uint32_t value;

    if (channel >= NS800_ADC_CHANNEL_MAX)
    {
        rt_kprintf("usage: adc_pr_read [channel]\r\n");
        return -RT_EINVAL;
    }

    device = adc_pr_find_device();
    if (device == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_adc_enable((rt_adc_device_t)device, channel);
    value = rt_adc_read((rt_adc_device_t)device, channel);
    rt_kprintf("adc_pr_read: channel=%u value=%u\r\n",
               (unsigned int)channel,
               (unsigned int)value);

    return 0;
}

static int adc_pr_irq(int argc, char **argv)
{
    rt_device_t device;
    rt_uint32_t channel = (argc > 1) ? adc_pr_parse_u32(argv[1], ADC_PR_DEFAULT_CHANNEL) : ADC_PR_DEFAULT_CHANNEL;
    rt_uint32_t value;
    rt_err_t result;
    struct ns800_adc_callback callback =
    {
        .callback = adc_pr_irq_callback,
        .user_data = (void *)(rt_ubase_t)channel,
    };

    if (channel >= NS800_ADC_CHANNEL_MAX)
    {
        rt_kprintf("usage: adc_pr_irq [channel]\r\n");
        return -RT_EINVAL;
    }

    if (adc_pr_irq_sem_init() != RT_EOK)
    {
        rt_kprintf("adc_pr_irq: semaphore init failed\r\n");
        return -RT_ERROR;
    }

    device = adc_pr_find_device();
    if (device == RT_NULL)
    {
        return -RT_ERROR;
    }

    while (rt_sem_take(&adc_pr_irq_sem, 0) == RT_EOK)
    {
    }

    adc_pr_irq_count = 0U;
    rt_adc_enable((rt_adc_device_t)device, channel);
    result = rt_device_control(device, NS800_ADC_CMD_ENABLE_IRQ, &callback);
    if (result != RT_EOK)
    {
        rt_kprintf("adc_pr_irq: enable irq failed %d\r\n", result);
        return result;
    }

    value = rt_adc_read((rt_adc_device_t)device, channel);
    result = rt_sem_take(&adc_pr_irq_sem, rt_tick_from_millisecond(ADC_PR_TIMEOUT_MS));
    rt_device_control(device, NS800_ADC_CMD_DISABLE_EXT, RT_NULL);

    if (result != RT_EOK)
    {
        rt_kprintf("adc_pr_irq: timeout value=%u\r\n", (unsigned int)value);
        return -RT_ETIMEOUT;
    }

    rt_kprintf("adc_pr_irq: channel=%u value=%u count=%u\r\n",
               (unsigned int)channel,
               (unsigned int)value,
               (unsigned int)adc_pr_irq_count);
    return 0;
}

static int adc_pr_ppb(int argc, char **argv)
{
    rt_device_t device;
    rt_uint32_t channel = (argc > 1) ? adc_pr_parse_u32(argv[1], ADC_PR_DEFAULT_CHANNEL) : ADC_PR_DEFAULT_CHANNEL;
    rt_uint32_t value;
    rt_err_t result;

    if (channel >= NS800_ADC_CHANNEL_MAX)
    {
        rt_kprintf("usage: adc_pr_ppb [channel]\r\n");
        return -RT_EINVAL;
    }

    device = adc_pr_find_device();
    if (device == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_adc_enable((rt_adc_device_t)device, channel);
    result = rt_device_control(device, NS800_ADC_CMD_ENABLE_PPB, RT_NULL);
    if (result != RT_EOK)
    {
        rt_kprintf("adc_pr_ppb: enable ppb failed %d\r\n", result);
        return result;
    }

    value = rt_adc_read((rt_adc_device_t)device, channel);
    rt_device_control(device, NS800_ADC_CMD_DISABLE_EXT, RT_NULL);
    rt_kprintf("adc_pr_ppb: channel=%u value=%u\r\n",
               (unsigned int)channel,
               (unsigned int)value);

    return 0;
}

#ifdef RT_USING_FINSH
MSH_CMD_EXPORT(adc_pr_read, test slim ADC read);
MSH_CMD_EXPORT(adc_pr_irq, test slim ADC IRQ callback);
MSH_CMD_EXPORT(adc_pr_ppb, test slim ADC PPB oversampling);
#endif

#endif
