/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_i2c1_app.h"

#include "gpio.h"
#include "i2c.h"
#include "rcc.h"

#define NS800_I2C1_BAUDRATE_HZ  400000UL

static rt_bool_t i2c1_started = RT_FALSE;
static rt_mutex_t i2c1_lock = RT_NULL;

static void ns800_i2c1_gpio_init(void)
{
    GPIO_setPinConfig(GPIO_91_I2C1_SDA);
    GPIO_setAnalogMode(GPIO_91, GPIO_ANALOG_DISABLED);
    GPIO_setPadConfig(GPIO_91, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(GPIO_91, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(GPIO_91, GPIO_DIR_MODE_IN);

    GPIO_setPinConfig(GPIO_92_I2C1_SCL);
    GPIO_setAnalogMode(GPIO_92, GPIO_ANALOG_DISABLED);
    GPIO_setPadConfig(GPIO_92, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(GPIO_92, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(GPIO_92, GPIO_DIR_MODE_IN);
}

int ns800_i2c1_app_start(void)
{
    if (i2c1_started)
    {
        return 0;
    }

    if (i2c1_lock == RT_NULL)
    {
        i2c1_lock = rt_mutex_create("i2c1", RT_IPC_FLAG_PRIO);
        if (i2c1_lock == RT_NULL)
        {
            return -RT_ERROR;
        }
    }

    ns800_i2c1_gpio_init();

    I2C_resetMaster(I2C1);
    I2C_disableMasterDebug(I2C1);
    I2C_setMasterWatermarks(I2C1, I2C_MASTER_WATERMARK_0, I2C_MASTER_WATERMARK_0);
    I2C_setMasterGlitchFilter(I2C1, I2C_MASTER_FILTER_PCLK1, I2C_MASTER_FILTER_PCLK1);
    I2C_configMasterBaudRate(I2C1, RCC_getPclk2Frequency(), NS800_I2C1_BAUDRATE_HZ);
    I2C_setMasterBusIdleTimeout(I2C1, 1U);
    I2C_setMasterPinLowTimeout(I2C1, I2C_MASTER_PINLOW_SCLSDA, 0U);
    I2C_enableMasterModule(I2C1);

    i2c1_started = RT_TRUE;
    return 0;
}

int ns800_i2c1_app_write(rt_uint8_t addr7, const rt_uint8_t *data, rt_uint32_t len)
{
    I2C_Status status;

    if ((!i2c1_started) || (data == RT_NULL) || (len == 0U))
    {
        return -RT_ERROR;
    }

    rt_mutex_take(i2c1_lock, RT_WAITING_FOREVER);

    status = I2C_sendStart(I2C1, (rt_uint8_t)(addr7 << 1), I2C_DIRECTION_WRITE);
    if (status == I2C_STATUS_SUCCESS)
    {
        status = I2C_sendDataMaster(I2C1, (void *)data, len);
    }
    if (status == I2C_STATUS_SUCCESS)
    {
        status = I2C_sendStop(I2C1);
    }
    else
    {
        I2C_sendStop(I2C1);
    }

    rt_mutex_release(i2c1_lock);

    return (status == I2C_STATUS_SUCCESS) ? 0 : -RT_ERROR;
}
