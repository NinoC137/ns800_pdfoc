/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    sl_observer_app.c
 * @brief   无感观测器应用层实现:静态实例、ISR 单拍更新与 FinSH 在线整定。
 *
 * 持有观测器的静态 config/state/output 三件套。@ref sl_observer_app_update 供
 * 电机控制 ISR 调用(Clarke + 单拍观测),查询接口与 FinSH 命令只读取已算好的
 * 结果。FinSH 在线整定直接改写静态配置:每个参数为独立 32 位 float,写入在
 * Cortex-M 上为单指令原子存储,与 ISR 的读取天然无撕裂,故不额外加临界区
 * (与工程既有 volatile 模式切换风格一致)。
 *
 * @note rt_kprintf 不支持 %f,故所有浮点量按“毫单位整数”打印/整定(值 x1000)。
 */

#include "sl_observer_app.h"

#include "svm_transform.h"   /* svm_clarke */

/** @brief 观测器配置静态实例(可经 FinSH 在线整定)。 */
static sl_observer_config_t s_obs_cfg;
/** @brief 观测器运行状态静态实例。 */
static sl_observer_state_t s_obs_state;
/** @brief 观测器最近一拍输出静态实例。 */
static sl_observer_output_t s_obs_output;

/**
 * @brief 清零观测器输出静态实例。
 */
static void sl_observer_app_clear_output(void)
{
    s_obs_output.theta_e_rad = 0.0f;
    s_obs_output.omega_e_rad_s = 0.0f;
    s_obs_output.omega_m_rad_s = 0.0f;
    s_obs_output.speed_rpm = 0.0f;
    s_obs_output.emf_alpha = 0.0f;
    s_obs_output.emf_beta = 0.0f;
    s_obs_output.emf_mag = 0.0f;
    s_obs_output.i_hat_alpha = 0.0f;
    s_obs_output.i_hat_beta = 0.0f;
    s_obs_output.valid = false;
}

int sl_observer_app_start(void)
{
    sl_observer_default_config(&s_obs_cfg);
    sl_observer_init(&s_obs_state, &s_obs_cfg);
    sl_observer_app_clear_output();
    return 0;
}

void sl_observer_app_update(const svm_ab_f32_t *v_ab, const svm_abc_f32_t *i_abc)
{
    svm_ab_f32_t i_ab;

    if ((v_ab == 0) || (i_abc == 0))
    {
        return;
    }

    /* 三相电流 -> alpha/beta,随后执行一拍无感观测(全程无阻塞、可进 ISR)。 */
    svm_clarke(i_abc, &i_ab);
    sl_observer_step(&s_obs_state, &s_obs_cfg, v_ab, &i_ab, &s_obs_output);
}

const sl_observer_output_t *sl_observer_app_get_output(void)
{
    return &s_obs_output;
}

float sl_observer_app_get_theta_e(void)
{
    return s_obs_output.theta_e_rad;
}

float sl_observer_app_get_speed_rpm(void)
{
    return s_obs_output.speed_rpm;
}

#ifdef RT_USING_FINSH
#include <rtthread.h>
#include "finsh.h"

/**
 * @brief 解析可选带符号的十进制整数(自包含,不依赖 libc)。
 *
 * @param text 待解析字符串;为空按 0 处理。
 * @return 解析得到的整数;非数字字符处停止解析。
 */
static int sl_obs_parse_int(const char *text)
{
    int sign = 1;
    int value = 0;

    if (text == 0)
    {
        return 0;
    }
    if (*text == '-')
    {
        sign = -1;
        text++;
    }
    else if (*text == '+')
    {
        text++;
    }

    while ((*text >= '0') && (*text <= '9'))
    {
        value = (value * 10) + (int)(*text - '0');
        text++;
    }

    return sign * value;
}

/**
 * @brief FinSH 命令:打印无感观测器状态与当前整定参数。
 *
 * 所有浮点量以毫单位整数打印(值 x1000):角度 mrad、角速度 mrad/s、
 * 反电动势 mV;转速 rpm 直接取整。
 */
static int sl_obs_status_cmd(int argc, char **argv)
{
    const sl_observer_output_t *o = &s_obs_output;

    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("sl_obs: valid=%d theta_e=%d mrad omega_e=%d mrad/s rpm=%d emf_mag=%d mV\r\n",
               (int)o->valid,
               (int)(o->theta_e_rad * 1000.0f),
               (int)(o->omega_e_rad_s * 1000.0f),
               (int)o->speed_rpm,
               (int)(o->emf_mag * 1000.0f));
    rt_kprintf("emf: ea=%d mV eb=%d mV  i_hat: ia=%d mA ib=%d mA\r\n",
               (int)(o->emf_alpha * 1000.0f),
               (int)(o->emf_beta * 1000.0f),
               (int)(o->i_hat_alpha * 1000.0f),
               (int)(o->i_hat_beta * 1000.0f));
    rt_kprintf("cfg(milli): rs=%d ls=%d flux=%d kslide=%d delta=%d wc=%d pll_kp=%d pll_ki=%d\r\n",
               (int)(s_obs_cfg.rs_ohm * 1000.0f),
               (int)(s_obs_cfg.ls_h * 1000.0f),
               (int)(s_obs_cfg.flux_wb * 1000.0f),
               (int)(s_obs_cfg.k_slide * 1000.0f),
               (int)(s_obs_cfg.boundary_delta * 1000.0f),
               (int)(s_obs_cfg.emf_lpf_wc * 1000.0f),
               (int)(s_obs_cfg.pll_pi.kp * 1000.0f),
               (int)(s_obs_cfg.pll_pi.ki * 1000.0f));
    return 0;
}

/**
 * @brief FinSH 命令:在线整定观测器参数。
 *
 * 用法:@c sl_obs_set @c <name> @c <milli>,其中 milli 为参数真实值 x1000 的整数。
 * 支持 name:rs、ls、flux、kslide、delta、wc、pll_kp、pll_ki。
 */
static int sl_obs_set_cmd(int argc, char **argv)
{
    int milli;
    float value;

    if (argc < 3)
    {
        rt_kprintf("usage: sl_obs_set <rs|ls|flux|kslide|delta|wc|pll_kp|pll_ki> <milli>\r\n");
        return -1;
    }

    milli = sl_obs_parse_int(argv[2]);
    value = (float)milli / 1000.0f;

    if (rt_strcmp(argv[1], "rs") == 0)
    {
        s_obs_cfg.rs_ohm = value;
    }
    else if (rt_strcmp(argv[1], "ls") == 0)
    {
        s_obs_cfg.ls_h = value;
    }
    else if (rt_strcmp(argv[1], "flux") == 0)
    {
        s_obs_cfg.flux_wb = value;
    }
    else if (rt_strcmp(argv[1], "kslide") == 0)
    {
        s_obs_cfg.k_slide = value;
    }
    else if (rt_strcmp(argv[1], "delta") == 0)
    {
        s_obs_cfg.boundary_delta = value;
    }
    else if (rt_strcmp(argv[1], "wc") == 0)
    {
        s_obs_cfg.emf_lpf_wc = value;
    }
    else if (rt_strcmp(argv[1], "pll_kp") == 0)
    {
        s_obs_cfg.pll_pi.kp = value;
    }
    else if (rt_strcmp(argv[1], "pll_ki") == 0)
    {
        s_obs_cfg.pll_pi.ki = value;
    }
    else
    {
        rt_kprintf("unknown param '%s'\r\n", argv[1]);
        return -1;
    }

    rt_kprintf("sl_obs_set %s = %d milli\r\n", argv[1], milli);
    return 0;
}

MSH_CMD_EXPORT_ALIAS(sl_obs_status_cmd, sl_obs_status, show sensorless observer status);
MSH_CMD_EXPORT_ALIAS(sl_obs_set_cmd, sl_obs_set, tune observer param: sl_obs_set name milli);
#endif /* RT_USING_FINSH */
