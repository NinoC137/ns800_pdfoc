/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "svm_power_alloc.h"

#include "svm_config.h"

static float svm_abs_f32(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float svm_clamp(float x, float min_value, float max_value)
{
    if (x > max_value) return max_value;
    if (x < min_value) return min_value;
    return x;
}

static uint8_t svm_status_or(uint8_t lhs, svm_status_t rhs)
{
    return (uint8_t)(lhs | (uint8_t)rhs);
}

static uint8_t svm_port_to_kk(svm_port_select_t port)
{
    return (port == SVM_PORT_LOWER) ? 1u : 0u;
}

svm_svm_config_t svm_svm_default_config(void)
{
    svm_svm_config_t cfg;
    cfg.sample_time_s = SVM_CONTROL_PERIOD_S;
    cfg.min_dc_voltage = SVM_MIN_DC_VOLTAGE;
    return cfg;
}

float svm_clamp_unit(float x)
{
    return svm_clamp(x, 0.0f, 1.0f);
}

void svm_power_split(const svm_ab_f32_t *u_ab, float power_factor, svm_split_ab_f32_t *split)
{
    float k;

    if ((u_ab == 0) || (split == 0)) return;
    k = svm_clamp_unit(power_factor);
    split->upper.alpha = (1.0f - k) * u_ab->alpha;
    split->upper.beta = (1.0f - k) * u_ab->beta;
    split->lower.alpha = k * u_ab->alpha;
    split->lower.beta = k * u_ab->beta;
}

uint8_t svm_sector_from_ab(const svm_ab_f32_t *u_ab)
{
    float ua;
    float ub;
    float uc;
    uint8_t ma;
    uint8_t mb;
    uint8_t mc;
    uint8_t sector_tmp;

    if (u_ab == 0) return 0u;

    ua = u_ab->beta;
    ub = SVM_SQRT3_OVER_2 * u_ab->alpha - 0.5f * u_ab->beta;
    uc = -SVM_SQRT3_OVER_2 * u_ab->alpha - 0.5f * u_ab->beta;
    ma = (ua > 0.0f) ? 1u : 0u;
    mb = (ub > 0.0f) ? 1u : 0u;
    mc = (uc > 0.0f) ? 1u : 0u;
    sector_tmp = (uint8_t)(4u * ma + 2u * mb + mc);

    switch (sector_tmp) {
    case 6u: return 1u;
    case 4u: return 2u;
    case 5u: return 3u;
    case 1u: return 4u;
    case 3u: return 5u;
    case 2u: return 6u;
    default: return 1u;
    }
}

svm_status_t svm_compute_port(const svm_ab_f32_t *u_ab,
                              float dc_voltage,
                              svm_port_select_t port,
                              const svm_svm_config_t *cfg,
                              svm_svm_out_t *out)
{
    svm_svm_config_t default_cfg;
    uint8_t status = SVM_STATUS_OK;
    uint8_t sector;
    uint8_t kk;
    float inv_dc;
    float scale;
    float t0;
    float ta;
    float tb;
    float tc;
    float t1 = 0.0f;
    float t2 = 0.0f;
    float t3 = 0.0f;
    float t4 = 0.0f;
    float t5 = 0.0f;
    float t6 = 0.0f;

    if ((u_ab == 0) || (out == 0)) return SVM_STATUS_NULL_POINTER;
    if (cfg == 0) {
        default_cfg = svm_svm_default_config();
        cfg = &default_cfg;
    }

    out->ta = 0.0f;
    out->tb = 0.0f;
    out->tc = 0.0f;
    out->sector = 1u;
    out->status = SVM_STATUS_OK;

    if (svm_abs_f32(dc_voltage) <= cfg->min_dc_voltage) {
        out->status = SVM_STATUS_LOW_DC_VOLTAGE;
        return SVM_STATUS_LOW_DC_VOLTAGE;
    }

    sector = svm_sector_from_ab(u_ab);
    kk = svm_port_to_kk(port);
    inv_dc = 1.0f / dc_voltage;
    scale = SVM_SQRT3 * cfg->sample_time_s * inv_dc;

    switch (sector) {
    case 1u:
        t4 = scale * (SVM_SQRT3_OVER_2 * u_ab->alpha - 0.5f * u_ab->beta);
        t6 = scale * u_ab->beta;
        t0 = (cfg->sample_time_s - t4 - t6) * (float)kk;
        ta = t4 + t6 + t0;
        tb = t6 + t0;
        tc = t0;
        break;
    case 2u:
        t2 = scale * (-SVM_SQRT3_OVER_2 * u_ab->alpha + 0.5f * u_ab->beta);
        t6 = scale * (SVM_SQRT3_OVER_2 * u_ab->alpha + 0.5f * u_ab->beta);
        t0 = (cfg->sample_time_s - t2 - t6) * (float)kk;
        ta = t6 + t0;
        tb = t2 + t6 + t0;
        tc = t0;
        break;
    case 3u:
        t3 = scale * (-SVM_SQRT3_OVER_2 * u_ab->alpha - 0.5f * u_ab->beta);
        t2 = scale * u_ab->beta;
        t0 = (cfg->sample_time_s - t2 - t3) * (float)kk;
        ta = t0;
        tb = t2 + t3 + t0;
        tc = t3 + t0;
        break;
    case 4u:
        t3 = scale * (-SVM_SQRT3_OVER_2 * u_ab->alpha + 0.5f * u_ab->beta);
        t1 = -scale * u_ab->beta;
        t0 = (cfg->sample_time_s - t1 - t3) * (float)kk;
        ta = t0;
        tb = t3 + t0;
        tc = t1 + t3 + t0;
        break;
    case 5u:
        t5 = scale * (SVM_SQRT3_OVER_2 * u_ab->alpha - 0.5f * u_ab->beta);
        t1 = scale * (-SVM_SQRT3_OVER_2 * u_ab->alpha - 0.5f * u_ab->beta);
        t0 = (cfg->sample_time_s - t1 - t5) * (float)kk;
        ta = t5 + t0;
        tb = t0;
        tc = t1 + t5 + t0;
        break;
    case 6u:
    default:
        t4 = scale * (SVM_SQRT3_OVER_2 * u_ab->alpha + 0.5f * u_ab->beta);
        t5 = -scale * u_ab->beta;
        t0 = (cfg->sample_time_s - t4 - t5) * (float)kk;
        ta = t4 + t5 + t0;
        tb = t0;
        tc = t5 + t0;
        break;
    }

    if (t0 < 0.0f) status = svm_status_or(status, SVM_STATUS_OVERMODULATION);
    ta /= cfg->sample_time_s;
    tb /= cfg->sample_time_s;
    tc /= cfg->sample_time_s;
    if ((ta < 0.0f) || (ta > 1.0f) || (tb < 0.0f) || (tb > 1.0f) || (tc < 0.0f) || (tc > 1.0f)) {
        status = svm_status_or(status, SVM_STATUS_CLAMPED);
    }

    out->ta = svm_clamp_unit(ta);
    out->tb = svm_clamp_unit(tb);
    out->tc = svm_clamp_unit(tc);
    out->sector = sector;
    out->status = status;
    return (svm_status_t)status;
}

svm_status_t svm_compute_dual(const svm_ab_f32_t *u_ab,
                              float power_factor,
                              float dc_upper_voltage,
                              float dc_lower_voltage,
                              const svm_svm_config_t *cfg,
                              svm_dual_out_t *out)
{
    svm_split_ab_f32_t split;
    uint8_t status = SVM_STATUS_OK;

    if ((u_ab == 0) || (out == 0)) return SVM_STATUS_NULL_POINTER;
    svm_power_split(u_ab, power_factor, &split);
    status = svm_status_or(status, svm_compute_port(&split.upper, dc_upper_voltage, SVM_PORT_UPPER, cfg, &out->upper));
    status = svm_status_or(status, svm_compute_port(&split.lower, dc_lower_voltage, SVM_PORT_LOWER, cfg, &out->lower));
    return (svm_status_t)status;
}

void svm_pwm_logic_preview(const svm_svm_out_t *upper,
                           const svm_svm_out_t *lower,
                           float carrier,
                           svm_pwm_logic_t *logic)
{
    uint8_t pwm11;
    uint8_t pwm13;
    uint8_t pwm21;
    uint8_t pwm23;
    uint8_t pwm31;
    uint8_t pwm33;

    if ((upper == 0) || (lower == 0) || (logic == 0)) return;

    carrier = svm_clamp_unit(carrier);
    pwm11 = (upper->ta > carrier) ? 1u : 0u;
    pwm13 = (lower->ta > carrier) ? 1u : 0u;
    pwm21 = (upper->tb > carrier) ? 1u : 0u;
    pwm23 = (lower->tb > carrier) ? 1u : 0u;
    pwm31 = (upper->tc > carrier) ? 1u : 0u;
    pwm33 = (lower->tc > carrier) ? 1u : 0u;

    logic->gate[0] = pwm11;
    logic->gate[1] = (uint8_t)!pwm11;
    logic->gate[2] = pwm13;
    logic->gate[3] = (uint8_t)!pwm13;
    logic->gate[4] = pwm21;
    logic->gate[5] = (uint8_t)!pwm21;
    logic->gate[6] = pwm23;
    logic->gate[7] = (uint8_t)!pwm23;
    logic->gate[8] = pwm31;
    logic->gate[9] = (uint8_t)!pwm31;
    logic->gate[10] = pwm33;
    logic->gate[11] = (uint8_t)!pwm33;
}
