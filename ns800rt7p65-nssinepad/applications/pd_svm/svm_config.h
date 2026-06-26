/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SVM_CONFIG_H
#define SVM_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define SVM_CONTROL_PERIOD_S           (1.0e-4f)
#define SVM_TWO_PI                     (6.2831853071795864769f)
#define SVM_PI                         (3.1415926535897932385f)
#define SVM_HALF_PI                    (1.5707963267948966192f)
#define SVM_SQRT3                      (1.7320508075688772935f)
#define SVM_INV_SQRT3                  (0.5773502691896257645f)
#define SVM_SQRT3_OVER_2               (0.8660254037844386468f)

#define SVM_DEFAULT_OUTER_VOLTAGE_KP   (0.2f)
#define SVM_DEFAULT_OUTER_VOLTAGE_KI   (210.0f)
#define SVM_DEFAULT_INNER_CURRENT_KP   (5.0f)
#define SVM_DEFAULT_INNER_CURRENT_KI   (220.0f)
#define SVM_DEFAULT_VOLTAGE_CROSS_GAIN (50.0f * SVM_PI * 15.0e-6f)
#define SVM_DEFAULT_CURRENT_CROSS_GAIN (50.0f * SVM_PI * 1.0e-2f)

#define SVM_DEFAULT_UD_REF             (12.0f)
#define SVM_DEFAULT_UQ_REF             (0.0f)

#define SVM_MIN_DC_VOLTAGE             (0.0f)
#define SVM_DEFAULT_FREQ_HZ            (50.0f)
#define SVM_LUT_QUARTER_SIZE           (64u)

#ifndef SVM_USE_NS800_MMATH
#define SVM_USE_NS800_MMATH            (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* SVM_CONFIG_H */
