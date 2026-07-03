/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NS800_LOG_H__
#define __NS800_LOG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

void ns800_log_i(const char *tag, const char *format, ...);
void ns800_log_w(const char *tag, const char *format, ...);
void ns800_log_e(const char *tag, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* __NS800_LOG_H__ */
