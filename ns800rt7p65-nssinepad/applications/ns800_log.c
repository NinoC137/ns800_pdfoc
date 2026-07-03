/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_log.h"

#include <stdarg.h>

#ifdef RT_USING_ULOG
#include <ulog.h>
#endif

#define NS800_LOG_LINE_SIZE  160U
#ifndef LOG_LVL_ERROR
#define LOG_LVL_ERROR        3U
#endif
#ifndef LOG_LVL_WARNING
#define LOG_LVL_WARNING      4U
#endif
#ifndef LOG_LVL_INFO
#define LOG_LVL_INFO         6U
#endif

static void ns800_log_output(rt_uint32_t level, const char *level_name, const char *tag, const char *format, va_list args)
{
    const char *safe_tag = (tag != RT_NULL) ? tag : "app";

#ifdef RT_USING_ULOG
    ulog_voutput(level, safe_tag, RT_TRUE, RT_NULL, 0, 0, 0, format, args);
#else
    char line[NS800_LOG_LINE_SIZE];

    rt_vsnprintf(line, sizeof(line), format, args);
    rt_kprintf("[%s] %s: ", level_name, safe_tag);
    rt_kprintf("%s\r\n", line);
#endif
}

void ns800_log_i(const char *tag, const char *format, ...)
{
    va_list args;

    va_start(args, format);
    ns800_log_output(LOG_LVL_INFO, "I", tag, format, args);
    va_end(args);
}

void ns800_log_w(const char *tag, const char *format, ...)
{
    va_list args;

    va_start(args, format);
    ns800_log_output(LOG_LVL_WARNING, "W", tag, format, args);
    va_end(args);
}

void ns800_log_e(const char *tag, const char *format, ...)
{
    va_list args;

    va_start(args, format);
    ns800_log_output(LOG_LVL_ERROR, "E", tag, format, args);
    va_end(args);
}
