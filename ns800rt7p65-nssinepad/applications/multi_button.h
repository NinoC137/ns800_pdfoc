/*
 * Copyright (c) 2016 Zibin Zheng <znbin@qq.com>
 * All rights reserved
 */

#ifndef MULTI_BUTTON_H
#define MULTI_BUTTON_H

#include <stdint.h>
#include <string.h>

#define TICKS_INTERVAL          5
#define DEBOUNCE_TICKS          3
#define SHORT_TICKS             (300 / TICKS_INTERVAL)
#define LONG_TICKS              (1000 / TICKS_INTERVAL)
#define PRESS_REPEAT_MAX_NUM    15

#if DEBOUNCE_TICKS > 7
#error "DEBOUNCE_TICKS exceeds 3-bit field maximum (7)"
#endif

typedef struct _Button Button;
typedef void (*BtnCallback)(Button* handle, void* user_data);

typedef enum {
    BTN_PRESS_DOWN = 0,
    BTN_PRESS_UP,
    BTN_PRESS_REPEAT,
    BTN_SINGLE_CLICK,
    BTN_DOUBLE_CLICK,
    BTN_LONG_PRESS_START,
    BTN_LONG_PRESS_HOLD,
    BTN_EVENT_COUNT,
    BTN_NONE_PRESS
} ButtonEvent;

typedef enum {
    BTN_STATE_IDLE = 0,
    BTN_STATE_PRESS,
    BTN_STATE_RELEASE,
    BTN_STATE_REPEAT,
    BTN_STATE_LONG_HOLD
} ButtonState;

struct _Button {
    uint16_t ticks;
    uint8_t  repeat : 4;
    uint8_t  event : 4;
    uint8_t  state : 3;
    uint8_t  debounce_cnt : 3;
    uint8_t  active_level : 1;
    uint8_t  button_level : 1;
    uint8_t  button_id;
    uint8_t  (*hal_button_level)(uint8_t button_id);
    BtnCallback cb[BTN_EVENT_COUNT];
    void*    user_data;
    Button* next;
};

#ifdef __cplusplus
extern "C" {
#endif

void button_init(Button* handle, uint8_t(*pin_level)(uint8_t), uint8_t active_level, uint8_t button_id);
void button_attach(Button* handle, ButtonEvent event, BtnCallback cb, void* user_data);
void button_detach(Button* handle, ButtonEvent event);
ButtonEvent button_get_event(Button* handle);
int  button_start(Button* handle);
void button_stop(Button* handle);
void button_ticks(void);
uint8_t button_get_repeat_count(Button* handle);
void button_reset(Button* handle);
int button_is_pressed(Button* handle);

#ifdef __cplusplus
}
#endif

#endif
