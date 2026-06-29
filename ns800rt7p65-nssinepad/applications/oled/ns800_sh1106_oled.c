/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ns800_sh1106_oled.h"

#include "ns800_i2c1_app.h"

#define NS800_SH1106_ADDR       0x3CU
#define NS800_SH1106_CMD        0x00U
#define NS800_SH1106_DATA       0x40U
#define NS800_SH1106_PAGES      8U
#define NS800_SH1106_PAGE_BYTES 128U
#define NS800_SH1106_CHUNK      16U

static rt_uint8_t oled_fb[NS800_SH1106_PAGES][NS800_SH1106_PAGE_BYTES];
static rt_bool_t oled_started = RT_FALSE;

static int oled_write_cmd(rt_uint8_t cmd)
{
    rt_uint8_t tx[2];

    tx[0] = NS800_SH1106_CMD;
    tx[1] = cmd;
    return ns800_i2c1_app_write(NS800_SH1106_ADDR, tx, 2U);
}

static int oled_write_data(const rt_uint8_t *data, rt_uint32_t len)
{
    rt_uint8_t tx[1U + NS800_SH1106_CHUNK];
    rt_uint32_t pos = 0U;

    while (pos < len)
    {
        rt_uint32_t n = len - pos;
        rt_uint32_t i;

        if (n > NS800_SH1106_CHUNK)
        {
            n = NS800_SH1106_CHUNK;
        }

        tx[0] = NS800_SH1106_DATA;
        for (i = 0U; i < n; i++)
        {
            tx[1U + i] = data[pos + i];
        }

        if (ns800_i2c1_app_write(NS800_SH1106_ADDR, tx, 1U + n) != 0)
        {
            return -RT_ERROR;
        }
        pos += n;
    }

    return 0;
}

static void oled_set_page(rt_uint32_t page)
{
    oled_write_cmd((rt_uint8_t)(0xB0U + page));
    oled_write_cmd(0x02U);
    oled_write_cmd(0x10U);
}

static const rt_uint8_t *oled_glyph(char c)
{
    static const rt_uint8_t blank[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const rt_uint8_t dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00};
    static const rt_uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const rt_uint8_t minus[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const rt_uint8_t lparen[5] = {0x00, 0x1C, 0x22, 0x41, 0x00};
    static const rt_uint8_t rparen[5] = {0x00, 0x41, 0x22, 0x1C, 0x00};
    static const rt_uint8_t digits[10][5] =
    {
        {0x3E, 0x51, 0x49, 0x45, 0x3E},
        {0x00, 0x42, 0x7F, 0x40, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46},
        {0x21, 0x41, 0x45, 0x4B, 0x31},
        {0x18, 0x14, 0x12, 0x7F, 0x10},
        {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3C, 0x4A, 0x49, 0x49, 0x30},
        {0x01, 0x71, 0x09, 0x05, 0x03},
        {0x36, 0x49, 0x49, 0x49, 0x36},
        {0x06, 0x49, 0x49, 0x29, 0x1E},
    };
    static const rt_uint8_t A[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
    static const rt_uint8_t C[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
    static const rt_uint8_t D[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
    static const rt_uint8_t F[5] = {0x7F, 0x09, 0x09, 0x09, 0x01};
    static const rt_uint8_t H[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F};
    static const rt_uint8_t I[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
    static const rt_uint8_t L[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
    static const rt_uint8_t N[5] = {0x7F, 0x02, 0x0C, 0x10, 0x7F};
    static const rt_uint8_t O[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
    static const rt_uint8_t P[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
    static const rt_uint8_t S[5] = {0x26, 0x49, 0x49, 0x49, 0x32};
    static const rt_uint8_t T[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
    static const rt_uint8_t V[5] = {0x1F, 0x20, 0x40, 0x20, 0x1F};
    static const rt_uint8_t a[5] = {0x20, 0x54, 0x54, 0x54, 0x78};
    static const rt_uint8_t d[5] = {0x38, 0x44, 0x44, 0x48, 0x7F};
    static const rt_uint8_t e[5] = {0x38, 0x54, 0x54, 0x54, 0x18};
    static const rt_uint8_t i[5] = {0x00, 0x44, 0x7D, 0x40, 0x00};
    static const rt_uint8_t m[5] = {0x7C, 0x04, 0x18, 0x04, 0x78};
    static const rt_uint8_t p[5] = {0x7C, 0x14, 0x14, 0x14, 0x08};
    static const rt_uint8_t s[5] = {0x48, 0x54, 0x54, 0x54, 0x20};
    static const rt_uint8_t x[5] = {0x44, 0x28, 0x10, 0x28, 0x44};

    if ((c >= '0') && (c <= '9'))
    {
        return digits[c - '0'];
    }

    switch (c)
    {
    case ' ': return blank;
    case '.': return dot;
    case ':': return colon;
    case '-': return minus;
    case '(': return lparen;
    case ')': return rparen;
    case 'A': return A;
    case 'C': return C;
    case 'D': return D;
    case 'F': return F;
    case 'H': return H;
    case 'I': return I;
    case 'L': return L;
    case 'N': return N;
    case 'O': return O;
    case 'P': return P;
    case 'S': return S;
    case 'T': return T;
    case 'V': return V;
    case 'a': return a;
    case 'd': return d;
    case 'e': return e;
    case 'i': return i;
    case 'm': return m;
    case 'p': return p;
    case 's': return s;
    case 'x': return x;
    default: return blank;
    }
}

int ns800_sh1106_oled_start(void)
{
    static const rt_uint8_t init_cmds[] =
    {
        0xAE, 0x02, 0x10, 0x40, 0x81, 0xA0, 0xA1, 0xC8,
        0xA6, 0xA8, 0x3F, 0xD3, 0x00, 0xD5, 0x80, 0xD9,
        0xF1, 0xDA, 0x12, 0xDB, 0x40, 0x20, 0x02, 0xA4,
        0xA6, 0xAF,
    };
    rt_uint32_t i;

    if (oled_started)
    {
        return 0;
    }

    if (ns800_i2c1_app_start() != 0)
    {
        return -RT_ERROR;
    }

    rt_thread_mdelay(100);
    for (i = 0U; i < sizeof(init_cmds); i++)
    {
        if (oled_write_cmd(init_cmds[i]) != 0)
        {
            return -RT_ERROR;
        }
    }

    oled_started = RT_TRUE;
    ns800_sh1106_oled_clear();
    ns800_sh1106_oled_flush();
    return 0;
}

void ns800_sh1106_oled_clear(void)
{
    rt_memset(oled_fb, 0, sizeof(oled_fb));
}

void ns800_sh1106_oled_flush(void)
{
    rt_uint32_t page;

    if (!oled_started)
    {
        return;
    }

    for (page = 0U; page < NS800_SH1106_PAGES; page++)
    {
        oled_set_page(page);
        oled_write_data(oled_fb[page], NS800_SH1106_PAGE_BYTES);
    }
}

void ns800_sh1106_oled_draw_pixel(rt_uint32_t x, rt_uint32_t y, rt_bool_t on)
{
    rt_uint8_t mask;

    if ((x >= NS800_SH1106_WIDTH) || (y >= NS800_SH1106_HEIGHT))
    {
        return;
    }

    mask = (rt_uint8_t)(1U << (y & 0x07U));
    if (on)
    {
        oled_fb[y >> 3][x] |= mask;
    }
    else
    {
        oled_fb[y >> 3][x] &= (rt_uint8_t)~mask;
    }
}

void ns800_sh1106_oled_draw_string(rt_uint32_t x, rt_uint32_t y, const char *text)
{
    while ((text != RT_NULL) && (*text != '\0') && ((x + 5U) < NS800_SH1106_WIDTH))
    {
        const rt_uint8_t *glyph = oled_glyph(*text++);
        rt_uint32_t col;

        for (col = 0U; col < 5U; col++)
        {
            rt_uint32_t row;
            for (row = 0U; row < 7U; row++)
            {
                ns800_sh1106_oled_draw_pixel(x + col, y + row, (glyph[col] & (1U << row)) ? RT_TRUE : RT_FALSE);
            }
        }
        x += 6U;
    }
}

void ns800_sh1106_oled_show_string(rt_uint32_t x, rt_uint32_t y, const char *text)
{
    ns800_sh1106_oled_draw_string(x, y, text);
}

static int oled_abs_i32(int value)
{
    return (value < 0) ? -value : value;
}

static void oled_i32_to_dec(char *buf, int value)
{
    char tmp[12];
    int pos = 0;
    int out = 0;
    unsigned int v;

    if (value < 0)
    {
        buf[out++] = '-';
        v = (unsigned int)(-value);
    }
    else
    {
        v = (unsigned int)value;
    }

    do
    {
        tmp[pos++] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v != 0U);

    while (pos > 0)
    {
        buf[out++] = tmp[--pos];
    }
    buf[out] = '\0';
}

void ns800_sh1106_oled_show_float(rt_uint32_t x, rt_uint32_t y, float value)
{
    char buf[16];
    char whole[12];
    int fixed;
    int abs_fixed;
    int pos = 0;
    int i;

    fixed = (int)(value * 100.0f);
    if ((value >= 0.0f) && (((float)fixed / 100.0f) < value))
    {
        fixed++;
    }
    if ((value < 0.0f) && (((float)fixed / 100.0f) > value))
    {
        fixed--;
    }

    abs_fixed = oled_abs_i32(fixed);
    if (fixed < 0)
    {
        buf[pos++] = '-';
    }

    oled_i32_to_dec(whole, abs_fixed / 100);
    for (i = 0; (whole[i] != '\0') && (pos < ((int)sizeof(buf) - 1)); i++)
    {
        buf[pos++] = whole[i];
    }

    if (pos < ((int)sizeof(buf) - 1))
    {
        buf[pos++] = '.';
    }
    if (pos < ((int)sizeof(buf) - 1))
    {
        buf[pos++] = (char)('0' + ((abs_fixed / 10) % 10));
    }
    if (pos < ((int)sizeof(buf) - 1))
    {
        buf[pos++] = (char)('0' + (abs_fixed % 10));
    }
    buf[pos] = '\0';

    ns800_sh1106_oled_draw_string(x, y, buf);
}
