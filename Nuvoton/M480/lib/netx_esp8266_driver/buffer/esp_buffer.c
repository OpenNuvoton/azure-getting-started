/**************************************************************************//**
 * @file     esp_buffer.c
 * @version  V1.00
 * @brief    M480 series ESP8266 WiFi driver buffer source file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "stdlib.h"

/* ESP includes */
#include "esp_cfg.h"
#include "buffer/esp_buffer.h"

void ESP_Buf_Setup(ESP_Buf_t *buf, uint16_t size)
{
    buf->ridx = buf->widx = 0;
    buf->size = size;
}

void ESP_Buf_Reset(ESP_Buf_t *buf)
{
    buf->ridx = buf->widx = 0;
}

void ESP_Buf_Push(ESP_Buf_t *buf, uint8_t d)
{
    buf->buf[buf->widx++] = d;
    if (buf->widx == buf->size) {
        buf->widx = 0;
    }
}

uint8_t ESP_Buf_Pop(ESP_Buf_t *buf)
{
    uint8_t d = buf->buf[(buf->ridx++)];

    if (buf->ridx == buf->size) {
        buf->ridx = 0;
    }

    return d;
}

bool ESP_Buf_IsFull(ESP_Buf_t *buf)
{
    if ((buf->widx == (buf->size - 1)) && (buf->ridx == 0)) {
        return true;
    }

    if (buf->ridx == (buf->widx + 1)) {
        return true;
    } else {
        return false;
    }
}

bool ESP_Buf_IsEmpty(ESP_Buf_t *buf)
{
    return (buf->ridx == buf->widx);
}

uint16_t ESP_Buf_Size(ESP_Buf_t *buf)
{
    if (buf->widx >= buf->ridx) {
        return (buf->widx - buf->ridx);
    } else {
        return (buf->size + buf->widx - buf->ridx);
    }
}

uint16_t ESP_Buf_MaxSize(ESP_Buf_t *buf)
{
    return (buf->size - 1);
}
