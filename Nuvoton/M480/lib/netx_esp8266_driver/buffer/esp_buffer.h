/**************************************************************************//**
 * @file     esp_buffer.h
 * @version  V1.00
 * @brief    M480 series ESP8266 WiFi driver buffer header file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __ESP_BUFFER_H__
#define __ESP_BUFFER_H__

#ifdef __cplusplus
 extern "C" {
#endif  

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t ridx;          // Read index
    uint16_t widx;          // Write index
    uint16_t size;          // Buffer size
    uint8_t buf[];          // Buffer in flexible array member (FAM, since C99) form.
                            // User must specify the actual size.
} ESP_Buf_t;

void ESP_Buf_Setup(ESP_Buf_t *esp_buf, uint16_t size);
void ESP_Buf_Reset(ESP_Buf_t *esp_buf);
void ESP_Buf_Push(ESP_Buf_t *esp_buf, uint8_t d);
uint8_t ESP_Buf_Pop(ESP_Buf_t *esp_buf);
bool ESP_Buf_IsFull(ESP_Buf_t *esp_buf);
bool ESP_Buf_IsEmpty(ESP_Buf_t *esp_buf);
uint16_t ESP_Buf_Size(ESP_Buf_t *esp_buf);
uint16_t ESP_Buf_MaxSize(ESP_Buf_t *esp_buf);

#ifdef __cplusplus
}
#endif

#endif /*__ESP_BUFFER_H__*/
