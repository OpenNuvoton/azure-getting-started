/**************************************************************************//**
 * @file     esp_platform.h
 * @version  V1.00
 * @brief    M480 series ESP8266 WiFi platform driver header file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __ESP_PLATFORM_H__
#define __ESP_PLATFORM_H__

#ifdef __cplusplus
 extern "C" {
#endif  

#include <stdint.h>
#include <stdbool.h>

uint64_t ESP_Plat_GetTime_MS(void);
void ESP_Plat_Wait_MS(uint32_t ms);
void ESP_Plat_EnterCriticalSection(void);
void ESP_Plat_ExitCriticalSection(void);

#ifdef __cplusplus
}
#endif

#endif /*__ESP_PLATFORM_H__*/
