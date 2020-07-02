/**************************************************************************//**
 * @file     esp_toolchain.h
 * @version  V1.00
 * @brief    ESP8266 WiFi driver toolchain header file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __ESP_TOOLCHAIN_H__
#define __ESP_TOOLCHAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ESP_WEAK
 *  Mark a function as being weak
 */
#if defined(__ICCARM__)
#define ESP_WEAK __weak
#else
#define ESP_WEAK __attribute__((weak))
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ESP_TOOLCHAIN_H__ */
