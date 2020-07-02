/**************************************************************************//**
 * @file     esp_assert.h
 * @version  V1.00
 * @brief    ESP8266 WiFi driver assert header file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __ESP_ASSERT_H__
#define __ESP_ASSERT_H__

#include "esp_cfg.h"
#include "platform/esp_toolchain.h"
#include "platform/esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_CONCAT(a, b)    ESP_CONCAT_(a, b)
#define ESP_CONCAT_(a, b)   a##b

/** ESP_ASSERT
 *  Declare runtime assertions, results in runtime error if condition is false
 *
 *  @note
 *  Use of ESP_ASSERT is limited to develop/debug builds.
 */
#if ESP_CFG_ASSERT
#define ESP_ASSERT(expr)                                                \
    do {                                                                \
        if (!(expr)) {                                                  \
            ESP_CFG_LOG_OUT(ESP_LOGLVLCLR_CRIT);                        \
            ESP_CFG_LOG_OUT("ESP ASSERT FAILURE: ");                    \
            ESP_CFG_LOG_OUT(#expr);                                     \
            ESP_CFG_LOG_OUT("\r\nFile: %s+%d\r\n", __FILE__, __LINE__); \
            ESP_CFG_LOG_OUT(ESP_LOGLVLCLR_NONE);                        \
            while (1);                                                  \
        }                                                               \
    } while (0)
#else
#define ESP_ASSERT(expr)    ((void)0)
#endif

/** ESP_STATIC_ASSERT
 *  Declare compile-time assertions, results in compile-time error if condition is false
 */
#if defined(__cplusplus) && (__cplusplus >= 201103L || __cpp_static_assert >= 200410L)
#define ESP_STATIC_ASSERT(expr, msg) static_assert(expr, msg)
#elif !defined(__cplusplus) && __STDC_VERSION__ >= 201112L
#define ESP_STATIC_ASSERT(expr, msg) _Static_assert(expr, msg)
#elif defined(__ICCARM__)
#define ESP_STATIC_ASSERT(expr, msg) static_assert(expr, msg)
#else
#define ESP_STATIC_ASSERT(expr, msg) \
    enum {ESP_CONCAT(ESP_ASSERTION_AT_, __LINE__) = sizeof(char[(expr) ? 1 : -1])}
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ESP_ASSERT_H__ */
