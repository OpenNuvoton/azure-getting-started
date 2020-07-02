/**************************************************************************//**
 * @file     esp_log.h
 * @version  V1.00
 * @brief    ESP8266 WiFi driver log header file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __ESP_LOG_H__
#define __ESP_LOG_H__

#include <stdint.h>
#include <stdio.h>
#include "esp_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* printf-like log output function */
#ifndef ESP_CFG_LOG_OUT
#define ESP_CFG_LOG_OUT(FMT, ...)                   \
    do {                                            \
        printf(FMT, ## __VA_ARGS__);                \
    } while (0)
#endif

#if ESP_CFG_LOG
#define ESP_LOG_OUT(CLR, MOD, LVL, ...)             \
    do {                                            \
        ESP_CFG_LOG_OUT(CLR);                       \
        ESP_CFG_LOG_OUT("[%s][%s]", #MOD, #LVL);    \
        ESP_CFG_LOG_OUT(__VA_ARGS__);               \
        ESP_CFG_LOG_OUT(ESP_LOGLVLCLR_NONE);        \
    } while (0)
#else
#define ESP_LOG_OUT(CLR, MOD, LVL, ...)
#endif

/* ESP log level */
#define ESP_LOGLVL_NONE         0x00
#define ESP_LOGLVL_CRIT         0x01    /* Print errors */
#define ESP_LOGLVL_WARN         0x03    /* Print warning messages */
#define ESP_LOGLVL_INFO         0x07    /* Print information messages */
#define ESP_LOGLVL_DBG          0x0F    /* Print debug messages */
#define ESP_LOGLVL_V            0x1F    /* Print verbose messages */

/* ESP log level color */
#define ESP_LOGLVLCLR_NONE      "\033[0m"       /* None */
#define ESP_LOGLVLCLR_CRIT      "\033[1;31m"    /* Red */
#define ESP_LOGLVLCLR_WARN      "\033[1;33m"    /* Yellow */
#define ESP_LOGLVLCLR_INFO      "\033[1;32m"    /* Green */
#define ESP_LOGLVLCLR_DBG       "\033[1;36m"    /* Cyan */
#define ESP_LOGLVLCLR_V         "\033[1;35m"    /* Purple */

#if ESP_CFG_LOGLVL_MAX >= ESP_LOGLVL_CRIT
#define ESP_LOG_CRIT(...)       ESP_LOG_OUT(ESP_LOGLVLCLR_CRIT, ESP, CRIT, ## __VA_ARGS__)
#else
#define ESP_LOG_CRIT(...)
#endif

#if ESP_CFG_LOGLVL_MAX >= ESP_LOGLVL_WARN
#define ESP_LOG_WARN(...)       ESP_LOG_OUT(ESP_LOGLVLCLR_WARN, ESP, WARN, ## __VA_ARGS__)
#else
#define ESP_LOG_WARN(...)
#endif

#if ESP_CFG_LOGLVL_MAX >= ESP_LOGLVL_INFO
#define ESP_LOG_INFO(...)       ESP_LOG_OUT(ESP_LOGLVLCLR_INFO, ESP, INFO, ## __VA_ARGS__)
#else
#define ESP_LOG_INFO(...)
#endif

#if ESP_CFG_LOGLVL_MAX >= ESP_LOGLVL_DBG
#define ESP_LOG_DBG(...)        ESP_LOG_OUT(ESP_LOGLVLCLR_DBG, ESP, DBG, ## __VA_ARGS__)
#else
#define ESP_LOG_DBG(...)
#endif

#if ESP_CFG_LOGLVL_MAX >= ESP_LOGLVL_V
#define ESP_LOG_V(...)          ESP_LOG_OUT(ESP_LOGLVLCLR_V, ESP, V, ## __VA_ARGS__)
#else
#define ESP_LOG_V(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ESP_LOG_H__ */
