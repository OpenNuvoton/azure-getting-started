/**************************************************************************//**
 * @file     esp8266_ll.h
 * @version  V1.00
 * @brief    M480 series ESP8266 WiFi low-level driver header file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __ESP_LL_H__
#define __ESP_LL_H__

#ifdef __cplusplus
 extern "C" {
#endif  

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief   Initialize ESP low-level driver
 */
bool ESP_LL_Init(void);

/**
 * @brief   Reset ESP modem via hardware power/reset pin
 */
void ESP_LL_ResetModem(uint32_t assert_delay, uint32_t deassert_delay);

/**
 * @brief   Configure baudrate
 */
void ESP_LL_ConfigBaudrate(uint32_t baudrate);

/**
 * @brief   Enable CTS/RTS flow control
 */
void ESP_LL_EnableFlowControl(bool enable);

/**
 * @brief   Purge Rx stream
 */
void ESP_LL_RxStrm_Purge(void);

/**
 * @brief   Check if Rx stream is readable
 */

bool ESP_LL_RxStrm_Readable(void);

/**
 * @brief   Read one character from Rx stream
 * @note    Call ESP_LL_RxStrm_Readable() first to avoid Rx stream underrun
 */
uint8_t ESP_LL_RxStrm_GetChar(void);

/**
 * @brief   Check if Tx stream is writable
 */

bool ESP_LL_TxStrm_Writable(void);

/**
 * @brief   Write one character to Tx stream
 * @note    Call ESP_LL_TxStrm_Writable() first to avoid Tx stream overrun
 */
void ESP_LL_TxStrm_PutChar(uint8_t c);

#ifdef __cplusplus
}
#endif

#endif /*__ESP_LL_H__*/
