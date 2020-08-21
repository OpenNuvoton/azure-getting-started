/**************************************************************************//**
 * @file     esp8266_wifi.h
 * @version  V1.00
 * @brief    M480 series ESP8266 WiFi driver header file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __ESP8266_WIFI_H__
#define __ESP8266_WIFI_H__

#ifdef __cplusplus
 extern "C" {
#endif  

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "esp_cfg.h"
#include "platform/esp_assert.h"
#include "platform/esp_log.h"
#include "platform/esp_platform.h"
#include "platform/esp_toolchain.h"

/* Enable anonymous structures and unions */
#if defined(__CC_ARM)
#pragma anon_unions
#endif

/**
 * @brief Wi-Fi Security types.
 *
 * @ingroup WiFi_datatypes_enums
 */
typedef enum
{
    WiFiSecurityOpen = 0,    /**< Open - No Security. */
    WiFiSecurityWEP,         /**< WEP Security. */
    WiFiSecurityWPA,         /**< WPA Security. */
    WiFiSecurityWPA2,        /**< WPA2 Security. */
    WiFiSecurityWPA2_ent,    /**< WPA2 Enterprise Security. */
    WiFiSecurityNotSupported /**< Unknown Security. */
} ESP_WIFI_Security_t;

/**
 * @brief Wi-Fi scan results.
 *
 * Structure to store the Wi-Fi scan results.
 *
 * @note The size of char arrays are the MAX lengths + 1 to
 * account for possible null terminating at the end of the
 * strings.
 *
 * @see WIFI_Scan
 *
 * @ingroup WiFi_datatypes_returnstructs
 */
typedef struct
{
    char cSSID[ ESP_CFG_SSID_SIZE + 1 ];        /**< SSID of the Wi-Fi network with a NULL termination. */
    uint8_t ucBSSID[ 6 ];                       /**< BSSID of the Wi-Fi network. */
    ESP_WIFI_Security_t xSecurity;              /**< Wi-Fi Security. @see ESP_WIFI_Security_t. */
    int8_t cRSSI;                               /**< Signal Strength. */
    int8_t cChannel;                            /**< Channel number. */
    uint8_t ucHidden;                           /**< Hidden channel. */
} ESP_WIFI_ScanResult_t;

typedef enum {
    /* Open status code */
    ESP_WIFI_STATUS_OK          = 0,
    ESP_WIFI_STATUS_ERROR       = 1,
    ESP_WIFI_STATUS_TIMEOUT     = 2,
    ESP_WIFI_STATUS_LINKCLOSED  = 3,
    ESP_WIFI_STATUS_BUSY        = 4,

    /* Internal status code */
    ESP_WIFI_STATUS_SEND        = 5,
    ESP_WIFI_STATUS_OOB         = 6,
} ESP_WIFI_Status_t;

typedef enum {
    ESP_WIFI_TCP    = 0,
    ESP_WIFI_UDP    = 1,
    ESP_WIFI_SSL    = 2
}ESP_WIFI_ConnType_t;

typedef enum {
    ESP_WIFI_STATION            = 1,
    ESP_WIFI_SOFTAP             = 2,
    ESP_WIFI_SOFTAP_STATION     = 3
} ESP_WIFI_Mode_t;

typedef void (*ESP_WIFI_CloseCallback_t)(void *Ctx, uint8_t LinkID);
typedef void (*ESP_WIFI_DataRecvCallback_t)(void *Ctx, uint8_t LinkID, uint8_t *Data, uint16_t Size);

/** ESP8266 firmware AT version
  *
  * @param major Major version number
  * @param minor Minor version number
  * @param patch Patch version number
  */
typedef struct {
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
} ESP_WIFI_FW_AT_Version_t;

/** ESP8266 firmware SDK version
  *
  * @param major Major version number
  * @param minor Minor version number
  * @param patch Patch version number
  */
typedef struct {
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
} ESP_WIFI_FW_SDK_Version_t;

/* Exported functions */

/**
 * @brief  Initialize WIFI module
 * @param  Obj: pointer to WiF module
 * @retval Operation status
 */
ESP_WIFI_Status_t ESP_WIFI_Init( void );

/**
 * @brief  Install close callback on link close
 */
ESP_WIFI_Status_t ESP_WIFI_InstallCloseCallback( ESP_WIFI_CloseCallback_t Callback, void *Ctx );

/**
 * @brief  Install data receive callback on reactive receive
 */
ESP_WIFI_Status_t ESP_WIFI_InstallDataRecvCallback( ESP_WIFI_DataRecvCallback_t Callback, void *Ctx );

/**
 * @brief  Get firmware version
 */
ESP_WIFI_Status_t ESP_WIFI_GetFirmwareVersion( ESP_WIFI_FW_AT_Version_t *at_version, ESP_WIFI_FW_SDK_Version_t *sdk_version );

/**
 * @brief  Set WiFi mode
 */
ESP_WIFI_Status_t ESP_WIFI_SetWiFiMode( ESP_WIFI_Mode_t mode );

/**
 * @brief  Enable DHCP
 */
ESP_WIFI_Status_t ESP_WIFI_EnableDHCP( ESP_WIFI_Mode_t mode, bool enable );

/**
 * @brief  Connects to an AP
 * @retval Operation status
 */
ESP_WIFI_Status_t ESP_WIFI_Connect( const char * SSID, const char * Password );

/**
 * @brief  Disconnects from the AP
 * @retval Operation status
 */
ESP_WIFI_Status_t ESP_WIFI_Disconnect( void );

/**
 * @brief  Restarts the module
 * @retval Operation status
 */
ESP_WIFI_Status_t ESP_WIFI_Reset( void );

/**
 * @brief  Lists Available APs
 * @retval Operation status
 */
ESP_WIFI_Status_t ESP_WIFI_Scan( ESP_WIFI_ScanResult_t * pxBuffer, uint8_t ucNumNetworks );

/**
 * @brief  Get network status
 * @retval Operation status
 */
ESP_WIFI_Status_t ESP_WIFI_GetNetStatus( uint8_t ApIpAddr[], uint8_t ApMacAddr[],
                                         uint8_t StaIpAddr[], uint8_t StaMacAddr[] );

/**
 * @brief  Get the connection status
 * @retval Operation status
 */
ESP_WIFI_Status_t ESP_WIFI_GetConnStatus( uint8_t LinkID );

/**
 * @brief  Ping an IP address
 * @retval Operation status
 */
ESP_WIFI_Status_t ESP_WIFI_Ping( uint8_t * pucIPAddr );

/**
 * @brief  Get the IP address from a host name
 * @retval Operation status
 */
ESP_WIFI_Status_t ESP_WIFI_GetHostIP( const char * pcHost, uint8_t * pucIPAddr );

/**
 * @brief  Establishes TCP Connection, UDP Transmission or SSL Connection
 * @retval Operation status
 */
ESP_WIFI_Status_t ESP_WIFI_OpenClientConnection( uint8_t LinkID, ESP_WIFI_ConnType_t Type, uint8_t *RemoteIP,
                                                 uint16_t RemotePort, uint32_t Option);
ESP_WIFI_Status_t ESP_WIFI_OpenClientConnection2( uint8_t LinkID, ESP_WIFI_ConnType_t Type, uint8_t *RemoteIP,
                                                 uint16_t RemotePort, uint32_t Option, uint32_t ulTimeout );

/**
 * @brief  Closes the TCP/UDP/SSL Connection
 * @retval Operation status
 */
 
ESP_WIFI_Status_t ESP_WIFI_CloseClientConnection( uint8_t LinkID);
ESP_WIFI_Status_t ESP_WIFI_CloseClientConnection2( uint8_t LinkID, uint32_t ulTimeout );

/**
 * @brief  Sends Data
 * @retval Operation status
 */
ESP_WIFI_Status_t ESP_WIFI_Send( uint8_t LinkID, uint8_t *pcBuf, 
                                 uint16_t usReqLen, uint16_t * usSendLen );
ESP_WIFI_Status_t ESP_WIFI_Send2( uint8_t LinkID, uint8_t *pcBuf, 
                                  uint16_t usReqLen, uint16_t * usSendLen,
                                  uint32_t ulTimeout );
ESP_WIFI_Status_t ESP_WIFI_SendTo( uint8_t LinkID, uint8_t *pcBuf, 
                                   uint16_t usReqLen, uint16_t * usSendLen,
                                   uint8_t *RemoteIPAlt, uint16_t RemotePortAlt);
ESP_WIFI_Status_t ESP_WIFI_SendTo2( uint8_t LinkID, uint8_t *pcBuf, 
                                    uint16_t usReqLen, uint16_t * usSendLen,
                                    uint8_t *RemoteIPAlt, uint16_t RemotePortAlt,
                                    uint32_t ulTimeout);
/**
 *  @brief  Receive Data
 *
 *  @details
 *  Proactive receive:
 *  When receive buffer is passed to this API, data received will first place
 *  into this buffer if socket ID is matched, then go reactive receive.
 *
 *  If ESP TCP passive receive mode is enabled, middleware can receive TCP socket
 *  data only through proactive receive.
 *
 *  Reactive receive:
 *  With data receive callback installed, data received is transferred to
 *  middleware's packet pool through this callback. Any ESP WiFi APIs can
 *  trigger reactive receive.
 *
 *  @note
 *  Due to view point, ESP AT has confusing definition on active/passive receive mode.
 *
 *  @note
 *  Supporting proactive/reactive, middleware must fetch data from middleware's
 *  packet pool first and then go proactive to avoid received data disorder.
 *
 *  @note
 *  Middleware must guarantee all ESP WiFi APIs are serialized.
 *
 *  @retval Operation status
 */
ESP_WIFI_Status_t ESP_WIFI_Recv( uint8_t LinkID, uint8_t * pcBuf, 
                                 uint16_t usReqLen, uint16_t * usRecvLen );
ESP_WIFI_Status_t ESP_WIFI_Recv2( uint8_t LinkID, uint8_t * pcBuf, 
                                 uint16_t usReqLen, uint16_t * usRecvLen, uint32_t ulTimeout );

/**
 * @brief  Check if proactive receive
 */
bool ESP_WIFI_IsProactRecv( uint8_t LinkID );
                       
/**
 * @brief  Check the module is connected to an AP
 */
bool ESP_WIFI_IsConnected( void );

#ifdef __cplusplus
}
#endif

#endif /*__ESP8266_WIFI_H__*/
