/**************************************************************************//**
 * @file     netx_esp8266_network.h
 * @version  V1.00
 * @brief    NetX/ESP8266 WiFi network header file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __NETX_ESP8266_NETWORK_H__
#define __NETX_ESP8266_NETWORK_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Azure RTOS includes */
#include "nx_api.h"
#include "nxd_dns.h"

extern NX_IP          nx_ip;
extern NX_PACKET_POOL nx_pool;
extern NX_DNS         nx_dns_client;

/**
 * @brief  Initialize NetX/ESP8266 WiFi network interface
 */
UINT netx_esp8266_network_initialize(CHAR *ssid, CHAR *password, ESP_WIFI_Security_t mode);

#ifdef __cplusplus
}
#endif

#endif /*__NETX_ESP8266_NETWORK_H__*/
