/**************************************************************************//**
 * @file     netx_esp8266_network.c
 * @version  V1.00
 * @brief    NetX/ESP8266 WiFi network source file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

/* Standard includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* ESP includes */
#include "esp8266_wifi.h"
#include "netx_esp8266_network.h"

/* Azure RTOS includes */
#include "nx_api.h"
//#include "nx_secure_tls_api.h"
#include "nx_wifi.h"
#include "nxd_dns.h"

/* printf-like log output function */
#ifndef NXESP_CFG_LOG_OUT
#define NXESP_CFG_LOG_OUT(FMT, ...)                 \
    do {                                            \
        printf(FMT, ## __VA_ARGS__);                \
    } while (0)
#endif

/* Prefixed log output function */
#define NXESP_LOG_OUT(FMT, ...)                     \
    do {                                            \
        NXESP_CFG_LOG_OUT("[NXESP]");               \
        NXESP_CFG_LOG_OUT(FMT, ## __VA_ARGS__);     \
    } while (0)

/* printf message color */
#define NXESP_MSG_CLR_NONE     "\033[0m"       /* None */
#define NXESP_MSG_CLR_RED      "\033[1;31m"    /* Red */
#define NXESP_MSG_CLR_YELLOW   "\033[1;33m"    /* Yellow */
#define NXESP_MSG_CLR_GREEN    "\033[1;32m"    /* Green */
#define NXESP_MSG_CLR_CYAN     "\033[1;36m"    /* Cyan */
#define NXESP_MSG_CLR_PURPLE   "\033[1;35m"    /* Purple */

#define NXESP_CHK_BOOL(EXPR)                                                        \
    do {                                                                            \
        if (!(EXPR)) {                                                              \
            NXESP_CFG_LOG_OUT(NXESP_MSG_CLR_YELLOW);                                \
            NXESP_CFG_LOG_OUT("NXESP CHECK FAILURE: " #EXPR "\r\n");                \
            NXESP_CFG_LOG_OUT("FILE: %s+%d\r\n", __FILE__, __LINE__);               \
            NXESP_CFG_LOG_OUT(NXESP_MSG_CLR_NONE);                                  \
            status = NX_NOT_SUCCESSFUL;                                             \
            goto clean_up;                                                          \
        }                                                                           \
    } while (0)

#define NXESP_CHK_STATUS(EXPR)                                          \
    do {                                                                \
        status = (EXPR);                                                \
        if (status != NX_SUCCESS) {                                     \
            NXESP_CFG_LOG_OUT(NXESP_MSG_CLR_YELLOW);                    \
            NXESP_CFG_LOG_OUT("NXESP CHECK FAILURE: " #EXPR "\r\n");    \
            NXESP_CFG_LOG_OUT("Expected 0x%08x, Got 0x%08x\r\n", NX_SUCCESS, status);   \
            NXESP_CFG_LOG_OUT("FILE: %s+%d\r\n", __FILE__, __LINE__);   \
            NXESP_CFG_LOG_OUT(NXESP_MSG_CLR_NONE);                      \
            goto clean_up;                                              \
        }                                                               \
    } while (0)

static UINT esp8266_wifi_initialize(CHAR *ssid, CHAR *password, ESP_WIFI_Security_t mode);
static UINT dns_client_create(void);

#define THREADX_PACKET_COUNT 20
#define THREADX_PACKET_SIZE  ESP_CFG_PAYLOAD_MAXSIZE    /* Set to ESP8266 max payload size */
#define THREADX_POOL_SIZE    ((THREADX_PACKET_SIZE + sizeof(NX_PACKET)) * THREADX_PACKET_COUNT)

static UCHAR threadx_ip_pool[THREADX_POOL_SIZE];

NX_IP           nx_ip;
NX_PACKET_POOL  nx_pool;
NX_DNS          nx_dns_client;

UINT netx_esp8266_network_initialize(CHAR *ssid, CHAR *password, ESP_WIFI_Security_t mode)
{
    UINT status = NX_SUCCESS;
    bool packet_pool_created = false;
    bool ip_created = false;    
    bool dns_client_created = false;

    /* Initialize ESP8266 WiFi */
    NXESP_CHK_STATUS(esp8266_wifi_initialize(ssid, password, mode));

    /* Initialize NetX system */
    nx_system_initialize();

    /* Create a packet pool */
    NXESP_CHK_STATUS(nx_packet_pool_create(&nx_pool, "NetX Packet Pool", THREADX_PACKET_SIZE, threadx_ip_pool, THREADX_POOL_SIZE));
    packet_pool_created = true;

    /* Create an IP instance */
    NXESP_CHK_STATUS(nx_ip_create(&nx_ip, "NetX IP Instance 0", 0, 0, &nx_pool, NULL,  NULL, 0, 0));
    ip_created = true;

    /* Initialize NetX WiFi */
    NXESP_CHK_STATUS(nx_wifi_initialize(&nx_ip, &nx_pool));

    /* Initialize TLS */
    //nx_secure_tls_initialize();
    
    /* Create DNS client */
    NXESP_CHK_STATUS(dns_client_create());
    dns_client_created = true;

    /* Success return */
    return status;

clean_up:

    if (dns_client_created) {
        nx_dns_delete(&nx_dns_client);
        dns_client_created = false;
    }

    if (ip_created) {
        nx_ip_delete(&nx_ip);
        ip_created = false;
    }

    if (packet_pool_created) {
        nx_packet_pool_delete(&nx_pool);
        packet_pool_created = false;
    }

    /* Failure return */
    return status;
}

static UINT esp8266_wifi_initialize(CHAR *ssid, CHAR *password, ESP_WIFI_Security_t mode)
{
    UINT status = NX_SUCCESS;

    /* Check validity of arguments */
    NXESP_CHK_BOOL(ssid);
    NXESP_CHK_BOOL(password);

    /* Initialize ESP modem */
    NXESP_CHK_BOOL(ESP_WIFI_Init() == ESP_WIFI_STATUS_OK);

    /* ESP modem F/W version */
    {
        ESP_WIFI_FW_AT_Version_t at_version;
        ESP_WIFI_FW_SDK_Version_t sdk_version;
        NXESP_CHK_BOOL(ESP_WIFI_GetFirmwareVersion(&at_version, &sdk_version) == ESP_WIFI_STATUS_OK);
        NXESP_LOG_OUT("AT Version: %d.%d.%d\r\n", at_version.major, at_version.minor, at_version.patch);
        NXESP_LOG_OUT("SDK Version: %d.%d.%d\r\n", sdk_version.major, sdk_version.minor, sdk_version.patch);
    }

    /* Configure to Station mode */
    NXESP_CHK_BOOL(ESP_WIFI_SetWiFiMode(ESP_WIFI_STATION) == ESP_WIFI_STATUS_OK);
    NXESP_LOG_OUT("Configure to Station mode\r\n");

    /* Enable Station DHCP client */
    NXESP_CHK_BOOL(ESP_WIFI_EnableDHCP(ESP_WIFI_STATION, true) == ESP_WIFI_STATUS_OK);
    NXESP_LOG_OUT("Enable Station mode DHCP client\r\n");

    NXESP_CHK_BOOL(ESP_WIFI_Connect(ssid, password) == ESP_WIFI_STATUS_OK);
    NXESP_LOG_OUT("Connect to AP with SSID=%s,PASSWORD=********\r\n", ssid);
    
    /* IP/MAC addresses */
    {
        uint8_t ApIpAddr[4];
        uint8_t ApMacAddr[6];
        uint8_t StaIpAddr[4];
        uint8_t StaMacAddr[6];

        NXESP_CHK_BOOL(ESP_WIFI_GetNetStatus(ApIpAddr, ApMacAddr, StaIpAddr, StaMacAddr) == ESP_WIFI_STATUS_OK);
        NXESP_LOG_OUT("SoftAP IP: %d.%d.%d.%d\r\n", ApIpAddr[0], ApIpAddr[1], ApIpAddr[2], ApIpAddr[3]);
        NXESP_LOG_OUT("SoftAP MAC: %2x.%2x.%2x.%2x.%2x.%2x\r\n", ApMacAddr[0], ApMacAddr[1], ApMacAddr[2], ApMacAddr[3], ApMacAddr[4], ApMacAddr[5]);
        NXESP_LOG_OUT("Station IP: %d.%d.%d.%d\r\n", StaIpAddr[0], StaIpAddr[1], StaIpAddr[2], StaIpAddr[3]);
        NXESP_LOG_OUT("Station MAC: %2x.%2x.%2x.%2x.%2x.%2x\r\n", StaMacAddr[0], StaMacAddr[1], StaMacAddr[2], StaMacAddr[3], StaMacAddr[4], StaMacAddr[5]);
    }

    /* Success return */
    return status;

clean_up:

    /* Failure return */
    return status;
}


static UINT dns_client_create(void)
{
    UINT status = NX_SUCCESS;
    bool dns_client_created = false;
    /* resolver1.opendns.com(208.67.222.222) */
    UCHAR dns_address_1[4] = {208, 67, 222, 222};

    NXESP_CHK_STATUS(nx_dns_create(&nx_dns_client, &nx_ip, (UCHAR *) "DNS Client"));
    dns_client_created = true;

    /* Use the packet pool here */
#ifdef NX_DNS_CLIENT_USER_CREATE_PACKET_POOL 
    NXESP_CHK_STATUS(nx_dns_packet_pool_set(&nx_dns_client, nx_ip.nx_ip_default_packet_pool));
#endif

    /* Add an IPv4 server address to the client list */
    NXESP_CHK_STATUS(nx_dns_server_add(&nx_dns_client, IP_ADDRESS(dns_address_1[0], dns_address_1[1], dns_address_1[2], dns_address_1[3])));

    /* Output DNS server address */
    NXESP_LOG_OUT("DNS server address: %d.%d.%d.%d\r\n", dns_address_1[0], dns_address_1[1], dns_address_1[2], dns_address_1[3]);

    /* Success return */
    return status;

clean_up:
    
    if (dns_client_created) {
        nx_dns_delete(&nx_dns_client);
        dns_client_created = false;
    }

    /* Failure return */
    return status;
}
