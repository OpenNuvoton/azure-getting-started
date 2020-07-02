/**************************************************************************//**
 * @file     esp8266_wifi.c
 * @version  V1.00
 * @brief    M480 series ESP8266 WiFi driver source file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "stdlib.h"

/* Azure RTOS includes */
#include "tx_api.h"

/* ESP includes */
#include "esp8266_wifi.h"
#include "driver/esp_ll.h"

/* ESP AT command delay/timeout */
#define ESP_DELAY_RST_ASSERT        2
#define ESP_DELAY_RST_DEASSERT      200
#define ESP_TIMEOUT_WIFI_CONNECT    20000
#define ESP_TIMEOUT_NORMAL          2000
#define ESP_TIMEOUT_READY           2000
#define ESP_TIMEOUT_OPENLINK        20000
#define ESP_TIMEOUT_CLOSELINK       10000
#define ESP_TIMEOUT_SEND            5000
#define ESP_TIMEOUT_RECV            5000
#define ESP_TIMEOUT_SEND_EXTRA      2000
#define ESP_TIMEOUT_RECV_EXTRA      1000
#define ESP_DELAY_OPENLINK_RETRY    1000
#define ESP_DELAY_BUSY_RETRY        500

/* ESP AT command response messages */
#define AT_RESP_OK                  "OK"
#define AT_RESP_ERROR               "ERROR"
#define AT_RESP_FAIL                "FAIL"
#define AT_RESP_SEND                ">"
#define AT_RESP_CIPRECVDATA         "+CIPRECVDATA,"

/* ESP AT command OOB message */
#define AT_OOB_READY                "ready"
#define AT_OOB_WIFI_CONNECTED       "WIFI CONNECTED"
#define AT_OOB_WIFI_GOTIP           "WIFI GOT IP"
#define AT_OOB_WIFI_DISCONNECT      "WIFI DISCONNECT"
#define AT_OOB_BUSY_SEND            "busy s..."
#define AT_OOB_BUSY_PROCESS         "busy p..."
#define AT_OOB_CONNECT              "CONNECT"
#define AT_OOB_CLOSED               "CLOSED"
#define AT_OOB_IPD                  "+IPD,"
#define AT_OOB_SENDOK               "SEND OK"
#define AT_OOB_SENDFAIL             "SEND FAIL"

/* ESP AT command OOB message bitmap */
#define AT_OOB_BM_READY             (1 << 0)
#define AT_OOB_BM_STARTSEND         (1 << 1)
#define AT_OOB_BM_SENDOK            (1 << 2)
#define AT_OOB_BM_SENDFAIL          (1 << 3)
#define AT_OOB_BM_IPD               (1 << 4)
#define AT_OOB_BM_LINKCLOSED        (1 << 5)


typedef struct {
    /* Caller-provided */
    uint8_t LinkID;
    ESP_WIFI_ConnType_t Type;
    uint8_t RemoteIP[4];
    uint16_t RemotePort;
    uint8_t RemoteIPAlt[4];
    uint16_t RemotePortAlt;
    union {
        struct {
            uint16_t TcpKeepAlive;
        } Tcp;
        struct {
            uint16_t UdpLocalPort;
            uint8_t UdpMode;
        } Udp;
    };

    /* ESP WiFi driver-managed */
    uint8_t Status;
    union {
        struct {
            uint16_t NewIpdDataAvbl;
        } ProactRecv;
        struct {
            uint16_t NewIpdDataRecvd;
        } ReactRecv;
    };
    bool IsConnected;
    bool IsPassiveMode;
    bool IsRemoteAlt;
} ESP_WIFI_Conn_t;

/** ESP WiFi driver control block
 */
typedef struct {
    uint8_t CmdData[ESP_CFG_CMDDATABUF_SIZE];
    uint64_t PrevATCmdCoolDownTime;
    bool IsConnected;
    bool IsMultiConnEnabled;
    bool IsPassiveModeEnabled;

    uint8_t ApIpAddr[4];
    uint8_t ApMacAddr[6];
    uint8_t StaIpAddr[4];
    uint8_t StaMacAddr[6];

    /* Connections reflecting ESP modem's sockets */
    ESP_WIFI_Conn_t Conns[ESP_CFG_SOCKET_COUNT];

    /* Proactive receive */
    struct {
        uint8_t *           Data;       // Pointer to data buffer
        uint16_t            MaxSize;    // Max size of data buffer
        uint16_t            Size;       // Actual size of data buffer
    } ProactRecv;

    /* Reactive receive */
    struct {
        struct {
            uint8_t Buffer[ESP_CFG_IPDBUF_SIZE];
            uint16_t Size;
        } IpdData;
        struct {
            ESP_WIFI_DataRecvCallback_t Callback;
            void *                      Ctx;
        } DataRecvCb;
    } ReactRecv;
} ESP_WIFI_t;

static ESP_WIFI_t esp_wifi_inst;

/** Software reset ESP modem
 */
static ESP_WIFI_Status_t ESP_WIFI_SoftReset( void );

/** Enable echo or not
 */
static ESP_WIFI_Status_t ESP_WIFI_EnableEcho( bool enable );

/**
 * @brief  Enable or Disable Multiple Connections
 * @retval Operation status
 */
static ESP_WIFI_Status_t ESP_WIFI_EnableMultiConn( bool enable );

/**
 * @brief  Enable or Disable TCP Passive Mode
 * @retval Operation status
 */
static ESP_WIFI_Status_t ESP_WIFI_EnableTcpPassive( bool enable );

/**
 * @brief  Enable H/W UART Flow Control
 * @retval Operation status
 */
static ESP_WIFI_Status_t ESP_WIFI_EnableSerialFlowCtrl( bool enable );

/**
 * @brief  Check the DNS server setting
 * @retval Operation status
 */
static ESP_WIFI_Status_t ESP_WIFI_CheckDnsServer( void );

/**
 * @brief  Write data to UART interface
 */
static uint16_t ESP_IO_Send( uint8_t pucTxBuf[], uint16_t usWriteBytes );

/**
 * @brief  Read data from UART interface
 */
static ESP_WIFI_Status_t ESP_IO_Recv( uint8_t pucRxBuf[], uint16_t usReadBytes, uint32_t *ulOobBitmap,
                               uint32_t xTimeout );
                               
                               /**
 * @brief  Execute AT command
 * @param  pucCmd                   Pointer to AT command string
 * @param  ulATCmdCoolDownTime      AT command cool down time in ms before starting next one
 * @param  ulATCmdTimeout           AT command timeout in ms
 * @retval Operation status
 */
static ESP_WIFI_Status_t ESP_AT_Command( uint8_t * pucCmd, uint32_t ulATCmdCoolDownTime, uint32_t ulATCmdTimeout );

/**
 * @brief  Parse an IP address
 */
static void ParseIpAddr( char * string, uint8_t * addr );

/**
 * @brief  Parse an MAC address
 */
static void ParseMacAddr( char * string, uint8_t * addr );

/**
 * @brief  Parse an Access Point information
 */
static void ParseApDetail( char * string, ESP_WIFI_ScanResult_t *xBuffer );

/**
 * @brief  Get device IP and MAC address
 * @param  Obj: pointer to WiF module
 * @retval None
 */
static void AT_ParseAddress( void );

/**
 * @brief  Get scanned AP information
 */
static void AT_ParseAccessPoint( ESP_WIFI_ScanResult_t * pxBuffer, uint8_t ucNumNetworks );

/**
 * @brief  Parse firmware version
 */
static void At_ParseFirmwareVersion( ESP_WIFI_FW_AT_Version_t *at_version, ESP_WIFI_FW_SDK_Version_t *sdk_version );

ESP_WIFI_Status_t ESP_WIFI_Init( void )
{
    ESP_WIFI_Status_t xRet = ESP_WIFI_STATUS_ERROR;

    /* Initialize ESP low-level driver */
    xRet = ESP_LL_Init() ? ESP_WIFI_STATUS_OK : ESP_WIFI_STATUS_ERROR;
    if (xRet != ESP_WIFI_STATUS_OK) {
        return xRet;
    }

    /* Reset ESP modem */
    xRet = ESP_WIFI_Reset();
    if (xRet != ESP_WIFI_STATUS_OK) {
        return xRet;
    }

    xRet = ESP_WIFI_CheckDnsServer();
    if (xRet != ESP_WIFI_STATUS_OK) {
        return xRet;
    }

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_InstallDataRecvCallback( ESP_WIFI_DataRecvCallback_t Callback, void *Ctx )
{
    esp_wifi_inst.ReactRecv.DataRecvCb.Callback = Callback;
    esp_wifi_inst.ReactRecv.DataRecvCb.Ctx = Ctx;
    
    return ESP_WIFI_STATUS_OK;
}

ESP_WIFI_Status_t ESP_WIFI_GetFirmwareVersion( ESP_WIFI_FW_AT_Version_t *at_version, ESP_WIFI_FW_SDK_Version_t *sdk_version )
{
    ESP_WIFI_Status_t xRet;

    ESP_LOG_DBG("Get firmware version...\r\n");

    sprintf((char *)esp_wifi_inst.CmdData, "AT+GMR\r\n");

    xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, ESP_TIMEOUT_NORMAL);
    if (xRet == ESP_WIFI_STATUS_OK) {
        At_ParseFirmwareVersion(at_version, sdk_version);
        ESP_LOG_DBG("Get firmware version...SUCCESS\r\n");
        ESP_LOG_INFO("AT version: %d.%d.%d\r\n", at_version->major, at_version->minor, at_version->patch);
        ESP_LOG_INFO("SDK version: %d.%d.%d\r\n", sdk_version->major, sdk_version->minor, sdk_version->patch);        
    } else {
        ESP_LOG_CRIT("Get firmware version...FAILURE\r\n");
    }

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_SetWiFiMode( ESP_WIFI_Mode_t mode )
{
    ESP_WIFI_Status_t xRet;

    if (mode == ESP_WIFI_STATION) {
        ESP_LOG_DBG("Set WiFi mode to Station...\r\n");
    } else if (mode == ESP_WIFI_SOFTAP) {
        ESP_LOG_DBG("Set WiFi mode to Soft AP...\r\n");
    } else if (mode == ESP_WIFI_SOFTAP) {
        ESP_LOG_DBG("Set WiFi mode to Soft AP+Station...\r\n");
    } else {
        ESP_LOG_CRIT("Unknown WiFi mode %d...\r\n", mode);
        return ESP_WIFI_STATUS_ERROR;
    }

    sprintf((char *)esp_wifi_inst.CmdData, "AT+CWMODE_CUR=%d\r\n", mode);

    xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, ESP_TIMEOUT_NORMAL);
    if (xRet == ESP_WIFI_STATUS_OK) {
        ESP_LOG_DBG("Set WiFi mode...SUCCESS\r\n");
    } else {
        ESP_LOG_CRIT("Set WiFi mode... ERROR\r\n");
    }

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_EnableDHCP( ESP_WIFI_Mode_t mode, bool enable )
{
    ESP_WIFI_Status_t xRet;

    if (mode == ESP_WIFI_STATION) {
        ESP_LOG_DBG("%s WiFi mode Station DHCP...\r\n", enable ? "Enable" : "Disable");
    } else if (mode == ESP_WIFI_SOFTAP) {
         ESP_LOG_DBG("%s WiFi mode Soft AP DHCP...\r\n", enable ? "Enable" : "Disable");
    } else if (mode == ESP_WIFI_SOFTAP) {
        ESP_LOG_DBG("%s WiFi mode Soft AP+Station DHCP...\r\n", enable ? "Enable" : "Disable");
    } else {
        ESP_LOG_CRIT("Unknown WiFi mode %d...\r\n", mode);
        return ESP_WIFI_STATUS_ERROR;
    }

    sprintf((char *)esp_wifi_inst.CmdData, "AT+CWDHCP_CUR=%d,%d\r\n", mode, enable);

    xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, ESP_TIMEOUT_NORMAL);
    if (xRet == ESP_WIFI_STATUS_OK) {
        if (mode == ESP_WIFI_STATION) {
            ESP_LOG_DBG("%s WiFi mode Station DHCP...SUCCESS\r\n", enable ? "Enable" : "Disable");
        } else if (mode == ESP_WIFI_SOFTAP) {
            ESP_LOG_DBG("%s WiFi mode Soft AP DHCP...SUCCESS\r\n", enable ? "Enable" : "Disable");
        } else if (mode == ESP_WIFI_SOFTAP) {
            ESP_LOG_DBG("%s WiFi mode Soft AP+Station DHCP...SUCCESS\r\n", enable ? "Enable" : "Disable");
        }
    } else {
        if (mode == ESP_WIFI_STATION) {
            ESP_LOG_CRIT("%s WiFi mode Station DHCP...FAILURE\r\n", enable ? "Enable" : "Disable");
        } else if (mode == ESP_WIFI_SOFTAP) {
            ESP_LOG_CRIT("%s WiFi mode Soft AP DHCP...FAILURE\r\n", enable ? "Enable" : "Disable");
        } else if (mode == ESP_WIFI_SOFTAP) {
            ESP_LOG_CRIT("%s WiFi mode Soft AP+Station DHCP...FAILURE\r\n", enable ? "Enable" : "Disable");
        }
    }

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_Connect( const char * cSSID, const char * cPassword )
{
    ESP_WIFI_Status_t xRet;

    ESP_LOG_INFO("Connect to AP \"%s\"...\r\n", cSSID);
    ESP_LOG_DBG("SSID=%s, PASSWORD=%s\r\n", cSSID, cPassword);

    sprintf((char *)esp_wifi_inst.CmdData, "AT+CWJAP_CUR=\"%s\",\"%s\"\r\n", cSSID, cPassword);

    xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, ESP_TIMEOUT_WIFI_CONNECT);
    if (xRet == ESP_WIFI_STATUS_OK) {
        esp_wifi_inst.IsConnected = true;
        ESP_LOG_INFO("Connect to AP \"%s\"...SUCCESS\r\n", cSSID);
    } else {
        ESP_LOG_CRIT("Connect to AP \"%s\"...FAILURE\r\n", cSSID);
    }

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_Disconnect( void )
{
    ESP_WIFI_Status_t xRet;

    xRet = ESP_AT_Command((uint8_t *)"AT+CWQAP\r\n", 0, ESP_TIMEOUT_NORMAL);
    if (xRet == ESP_WIFI_STATUS_OK) {
        esp_wifi_inst.IsConnected = false;
        /* Prevent scan AP soon and returns an error */
        ESP_Plat_Wait_MS(10);
    }

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_Reset( void )
{
    ESP_WIFI_Status_t xRet = ESP_WIFI_STATUS_OK;
    ESP_WIFI_FW_AT_Version_t at_version;
    ESP_WIFI_FW_SDK_Version_t sdk_version;

    /* Clear ESP modem driver control block */
    memset(&esp_wifi_inst, 0x00, sizeof(esp_wifi_inst));

#if ESP_CFG_SERIAL_FC
    /* Disable serial RTS/CTS flow control */
    xRet = ESP_WIFI_EnableSerialFlowCtrl(false);
    if (xRet != ESP_WIFI_STATUS_OK) {
        goto clean_up;
    }
#endif

#if ESP_CFG_HW_RST
    /* H/W reset */
    ESP_LOG_DBG("H/W reset...\r\n");
    ESP_LL_ResetModem(ESP_DELAY_RST_ASSERT, ESP_DELAY_RST_DEASSERT);
    ESP_LL_RxStrm_Purge();
    ESP_LOG_DBG("H/W reset...SUCCESS\r\n");
#endif

    /* S/W reset */
    xRet = ESP_WIFI_SoftReset();
    if (xRet != ESP_WIFI_STATUS_OK) {
        goto clean_up;
    }

    /* Disable echo */
    xRet = ESP_WIFI_EnableEcho(false);
    if (xRet != ESP_WIFI_STATUS_OK) {
        goto clean_up;
    }

#if ESP_CFG_SERIAL_FC
    /* Enable serial RTS/CTS flow control */
    xRet = ESP_WIFI_EnableSerialFlowCtrl(true);
    if (xRet != ESP_WIFI_STATUS_OK) {
        goto clean_up;
    }
#endif

    /* Get firmware version */
    xRet = ESP_WIFI_GetFirmwareVersion(&at_version, &sdk_version);
    if (xRet != ESP_WIFI_STATUS_OK) {
        goto clean_up;
    }

#if ESP_CFG_TCP_PASSIVE_RECV_MODE
    /* Enable TCP passive mode */
    xRet = ESP_WIFI_EnableTcpPassive(true);
    if (xRet != ESP_WIFI_STATUS_OK) {
        goto clean_up;
    }
#endif

    /* Set WiFi mode to Station */
    xRet = ESP_WIFI_SetWiFiMode(ESP_WIFI_STATION);
    if (xRet != ESP_WIFI_STATUS_OK) {
        goto clean_up;
    }

#if ESP_CFG_MULTICONN
    /* Enable multiple connections */
    xRet = ESP_WIFI_EnableMultiConn(true);
    if (xRet != ESP_WIFI_STATUS_OK) {
        goto clean_up;
    }
#endif

clean_up:

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_Scan( ESP_WIFI_ScanResult_t * pxBuffer, uint8_t ucNumNetworks )
{
    ESP_WIFI_Status_t xRet;

    xRet = ESP_AT_Command("AT+CWLAP\r\n", 0, ESP_TIMEOUT_NORMAL);

    if (xRet == ESP_WIFI_STATUS_OK) {
        AT_ParseAccessPoint(pxBuffer, ucNumNetworks);
    }

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_GetNetStatus( uint8_t ApIpAddr[], uint8_t ApMacAddr[],
                                         uint8_t StaIpAddr[], uint8_t StaMacAddr[] )
{
    ESP_WIFI_Status_t xRet;

    ESP_ASSERT(ApIpAddr);
    ESP_ASSERT(ApMacAddr);
    ESP_ASSERT(StaIpAddr);
    ESP_ASSERT(StaMacAddr);

    xRet = ESP_AT_Command((uint8_t *)"AT+CIFSR\r\n", 30, ESP_TIMEOUT_NORMAL);

    if (xRet == ESP_WIFI_STATUS_OK) {
        AT_ParseAddress();
        
        memcpy(ApIpAddr, esp_wifi_inst.ApIpAddr, 4);
        memcpy(ApMacAddr, esp_wifi_inst.ApMacAddr, 6);
        memcpy(StaIpAddr, esp_wifi_inst.StaIpAddr, 4);
        memcpy(StaMacAddr, esp_wifi_inst.StaMacAddr, 6);
    } else {
        memset(ApIpAddr, 0x00, 4);
        memset(ApMacAddr, 0x00, 6);
        memset(StaIpAddr, 0x00, 4);
        memset(StaMacAddr, 0x00, 6);
    }

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_GetConnStatus( uint8_t LinkID )
{
    ESP_WIFI_Status_t xRet;
    char *pcDelim = ":,\r\n";
    char *pcPtr;
    ESP_WIFI_Conn_t *pxConn;
    
    ESP_ASSERT(LinkID < ESP_CFG_SOCKET_COUNT);
    pxConn = &esp_wifi_inst.Conns[LinkID];

    xRet = ESP_AT_Command((uint8_t *)"AT+CIPSTATUS\r\n", 0, ESP_TIMEOUT_NORMAL);

    if (xRet == ESP_WIFI_STATUS_OK) {
        pxConn->IsConnected = false;
        pcPtr = strtok((char *)esp_wifi_inst.CmdData, pcDelim);
        /* Skip the first line of response */
        pcPtr = strtok(NULL, pcDelim);

        while (pcPtr != NULL){
            if (strcmp(pcPtr, "+CIPSTATUS") == 0) {
                pcPtr = strtok(NULL, pcDelim);
                if (pxConn->LinkID == (uint8_t)atoi(pcPtr)) {
                    pxConn->IsConnected = true;
                    break;
                }
            } else if (strcmp(pcPtr, "STATUS") == 0) {
                pcPtr = strtok(NULL, pcDelim);
                pxConn->Status = (uint8_t)atoi(pcPtr);
            }
            pcPtr = strtok(NULL, pcDelim);
        }
    }

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_Ping( uint8_t * pucIPAddr )
{
    sprintf((char *)esp_wifi_inst.CmdData, "AT+PING=\"%d.%d.%d.%d\"\r\n", pucIPAddr[0], pucIPAddr[1], pucIPAddr[2], pucIPAddr[3]);

    return ESP_AT_Command(esp_wifi_inst.CmdData, 0, ESP_TIMEOUT_NORMAL);
}

ESP_WIFI_Status_t ESP_WIFI_GetHostIP( const char * pcHost, uint8_t * pucIPAddr )
{
    ESP_WIFI_Status_t xRet;
    char *pcDelim = ":\r\n";
    char *pcPtr;

    sprintf((char *)esp_wifi_inst.CmdData, "AT+CIPDOMAIN=\"%s\"\r\n", pcHost);
    xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, ESP_TIMEOUT_NORMAL);

    if (xRet == ESP_WIFI_STATUS_OK) {
        xRet = ESP_WIFI_STATUS_ERROR;

        pcPtr = strtok((char *)esp_wifi_inst.CmdData, pcDelim);
        while (pcPtr != NULL){
            if (strcmp(pcPtr, "+CIPDOMAIN") == 0) {
                pcPtr = strtok(NULL, pcDelim);
                ParseIpAddr(pcPtr, pucIPAddr);
                xRet = ESP_WIFI_STATUS_OK;
                break;
            }
            pcPtr = strtok(NULL, pcDelim);
        }
    }

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_OpenClientConnection( uint8_t LinkID, ESP_WIFI_ConnType_t Type, uint8_t *RemoteIP,
                                                 uint16_t RemotePort, uint32_t Option)
{
    return ESP_WIFI_OpenClientConnection2(LinkID, Type, RemoteIP, RemotePort, Option, ESP_TIMEOUT_OPENLINK);
}

ESP_WIFI_Status_t ESP_WIFI_OpenClientConnection2( uint8_t LinkID, ESP_WIFI_ConnType_t Type, uint8_t *RemoteIP,
                                                 uint16_t RemotePort, uint32_t Option, uint32_t ulTimeout )
{
    ESP_WIFI_Status_t xRet;
    uint8_t ucCount;
    ESP_WIFI_Conn_t *pxConn;
    uint64_t xTickCurrent, xTickEnd;
    uint32_t xTickTimeout;

    ESP_ASSERT(LinkID < ESP_CFG_SOCKET_COUNT);
    pxConn = &esp_wifi_inst.Conns[LinkID];

    xTickEnd = ESP_Plat_GetTime_MS() + ulTimeout;

    /* Remaining timeout */
    xTickCurrent = ESP_Plat_GetTime_MS();    
    xTickTimeout = (xTickEnd > xTickCurrent) ? (xTickEnd - xTickCurrent) : 0;

    /* Close it first on open */
    if (pxConn->IsConnected) {
        ESP_WIFI_CloseClientConnection2(LinkID, xTickTimeout);
    }

    /* Error it if we still cannot close previous link */
    if (pxConn->IsConnected) {
        ESP_LOG_WARN("Open link(%d)...TIMEOUT(%d). Failed to close previous open one\r\n", LinkID, ulTimeout);
        return ESP_WIFI_STATUS_TIMEOUT;
    }
    
    while (1) {

        /* Remaining timeout */
        xTickCurrent = ESP_Plat_GetTime_MS();    
        xTickTimeout = (xTickEnd > xTickCurrent) ? (xTickEnd - xTickCurrent) : 0;

        if (xTickTimeout) {
            ESP_LOG_DBG("Open link(%d)...(RMN %d ms)\r\n", LinkID, xTickTimeout);
        } else {
            ESP_LOG_WARN("Open link(%d)...TIMEOUT(%d ms)\r\n", LinkID, ulTimeout);
            break;
        }
    
        /* Reset conn control block */
        memset(pxConn, 0x00, sizeof(*pxConn));

        pxConn->LinkID = LinkID;
        pxConn->Type = Type;
        memcpy(pxConn->RemoteIP, RemoteIP, 4);
        pxConn->RemotePort = RemotePort;
    
        /* Set multiple connection */
        if (esp_wifi_inst.IsMultiConnEnabled) {
            sprintf((char *)esp_wifi_inst.CmdData, "AT+CIPSTART=%d,", pxConn->LinkID);
        } else {
            sprintf((char *)esp_wifi_inst.CmdData, "AT+CIPSTART=");
        }

        pxConn->IsPassiveMode = false;
        /* Set connection type */
        switch (pxConn->Type) {
        case ESP_WIFI_TCP:
            sprintf((char *)esp_wifi_inst.CmdData + strlen((char *)esp_wifi_inst.CmdData), "\"TCP\",");
            pxConn->IsPassiveMode = esp_wifi_inst.IsPassiveModeEnabled;
            pxConn->Tcp.TcpKeepAlive = (uint16_t) (Option & 0xFFFF);
            break;
        case ESP_WIFI_UDP:
            sprintf((char *)esp_wifi_inst.CmdData + strlen((char *)esp_wifi_inst.CmdData), "\"UDP\",");
            pxConn->Udp.UdpLocalPort= (uint16_t) (Option & 0xFFFF);;
            pxConn->Udp.UdpMode = (uint8_t) ((Option & 0xFF0000) >> 16);
            break;
        case ESP_WIFI_SSL:
            sprintf((char *)esp_wifi_inst.CmdData + strlen((char *)esp_wifi_inst.CmdData), "\"SSL\",");
            break;
        }

        /* Set remote IP address and port */
        sprintf((char *)esp_wifi_inst.CmdData + strlen((char *)esp_wifi_inst.CmdData), "\"%d", pxConn->RemoteIP[0]);
        for (ucCount = 1; ucCount < 4; ucCount++) {
            sprintf((char *)esp_wifi_inst.CmdData + strlen((char *)esp_wifi_inst.CmdData), ".%d", pxConn->RemoteIP[ucCount]);
        }
        sprintf((char *)esp_wifi_inst.CmdData + strlen((char *)esp_wifi_inst.CmdData), "\",%d", pxConn->RemotePort);

        /* Set remaining data */
        if (pxConn->Type == ESP_WIFI_UDP) {
            if (pxConn->Udp.UdpLocalPort > 0) {
                sprintf((char *)esp_wifi_inst.CmdData + strlen((char *)esp_wifi_inst.CmdData), ",%d,%d", pxConn->Udp.UdpLocalPort, pxConn->Udp.UdpMode);
            }
        } else {
            if (pxConn->Tcp.TcpKeepAlive > 0) {
                sprintf((char *)esp_wifi_inst.CmdData + strlen((char *)esp_wifi_inst.CmdData), ",%d", pxConn->Tcp.TcpKeepAlive);
            }
        }

        sprintf((char *)esp_wifi_inst.CmdData + strlen((char *)esp_wifi_inst.CmdData), "\r\n");
        xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, xTickTimeout);

        if (xRet == ESP_WIFI_STATUS_OK) {
            ESP_LOG_DBG("Open link(%d)...SUCCESS\r\n", LinkID);
            break;
        } else {
            /* Ignore intermediate error. The only error would be timeout. */
        }

        /* Wait for a while for retry */
        esp_wifi_inst.PrevATCmdCoolDownTime = ESP_Plat_GetTime_MS() + ESP_DELAY_OPENLINK_RETRY;
    }

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_CloseClientConnection( uint8_t LinkID )
{
    return ESP_WIFI_CloseClientConnection2(LinkID, ESP_TIMEOUT_CLOSELINK);
}

ESP_WIFI_Status_t ESP_WIFI_CloseClientConnection2( uint8_t LinkID, uint32_t ulTimeout )
{
    ESP_WIFI_Status_t xRet;
    ESP_WIFI_Conn_t *pxConn;
    uint64_t xTickEnd;

    ESP_ASSERT(LinkID < ESP_CFG_SOCKET_COUNT);
    pxConn = &esp_wifi_inst.Conns[LinkID];

    ESP_LOG_DBG("Close link(%d)...\r\n", LinkID);

    xTickEnd = ESP_Plat_GetTime_MS() + ulTimeout;

    while (pxConn->IsConnected) {
        if (esp_wifi_inst.IsMultiConnEnabled) {
            sprintf((char *)esp_wifi_inst.CmdData, "AT+CIPCLOSE=%d\r\n", LinkID);
        } else {
            sprintf((char *)esp_wifi_inst.CmdData, "AT+CIPCLOSE\r\n");
        }

        xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, ESP_TIMEOUT_NORMAL);
        switch (xRet) {
            case ESP_WIFI_STATUS_OK:    // Successfully closed
            case ESP_WIFI_STATUS_ERROR: // Has closed before
                pxConn->IsConnected = false;
                break;
                
            case ESP_WIFI_STATUS_BUSY:
                ESP_LOG_WARN("Close link(%d)...BUSY\r\n", LinkID);
                break;
                
            default:
                break;
        }

        if (ESP_Plat_GetTime_MS() > xTickEnd) {
            xRet = ESP_WIFI_STATUS_TIMEOUT;
            ESP_LOG_WARN("Close link(%d)...TIMEOUT(%d)\r\n", LinkID, ESP_TIMEOUT_CLOSELINK);
            break;
        }
    }

    /* Return 'OK' as long as socket is closed */
    if (!pxConn->IsConnected) {
        ESP_LOG_DBG("Close link(%d)...SUCCESS\r\n", LinkID);
        xRet = ESP_WIFI_STATUS_OK;
    }

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_SendCommon( uint8_t LinkID, uint8_t * pcBuf, 
                                       uint16_t usReqLen, uint16_t * usSendLen, uint32_t xTimeout )
{
    ESP_WIFI_Status_t xRet = ESP_WIFI_STATUS_OK;
    uint64_t xTickCurrent, xTickEnd;
    uint32_t xTickTimeout;
    uint16_t usAskSend, usRealSend, usDataSent = 0;
    uint8_t ucCount;
    ESP_WIFI_Conn_t *pxConn;
    
    ESP_ASSERT(LinkID < ESP_CFG_SOCKET_COUNT);
    pxConn = &esp_wifi_inst.Conns[LinkID];

    xTickEnd = ESP_Plat_GetTime_MS() + xTimeout;

    if (pcBuf == NULL) {
        ESP_LOG_WARN("Null send buffer\r\n");
        xRet = ESP_WIFI_STATUS_ERROR;
        return xRet;
    }

    if (usSendLen) {
        *usSendLen = 0;
    }

    while ((usDataSent < usReqLen) && (xRet == ESP_WIFI_STATUS_OK)) {
        /* Remote peer has closed the link */
        if (!pxConn->IsConnected) {
            ESP_LOG_INFO("Exit send procedure with link %d closed\r\n", LinkID);
            xRet = ESP_WIFI_STATUS_LINKCLOSED;
            break;
        }

        usAskSend = usReqLen - usDataSent;
        if (usAskSend > ESP_CFG_SEND_MAXSIZE) {
            usAskSend = ESP_CFG_SEND_MAXSIZE;
        }

        /* Set multiple connection */
        if (esp_wifi_inst.IsMultiConnEnabled) {
            sprintf((char *)esp_wifi_inst.CmdData, "AT+CIPSEND=%d,%d", pxConn->LinkID, usAskSend);
        } else {
            sprintf((char *)esp_wifi_inst.CmdData, "AT+CIPSEND=%d", usAskSend);
        }

        /* Set remote IP and port for UDP connection */
        /* ESP AT UDP mode for switching remote IP address/port
         *
         * 0: the destination peer entity of UDP will not change; this is the default setting.
         * 1: the destination peer entity of UDP can change once.
         * 2: the destination peer entity of UDP is allowed to change.
         */
        if ((pxConn->Type == ESP_WIFI_UDP) && (pxConn->Udp.UdpMode > 0) && pxConn->IsRemoteAlt) {
            sprintf((char *)esp_wifi_inst.CmdData + strlen((char *)esp_wifi_inst.CmdData), ",\"%d", pxConn->RemoteIPAlt[0]);
            for (ucCount = 1; ucCount < 4; ucCount++) {
                sprintf((char *)esp_wifi_inst.CmdData + strlen((char *)esp_wifi_inst.CmdData), ".%d", pxConn->RemoteIPAlt[ucCount]);
            }
            sprintf((char *)esp_wifi_inst.CmdData + strlen((char *)esp_wifi_inst.CmdData), "\",%d", pxConn->RemotePortAlt);
        }

        sprintf((char *)esp_wifi_inst.CmdData + strlen((char *)esp_wifi_inst.CmdData), "\r\n");
        /* Must wait a period of time after the CIPSEND AT command */
        xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 10, ESP_TIMEOUT_NORMAL);

        if (xRet == ESP_WIFI_STATUS_OK) {
            xRet = ESP_WIFI_STATUS_ERROR;

            /* We must continue anyway; otherwise, we will break send process */
            xTickCurrent = ESP_Plat_GetTime_MS();
            if ((xTickCurrent + ESP_TIMEOUT_SEND_EXTRA) >= xTickEnd) {
                xTickEnd = xTickCurrent + ESP_TIMEOUT_SEND_EXTRA;
            }

            xTickTimeout = xTickEnd - xTickCurrent;
            xRet = ESP_IO_Recv(esp_wifi_inst.CmdData, sizeof(esp_wifi_inst.CmdData), NULL, xTickTimeout);
            if (xRet == ESP_WIFI_STATUS_SEND) {
                /* WiFi module allows to send data */
                memcpy(esp_wifi_inst.CmdData, pcBuf + usDataSent, usAskSend);
                *(esp_wifi_inst.CmdData + usAskSend) = 0;
                usRealSend = ESP_IO_Send(esp_wifi_inst.CmdData, usAskSend);
                if (usRealSend > 0) {
                    do {
                        uint32_t OobBm = AT_OOB_BM_SENDOK | AT_OOB_BM_SENDFAIL | AT_OOB_BM_LINKCLOSED;

                        xTickCurrent = ESP_Plat_GetTime_MS();
                        if (xTickEnd > xTickCurrent) {
                            xTickTimeout = xTickEnd - xTickCurrent;
                        } else {
                            /* Wait for no "SEDN OK" yet, but can regard it as OK */
                            xRet = ESP_WIFI_STATUS_OK;
                            break;
                        }

                        /* Remote peer has closed the link */
                        if (!pxConn->IsConnected) {
                            ESP_LOG_INFO("Exit send procedure (wait for \"SEND OK\") with link %d closed\r\n", LinkID);
                            xRet = ESP_WIFI_STATUS_LINKCLOSED;
                            break;
                        }

                        xRet = ESP_IO_Recv(esp_wifi_inst.CmdData, sizeof(esp_wifi_inst.CmdData), &OobBm, xTickTimeout);
                        if (xRet == ESP_WIFI_STATUS_ERROR) {
                            /* Error */
                            ESP_LOG_WARN("Error in waiting for \"SEND OK\"\r\n");
                            break;
                        } else if (xRet == ESP_WIFI_STATUS_TIMEOUT) {
                            /* Wait for no "SEDN OK" yet, but can regard it as OK */
                            break;
                        } else if (xRet == ESP_WIFI_STATUS_OOB) {
                            if (OobBm & AT_OOB_BM_SENDOK) {
                                xRet = ESP_WIFI_STATUS_OK;
                                break;
                            } else if (OobBm & AT_OOB_BM_SENDFAIL) {
                                xRet = ESP_WIFI_STATUS_ERROR;
                                break;
                            } else if (OobBm & AT_OOB_BM_LINKCLOSED) {
                                ESP_LOG_INFO("Link %d has closed\r\n", (OobBm & 0xFF0000) >> 16);
                            }
                        }
                    } while (true);
                    usDataSent += usRealSend;
                }
            } else {
                /* Send stream broken*/
                ESP_LOG_CRIT("Wait for no \"<\" after \"AT+CIPSEND\"\r\n");
                xRet = ESP_WIFI_STATUS_ERROR;
                break;
            }
        }
        else if (xRet == ESP_WIFI_STATUS_ERROR) {
            /* We can receive "CLOSED" OOB during "AT+CIPSEND" procedure.
             * Convert 'error' to 'closed'. */
            /* Remote peer has closed the link */
            if (!pxConn->IsConnected) {
                ESP_LOG_INFO("Exit send procedure with link %d closed\r\n", LinkID);
                xRet = ESP_WIFI_STATUS_LINKCLOSED;
                break;
            }

            /* Error */
            ESP_LOG_DBG("AT+CIPSEND...ERROR\r\n");
            break;
        } else if (xRet == ESP_WIFI_STATUS_TIMEOUT) {
            /* Timeout with send stream still unbroken */
            break;
        } else {
            /* Ignore intermediate status code and continue */
            xRet = ESP_WIFI_STATUS_OK;
        }
    }

    /* Regard it as OK as long as there has been data successfully
     * transfered and there is no error */
    if (xRet != ESP_WIFI_STATUS_ERROR && usDataSent) {
        xRet = ESP_WIFI_STATUS_OK;
    }

    if (usSendLen) {
        *usSendLen = usDataSent;
    }
    
    return xRet;
}
                            
ESP_WIFI_Status_t ESP_WIFI_Send( uint8_t LinkID, uint8_t * pcBuf, 
                                 uint16_t usReqLen, uint16_t * usSendLen )
{
    return ESP_WIFI_Send2(LinkID, pcBuf, usReqLen, usSendLen, ESP_TIMEOUT_SEND);
}

ESP_WIFI_Status_t ESP_WIFI_Send2( uint8_t LinkID, uint8_t * pcBuf, 
                                  uint16_t usReqLen, uint16_t * usSendLen,
                                  uint32_t ulTimeout )
{
    ESP_WIFI_Conn_t *pxConn;
    
    ESP_ASSERT(LinkID < ESP_CFG_SOCKET_COUNT);
    pxConn = &esp_wifi_inst.Conns[LinkID];

    pxConn->IsRemoteAlt = false;
    return ESP_WIFI_SendCommon(LinkID, pcBuf, usReqLen, usSendLen, ulTimeout);
}

ESP_WIFI_Status_t ESP_WIFI_SendTo( uint8_t LinkID, uint8_t *pcBuf, 
                                   uint16_t usReqLen, uint16_t * usSendLen,
                                   uint8_t *RemoteIPAlt, uint16_t RemotePortAlt)
{
    return ESP_WIFI_SendTo2(LinkID, pcBuf, usReqLen, usSendLen, RemoteIPAlt, RemotePortAlt, ESP_TIMEOUT_SEND);
}

ESP_WIFI_Status_t ESP_WIFI_SendTo2( uint8_t LinkID, uint8_t *pcBuf, 
                                    uint16_t usReqLen, uint16_t * usSendLen,
                                    uint8_t *RemoteIPAlt, uint16_t RemotePortAlt,
                                    uint32_t ulTimeout)
{
    ESP_WIFI_Conn_t *pxConn;

    ESP_ASSERT(LinkID < ESP_CFG_SOCKET_COUNT);
    pxConn = &esp_wifi_inst.Conns[LinkID];

    pxConn->IsRemoteAlt = true;
    memcpy(pxConn->RemoteIPAlt, RemoteIPAlt, 4);
    pxConn->RemotePortAlt = RemotePortAlt;
    return ESP_WIFI_SendCommon(LinkID, pcBuf, usReqLen, usSendLen, ulTimeout);
}

/**
 * @brief  Receive Data for TCP passive mode
 * @retval Operation status
 */
ESP_WIFI_Status_t ESP_WIFI_Recv_TCP_PASSIVE( uint8_t LinkID, uint32_t xTimeout )
{
    ESP_WIFI_Status_t xRet = ESP_WIFI_STATUS_OK;
    ESP_WIFI_Conn_t *pxConn;

    ESP_ASSERT(LinkID < ESP_CFG_SOCKET_COUNT);
    pxConn = &esp_wifi_inst.Conns[LinkID];

    ESP_ASSERT(esp_wifi_inst.ProactRecv.Data);
    ESP_ASSERT(esp_wifi_inst.ProactRecv.MaxSize);

    // +CIPRECVDATA supports up to 2048 bytes at a time
    if (esp_wifi_inst.ProactRecv.MaxSize > 2048) {
        esp_wifi_inst.ProactRecv.MaxSize = 2048;
    }

    sprintf((char *)esp_wifi_inst.CmdData, "AT+CIPRECVDATA=%d,%d\r\n", pxConn->LinkID, esp_wifi_inst.ProactRecv.MaxSize);
    xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, xTimeout);

    return xRet;
}

ESP_WIFI_Status_t ESP_WIFI_Recv( uint8_t LinkID, uint8_t * pcBuf, 
                                 uint16_t usReqLen, uint16_t * usRecvLen )
{
    return ESP_WIFI_Recv2(LinkID, pcBuf, usReqLen, usRecvLen, ESP_TIMEOUT_RECV);
}

ESP_WIFI_Status_t ESP_WIFI_Recv2( uint8_t LinkID, uint8_t * pcBuf, 
                                 uint16_t usReqLen, uint16_t * usRecvLen, uint32_t xTimeout )
{
    ESP_WIFI_Status_t xRet = ESP_WIFI_STATUS_ERROR;
    uint16_t usDataRecvd = 0;
    uint64_t xTickCurrent, xTickEnd;
    uint32_t xTickTimeout;
    ESP_WIFI_Conn_t *pxConn;

    ESP_ASSERT(LinkID < ESP_CFG_SOCKET_COUNT);
    pxConn = &esp_wifi_inst.Conns[LinkID];

    /* If proactive receive, check its buffer */
    if (ESP_WIFI_IsProactRecv(LinkID)) {
        /* Control flow with buffer/size
         *
         * Non-null buffer/Non-zero size:   Go "AT+CIPRECVDATA"/"CIPRECVDATA" protocol
         * Null buffer/Any size:            Go "+IPD" to get notified of incoming data
         */
        if (pcBuf && !usReqLen) {
            ESP_LOG_WARN("Proactive receive requires valid buffer/size or passed in\r\n");
            return xRet;
        }
    } else {
        if (pcBuf) {
            ESP_LOG_WARN("Reactive receive requires no receive buffer passed in\r\n");
            return xRet;
        }
    }

    xTickEnd = ESP_Plat_GetTime_MS() + xTimeout;

    if (usRecvLen) {
        *usRecvLen = 0;
    };

    if (ESP_WIFI_IsProactRecv(LinkID)) {
        /* Proactive receive */

        do {
            uint32_t OobBm = AT_OOB_BM_IPD | AT_OOB_BM_LINKCLOSED;

            xTickCurrent = ESP_Plat_GetTime_MS();
            if (xTickCurrent < xTickEnd) {
                xTickTimeout = xTickEnd - xTickCurrent;
            } else {
                xRet = ESP_WIFI_STATUS_TIMEOUT;
                break;
            }

            /* Set up proactive receive */
            esp_wifi_inst.ProactRecv.Data = pcBuf ? (pcBuf + usDataRecvd) : NULL;
            esp_wifi_inst.ProactRecv.MaxSize = pcBuf ? (usReqLen - usDataRecvd) : 0;
            esp_wifi_inst.ProactRecv.Size = 0;

            /* Check "+IPD" to avoid repetitive, trivial "AT+CIPRECVDATA" */
            if (pxConn->ProactRecv.NewIpdDataAvbl && pcBuf) {
                /* ESP modem has buffered received data. We can still
                 * receive it even though the link has closed (before it
                 * is reopened). */
                xRet = ESP_WIFI_Recv_TCP_PASSIVE(LinkID, xTickTimeout);
            } else if (!pxConn->IsConnected) {
                /* Remote peer has closed the link and ESP modem hasn't buffered received data */
                ESP_LOG_INFO("Exit proactive receive procedure with link %d closed\r\n", LinkID);
                xRet = ESP_WIFI_STATUS_LINKCLOSED;
                break;
            }
            else {
                xRet = ESP_IO_Recv(esp_wifi_inst.CmdData, sizeof(esp_wifi_inst.CmdData), &OobBm, xTickTimeout);
            }

            if (xRet == ESP_WIFI_STATUS_OK) {
                /* (Trivial) OK */
                usDataRecvd += esp_wifi_inst.ProactRecv.Size;
                if (pxConn->ProactRecv.NewIpdDataAvbl > esp_wifi_inst.ProactRecv.Size) {
                    pxConn->ProactRecv.NewIpdDataAvbl -= esp_wifi_inst.ProactRecv.Size;
                } else {
                    pxConn->ProactRecv.NewIpdDataAvbl = 0;
                }
            } else if (xRet == ESP_WIFI_STATUS_ERROR) {
                /* Error */
                ESP_LOG_WARN("Proactive receive(+CIPRECVDATA)...ERROR\r\n");
                break;
            } else if (xRet == ESP_WIFI_STATUS_TIMEOUT) {
                /* Timeout */
                break;
            } else if (xRet == ESP_WIFI_STATUS_OOB) {
                if (OobBm & AT_OOB_BM_IPD) {
                    /* Do nothing */
                } else if (OobBm & AT_OOB_BM_LINKCLOSED) {
                    uint8_t ClosedLinkID = (uint8_t) ((OobBm & 0xFF0000) >> 16);
                    if (LinkID == ClosedLinkID) {
                        ESP_LOG_INFO("Link %d has closed\r\n", ClosedLinkID);
                    }
                }
            }

            /* FIXME: Exit condition
             *
             * 1. Timeout
             * 2. Receive buffer is full or has any data?
             */
        //} while ((usReqLen && (usDataRecvd < usReqLen)) || (!usReqLen && usDataRecvd));
        } while ((pcBuf && !usDataRecvd) || (!pcBuf && !pxConn->ProactRecv.NewIpdDataAvbl));

        /* Reset proactive receive for safe */
        esp_wifi_inst.ProactRecv.Data = NULL;
        esp_wifi_inst.ProactRecv.MaxSize = 0;
        esp_wifi_inst.ProactRecv.Size = 0;
    } else {
        /* Reactive receive */

        do {
            uint32_t OobBm = AT_OOB_BM_IPD | AT_OOB_BM_LINKCLOSED;

            xTickCurrent = ESP_Plat_GetTime_MS();
            if (xTickCurrent < xTickEnd) {
                xTickTimeout = xTickEnd - xTickCurrent;
            } else {
                xRet = ESP_WIFI_STATUS_TIMEOUT;
                break;
            }

            /* Remote peer has closed the link */
            if (!pxConn->IsConnected) {
                ESP_LOG_INFO("Exit reactive receive procedure with link %d closed\r\n", LinkID);
                xRet = ESP_WIFI_STATUS_LINKCLOSED;
                break;
            }

            pxConn->ReactRecv.NewIpdDataRecvd = 0;

            xRet = ESP_IO_Recv(esp_wifi_inst.CmdData, sizeof(esp_wifi_inst.CmdData), &OobBm, xTickTimeout);
            if (xRet == ESP_WIFI_STATUS_ERROR) {
                /* Error */
                ESP_LOG_WARN("Reactive receive(+IPD)...ERROR\r\n");
                break;
            } else if (xRet == ESP_WIFI_STATUS_TIMEOUT) {
                /* Timeout */
                break;
            } else if (xRet == ESP_WIFI_STATUS_OOB) {
                if (OobBm & AT_OOB_BM_IPD) {
                    usDataRecvd += pxConn->ReactRecv.NewIpdDataRecvd;
                } else if (OobBm & AT_OOB_BM_LINKCLOSED) {
                    uint8_t ClosedLinkID = (uint8_t) ((OobBm & 0xFF0000) >> 16);
                    if (LinkID == ClosedLinkID) {
                        ESP_LOG_INFO("Link %d has closed\r\n", ClosedLinkID);
                    }
                }
            }

            /* FIXME: Exit condition
             *
             * 1. Timeout
             * 2. Receive buffer is full or has any data?
             */
        //} while ((usReqLen && (usDataRecvd < usReqLen)) || (!usReqLen && usDataRecvd));
        } while (!usDataRecvd);
    }

    /* Regard it as OK as long as there has been data successfully
     * transfered and there is no error */
    if (xRet != ESP_WIFI_STATUS_ERROR && usDataRecvd) {
        xRet = ESP_WIFI_STATUS_OK;
    }

    if (usRecvLen) {
        *usRecvLen = usDataRecvd;
    };

    return xRet;
}

bool ESP_WIFI_IsProactRecv( uint8_t LinkID )
{
    ESP_WIFI_Conn_t *pxConn;

    ESP_ASSERT(LinkID < ESP_CFG_SOCKET_COUNT);
    pxConn = &esp_wifi_inst.Conns[LinkID];

    return pxConn->IsPassiveMode;
}

static ESP_WIFI_Status_t ESP_WIFI_SoftReset( void )
{
    ESP_WIFI_Status_t xRet;

    /* S/W reset */
    ESP_LOG_DBG("S/W reset...\r\n");
    xRet = ESP_AT_Command((uint8_t *)"AT+RST\r\n", 1000, ESP_TIMEOUT_NORMAL);
    if (xRet == ESP_WIFI_STATUS_OK) {
        /* Wait for "ready" OOB */
        uint32_t oob_bm = AT_OOB_BM_READY;
        xRet = ESP_IO_Recv(esp_wifi_inst.CmdData, sizeof(esp_wifi_inst.CmdData), &oob_bm, ESP_TIMEOUT_READY);
        if (xRet == ESP_WIFI_STATUS_OOB && (oob_bm & AT_OOB_BM_READY)) {
            ESP_LOG_DBG("S/W reset...SUCCESS\r\n");
            xRet = ESP_WIFI_STATUS_OK;
        } else {
            ESP_LOG_WARN("S/W reset...FAILURE(%d)\r\n", xRet);
        }
    } else {
        ESP_LOG_WARN("S/W reset...FAILURE(%d)\r\n", xRet);
    }

    return xRet;
}

bool ESP_WIFI_IsConnected( void )
{
    return esp_wifi_inst.IsConnected;
}

static ESP_WIFI_Status_t ESP_WIFI_EnableEcho( bool enable )
{
    ESP_WIFI_Status_t xRet;

    ESP_LOG_INFO("%s echo...\r\n", enable ? "Enable" : "Disable");
    sprintf((char *)esp_wifi_inst.CmdData, "ATE%d\r\n", enable ? 1 : 0);

    xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, ESP_TIMEOUT_NORMAL);
    if (xRet == ESP_WIFI_STATUS_OK) {
        ESP_LOG_DBG("%s echo...SUCCESS\r\n", enable ? "Enable" : "Disable");
    } else {
        ESP_LOG_CRIT("%s echo...FAILURE\r\n", enable ? "Enable" : "Disable");
    }

    return xRet;
}

#if ESP_CFG_MULTICONN
static ESP_WIFI_Status_t ESP_WIFI_EnableMultiConn( bool enable )
{
    ESP_WIFI_Status_t xRet;

    ESP_LOG_DBG("%s multiple-connection mode...\r\n", enable ? "Enable" : "Disable");

    sprintf((char *)esp_wifi_inst.CmdData, "AT+CIPMUX=%d\r\n", enable ? 1 : 0);

    xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, ESP_TIMEOUT_NORMAL);
    if (xRet == ESP_WIFI_STATUS_OK) {
        esp_wifi_inst.IsMultiConnEnabled = enable;
        ESP_LOG_DBG("%s multiple-connection mode...SUCCESS\r\n", enable ? "Enable" : "Disable");
    } else {
        ESP_LOG_CRIT("%s multiple-connection mode... ERROR\r\n", enable ? "Enable" : "Disable");
    }

    return xRet;
}
#endif

#if ESP_CFG_TCP_PASSIVE_RECV_MODE
static ESP_WIFI_Status_t ESP_WIFI_EnableTcpPassive( bool enable )
{
    ESP_WIFI_Status_t xRet;

    ESP_LOG_DBG("%s TCP passive receive mode...\r\n", enable ? "Enable" : "Disable");

    sprintf((char *)esp_wifi_inst.CmdData, "AT+CIPRECVMODE=%d\r\n", enable ? 1 : 0);

    xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, ESP_TIMEOUT_NORMAL);
    if (xRet == ESP_WIFI_STATUS_OK) {
        esp_wifi_inst.IsPassiveModeEnabled = enable;
        ESP_LOG_DBG("%s TCP passive receive mode...SUCCESS\r\n", enable ? "Enable" : "Disable");
    } else {
        ESP_LOG_CRIT("%s TCP passive receive mode...FAILURE\r\n", enable ? "Enable" : "Disable");
    }

    return xRet;
}
#endif

#if ESP_CFG_SERIAL_FC
static ESP_WIFI_Status_t ESP_WIFI_EnableSerialFlowCtrl( bool enable )
{
    ESP_WIFI_Status_t xRet;

    ESP_LOG_DBG("%s H/W flow control with baudrate=%d...\r\n", enable ? "Enable" : "Disable", ESP_CFG_SERIAL_BAUDRATE);

    sprintf((char *)esp_wifi_inst.CmdData, "AT+UART_CUR=%d,8,1,0,%d\r\n", ESP_CFG_SERIAL_BAUDRATE, enable ? 3 : 0);

    xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, ESP_TIMEOUT_NORMAL);
    if (xRet == ESP_WIFI_STATUS_OK) {
        ESP_LL_EnableFlowControl(enable);
        ESP_LOG_DBG("%s H/W flow control...SUCCESS\r\n", enable ? "Enable" : "Disable");
    } else {
        ESP_LOG_CRIT("%s H/W flow control...FAILURE\r\n", enable ? "Enable" : "Disable");
    }

    return xRet;
}
#endif

static ESP_WIFI_Status_t ESP_WIFI_CheckDnsServer( void )
{
    ESP_WIFI_Status_t xRet;
    char *pcDelim = ":\r\n";
    char *pcPtr;

    sprintf((char *)esp_wifi_inst.CmdData, "AT+CIPDNS_CUR?\r\n");
    xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, ESP_TIMEOUT_NORMAL);

    if (xRet == ESP_WIFI_STATUS_OK) {
        uint16_t dns_num = 0;

        pcPtr = strtok((char *)esp_wifi_inst.CmdData, pcDelim);
        while (pcPtr != NULL) {
            if (strcmp(pcPtr, "+CIPDNS_CUR") == 0) {
                pcPtr = strtok(NULL, pcDelim);
                if (strcmp(pcPtr, "255.255.255.255") != 0) {
                    ESP_LOG_INFO("Found DNS(%d): %s\r\n", dns_num, pcPtr);
                    dns_num ++;
                }
            }
            pcPtr = strtok(NULL, pcDelim);
        }

        /* To check is there a DNS server */
        if (!dns_num) {
            ESP_LOG_INFO("There is no DNS server. Set one to be resolver1.opendns.com(208.67.222.222).\r\n");
            sprintf((char *)esp_wifi_inst.CmdData, "AT+CIPDNS_CUR=1,\"208.67.222.222\"\r\n");
            xRet = ESP_AT_Command(esp_wifi_inst.CmdData, 0, ESP_TIMEOUT_NORMAL);
        }
    }

    return xRet;
}

static uint16_t ESP_IO_Send( uint8_t pucTxBuf[], uint16_t usWriteBytes )
{
    uint16_t i;
    
    for (i = 0; i < usWriteBytes; i ++) {
        while (!ESP_LL_TxStrm_Writable());
        ESP_LL_TxStrm_PutChar(pucTxBuf[i]);
    }
    
    return i;
}

static ESP_WIFI_Status_t ESP_IO_Recv( uint8_t pucRxBuf[], uint16_t usReadBytes, uint32_t *ulOobBitmap,
                               uint32_t xTimeout )
{
    ESP_WIFI_Status_t xRet = ESP_WIFI_STATUS_TIMEOUT;
    uint64_t xTickEnd;
    uint8_t ucRecvHeader[64];
    char *pcDelim = ",:\r\n";
    char *pcPtr;
    char *pcRecv;
    uint8_t ucExit = 0;
    uint16_t usCount;
    uint16_t usNum;
    uint16_t usIpdLength;

    /* Set timeout in ms */
    xTickEnd = ESP_Plat_GetTime_MS() + xTimeout;

    /* Clear the receive buffer */
    pcPtr = (char *)(pucRxBuf);
    memset(pcPtr, 0x0, usReadBytes);

    for (usCount = 0; (usCount < usReadBytes) && !ucExit; usCount++) {
        /* Check Rx empty => failed */
        while (!ESP_LL_RxStrm_Readable()) {
            if (ESP_Plat_GetTime_MS() > xTickEnd) {
                /* If ESP8266 already receives data, need to wait for data send out complete */
                if ((strstr((char *)pucRxBuf, "\r\nRecv") > 0) && (strstr((char *)pucRxBuf, "bytes\r\n") > 0)) {
                    xTickEnd += ESP_TIMEOUT_RECV_EXTRA;
                } else {
                    ESP_LOG_DBG("Read %d bytes...TIMEOUT(%d ms)\r\n", usReadBytes, xTimeout);
                    ucExit = 1;
                    break;
                }
            }
        }
        if (ucExit == 1) {
            break;
        }

        /* Get data from Rx buffer */
        pucRxBuf[usCount] = ESP_LL_RxStrm_GetChar();
        if (xTickEnd < (ESP_Plat_GetTime_MS() + 5)) {
            xTickEnd = ESP_Plat_GetTime_MS() + 10;
        }

        /* Check the end of the message to reduce the response time */
        switch (pucRxBuf[usCount]) {
        case '\n':
            if (strncmp(pcPtr, AT_RESP_OK, sizeof(AT_RESP_OK) - 1) == 0) {
                ucExit = 1;
                xRet = ESP_WIFI_STATUS_OK;
            } else if ((strncmp(pcPtr, AT_RESP_FAIL, sizeof(AT_RESP_FAIL) - 1)) == 0 || 
                       (strncmp(pcPtr, AT_RESP_ERROR, sizeof(AT_RESP_ERROR) - 1)) == 0) {
                ucExit = 1;
                xRet = ESP_WIFI_STATUS_ERROR;
            } else if (strncmp(pcPtr, AT_OOB_CONNECT, sizeof(AT_OOB_CONNECT) - 1) == 0 ||
                       strncmp(pcPtr + 1, "," AT_OOB_CONNECT, sizeof(AT_OOB_CONNECT)) == 0) {
                uint8_t OpenLinkID = 0;

                ESP_STATIC_ASSERT(sizeof(OpenLinkID) == sizeof(uint8_t), "Must be uint8_t type to match scanf %hhd specifier");
                if (esp_wifi_inst.IsMultiConnEnabled) {
                    sscanf((const char *) pcPtr, "%hhd,CONNECT", &OpenLinkID);
                } else {
                    OpenLinkID = 0;
                }

                if (OpenLinkID < ESP_CFG_SOCKET_COUNT) {
                    esp_wifi_inst.Conns[OpenLinkID].IsConnected = true;
                } else {
                    ESP_LOG_CRIT("Invalid \"%d,CONNECT\". Should be 0-%d\r\n", OpenLinkID, ESP_CFG_SOCKET_COUNT - 1);
                }

                /* Advance to next line */
                pcPtr = (char *) pucRxBuf + usCount + 1;
            } else if (strstr(pcPtr, AT_OOB_CLOSED)) {
                uint8_t ClosedLinkID = 0;

                ESP_STATIC_ASSERT(sizeof(ClosedLinkID) == sizeof(uint8_t), "Must be uint8_t type to match scanf %hhd specifier");
                if (esp_wifi_inst.IsMultiConnEnabled) {
                    sscanf((const char *) pcPtr, "%hhd,CLOSED", &ClosedLinkID);
                } else {
                    ClosedLinkID = 0;
                }
                
                if (ClosedLinkID < ESP_CFG_SOCKET_COUNT) {
                    esp_wifi_inst.Conns[ClosedLinkID].IsConnected = false;
                } else {
                    ESP_LOG_CRIT("Invalid \"%d,CLOSED\". Should be 0-%d\r\n", ClosedLinkID, ESP_CFG_SOCKET_COUNT - 1);
                }

                if (ulOobBitmap && (*ulOobBitmap & AT_OOB_BM_LINKCLOSED)) {
                    /* Premature exit with matched OOB */
                    ucExit = 1;
                    xRet = ESP_WIFI_STATUS_OOB;
                    *ulOobBitmap = (ClosedLinkID << 16) | AT_OOB_BM_LINKCLOSED;
                } else {
                    /* Advance to next line */
                    pcPtr = (char *) pucRxBuf + usCount + 1;
                }

                /* Advance to next line */
                pcPtr = (char *) pucRxBuf + usCount + 1;
            } else if (strstr(pcPtr, AT_OOB_READY)) {
                ESP_LOG_INFO(AT_OOB_READY "\r\n");
                if (ulOobBitmap && (*ulOobBitmap & AT_OOB_BM_READY)) {
                    /* Premature exit with matched OOB */
                    ucExit = 1;
                    xRet = ESP_WIFI_STATUS_OOB;
                    *ulOobBitmap = AT_OOB_BM_READY;
                } else {
                    /* Advance to next line */
                    pcPtr = (char *) pucRxBuf + usCount + 1;
                }
            } else if (strstr(pcPtr, AT_OOB_WIFI_CONNECTED)) {
                ESP_LOG_INFO(AT_OOB_WIFI_CONNECTED "\r\n");

                /* Advance to next line */
                pcPtr = (char *) pucRxBuf + usCount + 1;
            } else if (strstr(pcPtr, AT_OOB_WIFI_GOTIP)) {
                ESP_LOG_INFO(AT_OOB_WIFI_GOTIP "\r\n");

                /* Advance to next line */
                pcPtr = (char *) pucRxBuf + usCount + 1;
            } else if (strstr(pcPtr, AT_OOB_WIFI_DISCONNECT)) {
                ESP_LOG_INFO(AT_OOB_WIFI_DISCONNECT "\r\n");
                esp_wifi_inst.IsConnected = false;

                /* Advance to next line */
                pcPtr = (char *) pucRxBuf + usCount + 1;
            } else if (strstr(pcPtr, AT_OOB_BUSY_SEND)) {
                ESP_LOG_DBG("Busy sending...\r\n");

                ucExit = 1;
                xRet = ESP_WIFI_STATUS_BUSY;
            } else if (strstr(pcPtr, AT_OOB_BUSY_PROCESS)) {
                ESP_LOG_DBG("Busy processing...\r\n");

                ucExit = 1;
                xRet = ESP_WIFI_STATUS_BUSY;
            } else if (strstr(pcPtr, AT_OOB_SENDOK)) {
                if (ulOobBitmap && (*ulOobBitmap & AT_OOB_BM_SENDOK)) {
                    /* Premature exit with matched OOB */
                    ucExit = 1;
                    xRet = ESP_WIFI_STATUS_OOB;
                    *ulOobBitmap = AT_OOB_BM_SENDOK;
                } else {
                    /* Advance to next line */
                    pcPtr = (char *) pucRxBuf + usCount + 1;
                }
            } else if (strstr(pcPtr, AT_OOB_SENDFAIL)) {
                if (ulOobBitmap && (*ulOobBitmap & AT_OOB_BM_SENDFAIL)) {
                    /* Premature exit with matched OOB */
                    ucExit = 1;
                    xRet = ESP_WIFI_STATUS_OOB;
                    *ulOobBitmap = AT_OOB_BM_SENDFAIL;
                } else {
                    /* Advance to next line */
                    pcPtr = (char *) pucRxBuf + usCount + 1;
                }
            } else if ((strstr(pcPtr, AT_OOB_IPD)) && esp_wifi_inst.IsPassiveModeEnabled) {
                uint8_t ucIpdLinkID = 0;

                pcRecv = strstr(pcPtr, AT_OOB_IPD);
                /* Clear the receive buffer */
                memcpy(ucRecvHeader, pcRecv, sizeof(ucRecvHeader));
                pcRecv = (char *)(ucRecvHeader);

                pcPtr = strstr(pcRecv, AT_OOB_IPD);
                pcPtr = strtok(pcPtr, pcDelim);
                /* Check multiple connection */
                if (esp_wifi_inst.IsMultiConnEnabled) {
                    /* Get the Link ID */
                    pcPtr = strtok(NULL, pcDelim);
                    ucIpdLinkID = (uint8_t)atoi(pcPtr);
                }
                pcPtr = strtok(NULL, pcDelim);
                usIpdLength = (uint16_t)atoi(pcPtr);
                
                ESP_LOG_DBG("+IPD for TCP passive receive mode: socket ID(%d), data size(%d)\r\n", ucIpdLinkID, usIpdLength);

                /* Inform that data is ready in proactive receive link through receive callback */   
                if (esp_wifi_inst.ReactRecv.DataRecvCb.Callback) {
                    esp_wifi_inst.ReactRecv.DataRecvCb.Callback(esp_wifi_inst.ReactRecv.DataRecvCb.Ctx, ucIpdLinkID, NULL, usIpdLength);
                }

                if (ulOobBitmap && (*ulOobBitmap & AT_OOB_BM_IPD)) {
                    /* Premature exit with matched OOB */
                    ucExit = 1;
                    xRet = ESP_WIFI_STATUS_OOB;
                    *ulOobBitmap = AT_OOB_BM_IPD;
                    /* Flag the active link has received new IPD data */
                    esp_wifi_inst.Conns[ucIpdLinkID].ProactRecv.NewIpdDataAvbl = usIpdLength;
                } else {
                    /* Advance to next line */
                    pcPtr = (char *) pucRxBuf + usCount + 1;
                }
            } else {
                /* Skip this line and advance to next one */
                pcPtr = (char *) pucRxBuf + usCount + 1;
            }
            break;

        case '>':
            if (strstr(pcPtr, AT_RESP_SEND) == pcPtr) {
                ucExit = 1;
                xRet = ESP_WIFI_STATUS_SEND;
            }
            break;

        case ':':
            /* Check if +CIPRECVDATA */
            pcRecv = strstr(pcPtr, AT_RESP_CIPRECVDATA);
            if (pcRecv) {
                /* Go +CIPRECVDATA */
                /* Clear the receive buffer */
                memcpy(ucRecvHeader, pcRecv, sizeof(ucRecvHeader));
                pcRecv = (char *)(ucRecvHeader);

                /* Get the IPD length */
                pcPtr = strstr(pcRecv, AT_RESP_CIPRECVDATA);
                pcPtr = strtok(pcPtr, pcDelim);
                pcPtr = strtok(NULL, pcDelim);
                usIpdLength = (uint16_t)atoi(pcPtr);

                /* Check proactive receive struct */
                ESP_ASSERT(esp_wifi_inst.ProactRecv.Data);
                ESP_ASSERT(esp_wifi_inst.ProactRecv.MaxSize >= usIpdLength);
                ESP_ASSERT(!esp_wifi_inst.ProactRecv.Size);

                for (usNum = 0; usNum < usIpdLength; usNum++) {
                    while (!ESP_LL_RxStrm_Readable()) {
                        if (ESP_Plat_GetTime_MS() > xTickEnd) {
                            ESP_LOG_CRIT("+CIPRECVDATA...TIMEOUT(%d ms)\r\n", xTimeout);
                            ucExit = 1;
                            break;
                        }
                    }
                    if (ucExit == 0) {
                        esp_wifi_inst.ProactRecv.Data[esp_wifi_inst.ProactRecv.Size ++] = ESP_LL_RxStrm_GetChar();
                        if (xTickEnd < (ESP_Plat_GetTime_MS() + 5)) {
                            xTickEnd = ESP_Plat_GetTime_MS() + 10;
                        }
                    } else {
                        break;
                    }
                }

                /* Advance to next line */
                pcPtr = (char *) pucRxBuf + usCount + 1;
            } else {
                /* Check if +IPD */
                pcRecv = strstr(pcPtr, AT_OOB_IPD);
                if (pcRecv) {
                    uint8_t ucIpdLinkID = 0;

                    /* Go +IPD */
                    /* Clear the receive buffer */
                    memcpy(ucRecvHeader, pcRecv, sizeof(ucRecvHeader));
                    pcRecv = (char *)(ucRecvHeader);

                    pcPtr = strstr(pcRecv, AT_OOB_IPD);
                    pcPtr = strtok(pcPtr, pcDelim);
                    /* Check multiple connection */
                    if (esp_wifi_inst.IsMultiConnEnabled) {
                        /* Get the Link ID */
                        pcPtr = strtok(NULL, pcDelim);
                        ucIpdLinkID = (uint8_t)atoi(pcPtr);
                    }

                    /* FIXME: Change to dynamic check rather than assert */
                    ESP_ASSERT(ucIpdLinkID < ESP_CFG_SOCKET_COUNT);

                    /* Get the IPD length */
                    pcPtr = strtok(NULL, pcDelim);
                    usIpdLength = (uint16_t)atoi(pcPtr);
                    if (usIpdLength > sizeof(esp_wifi_inst.ReactRecv.IpdData.Buffer)) {
                        ESP_LOG_CRIT("Max IPD size Expected %d, Got %d\r\n", sizeof(esp_wifi_inst.ReactRecv.IpdData.Buffer), usIpdLength);
                        ucExit = 1;
                        xRet = ESP_WIFI_STATUS_ERROR;
                    }

                    /* Set up for reactive receive */
                    esp_wifi_inst.ReactRecv.IpdData.Size = 0;

                    for (usNum = 0; usNum < usIpdLength; usNum++) {
                        while (!ESP_LL_RxStrm_Readable()) {
                            if (ESP_Plat_GetTime_MS() > xTickEnd) {
                                ESP_LOG_CRIT("IPD data...TIMEOUT(%d ms)\r\n", xTimeout);
                                ucExit = 1;
                                break;
                            }
                        }
                        if (ucExit == 0) {
                            esp_wifi_inst.ReactRecv.IpdData.Buffer[esp_wifi_inst.ReactRecv.IpdData.Size ++] = ESP_LL_RxStrm_GetChar();
                            if (xTickEnd < (ESP_Plat_GetTime_MS() + 5)) {
                                xTickEnd = ESP_Plat_GetTime_MS() + 10;
                            }
                        } else {
                            break;
                        }
                    }

                    ESP_LOG_DBG("IPD data: Expected %d bytes, Got %d bytes, Reactive %d bytes\r\n", usIpdLength, usNum, esp_wifi_inst.ReactRecv.IpdData.Size);

                    /* Add into reactive receive callback */   
                    if (esp_wifi_inst.ReactRecv.IpdData.Size) {
                        if (esp_wifi_inst.ReactRecv.DataRecvCb.Callback) {
                            esp_wifi_inst.ReactRecv.DataRecvCb.Callback(esp_wifi_inst.ReactRecv.DataRecvCb.Ctx, ucIpdLinkID, esp_wifi_inst.ReactRecv.IpdData.Buffer, esp_wifi_inst.ReactRecv.IpdData.Size);
                        } else {
                            ESP_LOG_WARN("IPD data: Lost due to no receive callback installed\r\n");
                        }
                    }

                    if (ulOobBitmap && (*ulOobBitmap & AT_OOB_BM_IPD)) {
                        /* Premature exit with matched OOB */
                        ucExit = 1;
                        xRet = ESP_WIFI_STATUS_OOB;
                        *ulOobBitmap = AT_OOB_BM_IPD;
                        /* Flag the active link has received new IPD data */
                        esp_wifi_inst.Conns[ucIpdLinkID].ReactRecv.NewIpdDataRecvd = usNum;
                    } else {
                        /* Advance to next line */
                        pcPtr = (char *) pucRxBuf + usCount + 1;
                    }
                }
            }
            break;

        default:
            break;
        }
    }
    if (usCount < 200) {
        if (usCount > 0) {
            ESP_LOG_DBG("[AT<]%s%s", pucRxBuf, pucRxBuf[usCount - 1] == '\n' ? "" : "\r\n");
        } else {
            ESP_LOG_DBG("[AT<]None\r\n");
        }
    } else {
        ESP_LOG_DBG("[AT<]...%d chars...\r\n", usCount);
    }

    return xRet;
}

static ESP_WIFI_Status_t ESP_AT_Command( uint8_t * pucCmd, uint32_t ulATCmdCoolDownTime, uint32_t ulATCmdTimeout )
{
    ESP_WIFI_Status_t xRet = ESP_WIFI_STATUS_ERROR;
    uint64_t xTickCurrent;
    uint32_t xTickTimeout;

    xTickTimeout = ulATCmdTimeout;

    /* Wait for previous AT command to cool down even though it has finished (Undocumented?) */
    {
        xTickCurrent = ESP_Plat_GetTime_MS();
        if (esp_wifi_inst.PrevATCmdCoolDownTime > xTickCurrent) {
            ESP_Plat_Wait_MS(esp_wifi_inst.PrevATCmdCoolDownTime - xTickCurrent);
        }
    }

    ESP_LOG_DBG("[AT>]%s", pucCmd);
    if (ESP_IO_Send(pucCmd, strlen((char *)pucCmd)) > 0) {
        xRet = ESP_IO_Recv(esp_wifi_inst.CmdData, sizeof(esp_wifi_inst.CmdData), NULL, xTickTimeout);
        if (xRet == ESP_WIFI_STATUS_OK ||
            xRet == ESP_WIFI_STATUS_ERROR ||
            xRet == ESP_WIFI_STATUS_TIMEOUT) {
            if (ulATCmdCoolDownTime > 0) {
                /* Set the time that next AT command is workable */
                esp_wifi_inst.PrevATCmdCoolDownTime = ESP_Plat_GetTime_MS() + ulATCmdCoolDownTime;
            }
        } else if (xRet == ESP_WIFI_STATUS_BUSY) {
            ESP_LOG_WARN("AT command canceled with busy\r\n");
        }
    }

    return xRet;
}

static void ParseIpAddr( char * string, uint8_t * addr )
{
    char *pcDelim = "\",.";
    char *pcPtr;
    uint8_t ucCount;

    pcPtr = strtok(string, pcDelim);

    for (ucCount = 0; ucCount < 4; ucCount++) {
        addr[ucCount] = atoi(pcPtr);
        pcPtr = strtok(NULL, pcDelim);
    }
}

static void ParseMacAddr( char * string, uint8_t * addr )
{
    char *pcDelim = "\",:";
    char *pcPtr;
    uint8_t ucCount;

    pcPtr = strtok(string, pcDelim);

    for (ucCount = 0; ucCount < 6; ucCount++) {
        addr[ucCount] = strtol(pcPtr, NULL, 16);
        pcPtr = strtok(NULL, pcDelim);
    }
}

static void ParseApDetail( char * string, ESP_WIFI_ScanResult_t *xBuffer )
{
    char *pcDelim = ",";
    char *pcPtr;
    char *pcSave;
    char *pcSSID;
    uint8_t ucNum = 1;
    uint8_t ucMode;

    pcPtr = strtok_r(string, pcDelim, &pcSave);

    while (pcPtr != NULL) {
        switch (ucNum++) {
        case 1:
            ucMode = (uint8_t)atoi(pcPtr);
            if (ucMode == 4) {
                /* Does not support WPA_WPA2, set it as WPA2 mode */
                ucMode = 3;
            } else if (ucMode > 4) {
                /* Set as WiFiSecurityNotSupported */
                ucMode = 4;
            }
            xBuffer->xSecurity = (ESP_WIFI_Security_t)ucMode;
            break;

        case 2:
            pcSSID = strtok(pcPtr, "\"");
            if (pcSSID) {
                strncpy(&xBuffer->cSSID[0], pcSSID, ESP_CFG_SSID_SIZE);
                xBuffer->cSSID[ESP_CFG_SSID_SIZE] = '\0';
            }
            break;

        case 3:
            xBuffer->cRSSI = (int8_t)atoi(pcPtr);
            break;

        case 4:
            ParseMacAddr(pcPtr, &xBuffer->ucBSSID[0]);
            break;

        case 5:
            xBuffer->cChannel = (int8_t)atoi(pcPtr);
            break;

        default:
            break;
        }
        pcPtr = strtok_r(NULL, pcDelim, &pcSave);
    }
}

static void AT_ParseAddress( void )
{
    char *pcDelim = ",\r\n";
    char *pcPtr;
    char *pcSave;

    pcPtr = strtok_r((char *)esp_wifi_inst.CmdData, pcDelim, &pcSave);

    while (pcPtr != NULL){
        if (strcmp(pcPtr, "+CIFSR:STAIP") == 0) {
            pcPtr = strtok_r(NULL, pcDelim, &pcSave);
            ParseIpAddr(pcPtr, esp_wifi_inst.StaIpAddr);
        } else if (strcmp(pcPtr, "+CIFSR:STAMAC") == 0) {
            pcPtr = strtok_r(NULL, pcDelim, &pcSave);
            ParseMacAddr(pcPtr, esp_wifi_inst.StaMacAddr);
        } else if (strcmp(pcPtr, "+CIFSR:APIP") == 0) {
            pcPtr = strtok_r(NULL, pcDelim, &pcSave);
            ParseIpAddr(pcPtr, esp_wifi_inst.ApIpAddr);
        } else if (strcmp(pcPtr, "+CIFSR:APMAC") == 0) {
            pcPtr = strtok_r(NULL, pcDelim, &pcSave);
            ParseMacAddr(pcPtr, esp_wifi_inst.ApMacAddr);
        }
        pcPtr = strtok_r(NULL, pcDelim, &pcSave);
    }
}

static void AT_ParseAccessPoint( ESP_WIFI_ScanResult_t * pxBuffer, uint8_t ucNumNetworks )
{
    char *pcDelim = "()\r\n";
    char *pcPtr;
    char *pcSave;
    uint8_t ucCount;

    pcPtr = strtok_r((char *)esp_wifi_inst.CmdData, pcDelim, &pcSave);

    for (ucCount = 0; ucCount < ucNumNetworks; ) {
        if (pcPtr != NULL){
            if (strcmp(pcPtr, "+CWLAP:") == 0) {
                pcPtr = strtok_r(NULL, pcDelim, &pcSave);
                ParseApDetail(pcPtr, (ESP_WIFI_ScanResult_t *)(&pxBuffer[ucCount]));
                ucCount++;
            }
            pcPtr = strtok_r(NULL, pcDelim, &pcSave);
        } else
            break;
    }
}

static void At_ParseFirmwareVersion( ESP_WIFI_FW_AT_Version_t *at_version, ESP_WIFI_FW_SDK_Version_t *sdk_version )
{
    char *tok_line;
    char *tok_ctx;
    
    /* First line */
    tok_line = strtok_r((char *)esp_wifi_inst.CmdData, "\r\n", &tok_ctx);

    /* Parse every line */
    while (tok_line != NULL) {        
        if (strstr(tok_line, "AT version")) {
            uint16_t unused;

            ESP_STATIC_ASSERT(sizeof(at_version->major) == sizeof(uint16_t), "Must be uint16_t type to match scanf %hd specifier");
            ESP_STATIC_ASSERT(sizeof(unused) == sizeof(uint16_t), "Must be uint16_t type to match scanf %hd specifier");
            sscanf(tok_line, "AT version:%hd.%hd.%hd.%hd", &at_version->major, &at_version->minor, &at_version->patch, &unused);
        } else if (strstr(tok_line, "SDK version")) {
            ESP_STATIC_ASSERT(sizeof(sdk_version->major) == sizeof(uint16_t), "Must be uint16_t type to match scanf %hd specifier");
            sscanf(tok_line, "SDK version:%hd.%hd.%hd", &sdk_version->major, &sdk_version->minor, &sdk_version->patch);
        }

        /* Next line */
        tok_line = strtok_r(NULL, "\r\n", &tok_ctx);
    }
}
