/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demo Azure RTOS ThreadX for M480 MCU.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "tx_api.h"

/* ESP8266 include */
#include "esp8266_wifi.h"

#define PLL_CLOCK           192000000

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();



    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    /* Lock protected registers */
    SYS_LockReg();
}

/* Port of Azure RTOS ThreadX demo code onto Nuvoton platform
 *
 * https://github.com/azure-rtos/threadx/blob/master/samples/demo_threadx.c
 *
 * 1. Enable stdio output console
 * 2. Add/refine sleep cycles and output messages in threads
 */
int main()
{
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("Demo Azure RTOS ThreadX...\n");

    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}

#define DEMO_STACK_SIZE         2048
#define DEMO_BYTE_POOL_SIZE     9120
#define DEMO_BLOCK_POOL_SIZE    100
#define DEMO_QUEUE_SIZE         100


/* Define the ThreadX object control blocks...  */

TX_THREAD               thread_0;
TX_QUEUE                queue_0;
TX_SEMAPHORE            semaphore_0;
TX_MUTEX                mutex_0;
TX_EVENT_FLAGS_GROUP    event_flags_0;
TX_BYTE_POOL            byte_pool_0;
TX_BLOCK_POOL           block_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];


/* Define the counters used in the demo application...  */

ULONG                   thread_0_counter;

/* Define thread prototypes.  */

void    thread_0_entry(ULONG thread_input);


/* Define what the initial system looks like.  */

void    tx_application_define(void *first_unused_memory)
{

CHAR    *pointer = TX_NULL;


    /* Create a byte memory pool from which to allocate the thread stacks.  */
    tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);

    /* Put system definition stuff in here, e.g. thread creates and other assorted
       create information.  */

    /* Allocate the stack for thread 0.  */
    tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);

    /* Create the main thread.  */
    tx_thread_create(&thread_0, "thread 0", thread_0_entry, 0,  
            pointer, DEMO_STACK_SIZE, 
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);

}

static void my_esp_datarecv_handler(void *Ctx, uint8_t LinkID, uint8_t *Data, uint16_t Size);
static void my_dump_hex(uint8_t *data, uint16_t size);

typedef struct {
    uint32_t dummy;     // Avoid empty struct
} MY_ESP_DATARECV_CTX;

MY_ESP_DATARECV_CTX my_esp_datarecv_ctx;

/* Define the test threads.  */

void    thread_0_entry(ULONG thread_input)
{
    ESP_LOG_CRIT("ESP critical message\r\n");
    ESP_LOG_WARN("ESP warning message\r\n");
    ESP_LOG_INFO("ESP informational message\r\n");
    ESP_LOG_DBG("ESP debug message\r\n");
    ESP_LOG_V("ESP verbose message\r\n");

    ESP_ASSERT(ESP_WIFI_Init() == ESP_WIFI_STATUS_OK);
    
    ESP_WIFI_FW_AT_Version_t at_version;
    ESP_WIFI_FW_SDK_Version_t sdk_version;
    ESP_ASSERT(ESP_WIFI_GetFirmwareVersion(&at_version, &sdk_version) == ESP_WIFI_STATUS_OK);

    ESP_ASSERT(ESP_WIFI_SetWiFiMode(ESP_WIFI_STATION) == ESP_WIFI_STATUS_OK);

    ESP_ASSERT(ESP_WIFI_EnableDHCP(ESP_WIFI_STATION, true) == ESP_WIFI_STATUS_OK);
    
    const char *SSID = "HUAWEI-F2C9";
    const char *Password = "td06309i";
    ESP_ASSERT(ESP_WIFI_Connect(SSID, Password) == ESP_WIFI_STATUS_OK);

    const char *host_name = "iot.espressif.cn";
    uint8_t host_ip[4];
    ESP_ASSERT(ESP_WIFI_GetHostIP(host_name, host_ip) == ESP_WIFI_STATUS_OK);
    ESP_LOG_INFO("%s(%d.%d.%d.%d)\r\n", host_name, host_ip[0], host_ip[1], host_ip[2], host_ip[3]);
    
    uint8_t ApIpAddr[4];
    uint8_t ApMacAddr[6];
    uint8_t StaIpAddr[4];
    uint8_t StaMacAddr[6];
    ESP_ASSERT(ESP_WIFI_GetNetStatus(ApIpAddr, ApMacAddr, StaIpAddr, StaMacAddr) == ESP_WIFI_STATUS_OK);
    ESP_LOG_INFO("SoftAP IP: %d.%d.%d.%d\r\n", ApIpAddr[0], ApIpAddr[1], ApIpAddr[2], ApIpAddr[3]);
    ESP_LOG_INFO("SoftAP MAC: %2x.%2x.%2x.%2x.%2x.%2x\r\n", ApMacAddr[0], ApMacAddr[1], ApMacAddr[2], ApMacAddr[3], ApMacAddr[4], ApMacAddr[5]);
    ESP_LOG_INFO("Station IP: %d.%d.%d.%d\r\n", StaIpAddr[0], StaIpAddr[1], StaIpAddr[2], StaIpAddr[3]);
    ESP_LOG_INFO("Station MAC: %2x.%2x.%2x.%2x.%2x.%2x\r\n", StaMacAddr[0], StaMacAddr[1], StaMacAddr[2], StaMacAddr[3], StaMacAddr[4], StaMacAddr[5]);

    uint8_t RemoteIP[4] = {192, 168, 8, 112};
    uint16_t RemotePort = 21;
    uint16_t UdpLocalPort = 21;
    uint8_t LinkID = 2;

    /* Install data receive callback */
    ESP_ASSERT(ESP_WIFI_InstallDataRecvCallback(my_esp_datarecv_handler, &my_esp_datarecv_ctx) == ESP_WIFI_STATUS_OK);

    /* TCP open/close */
    if (0) {
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_TCP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_TCP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_TCP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_TCP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_TCP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_TCP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_TCP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_TCP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);

    }

    /* TCP open/send/close */
    if (0) {
        ESP_WIFI_Status_t xRet;
        const char send_data1[] = "abcdefghijk\r\n";
        const char send_data2[] = "0123456789\r\n";
        uint16_t sent = 0;
        uint32_t timeout = 60000;
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_TCP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);

        while (1) {
            xRet = ESP_WIFI_Send2(LinkID, (uint8_t *) send_data1, sizeof(send_data1) - 1, &sent, timeout);
            if (xRet == ESP_WIFI_STATUS_OK) {
                ESP_ASSERT(sent == sizeof(send_data1) - 1);
            } else if (xRet == ESP_WIFI_STATUS_LINKCLOSED) {
                ESP_ASSERT(sent == 0);
                break;
            } else {
                ESP_ASSERT(false);
            }

            xRet = ESP_WIFI_Send2(LinkID, (uint8_t *) send_data2, sizeof(send_data2) - 1, &sent, timeout);
            if (xRet == ESP_WIFI_STATUS_OK) {
                ESP_ASSERT(sent == sizeof(send_data2) - 1);
            } else if (xRet == ESP_WIFI_STATUS_LINKCLOSED) {
                ESP_ASSERT(sent == 0);
                break;
            } else if (xRet == ESP_WIFI_STATUS_TIMEOUT) {
                ESP_ASSERT(sent == 0);
            } else {
                ESP_ASSERT(false);
            }

            ESP_Plat_Wait_MS(5000);
        }

        ESP_ASSERT(ESP_WIFI_CloseClientConnection(LinkID) == ESP_WIFI_STATUS_OK);
    }

    /* TCP open/receive/close */
    if (0) {
        ESP_WIFI_Status_t xRet;
        char recv_buffer1[3 + 1];
        uint16_t recvd = 0;
        uint32_t timeout = 60000;
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_TCP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);

        while (1) {
            memset(recv_buffer1, 0x00, sizeof(recv_buffer1));
            xRet = ESP_WIFI_Recv2(LinkID, (uint8_t *) recv_buffer1, sizeof(recv_buffer1) - 1, &recvd, timeout);
            if (xRet == ESP_WIFI_STATUS_OK) {
                ESP_LOG_INFO("recvd=%d\r\n", recvd);
                my_dump_hex((uint8_t *) recv_buffer1, recvd);
            } else if (xRet == ESP_WIFI_STATUS_LINKCLOSED) {
                ESP_ASSERT(recvd == 0);
                break;
            } else if (xRet == ESP_WIFI_STATUS_TIMEOUT) {
                ESP_ASSERT(recvd == 0);
            } else {
                ESP_ASSERT(false);
            }
        }

        ESP_ASSERT(ESP_WIFI_CloseClientConnection(LinkID) == ESP_WIFI_STATUS_OK);
    }

    /* UDP open/close */
    if (0) {
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_UDP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_UDP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_UDP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_UDP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_UDP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_UDP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_UDP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_UDP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);
    }

    /* UDP open/send/close */
    if (0) {
        ESP_WIFI_Status_t xRet;
        const char send_data1[] = "abcdefghijk\r\n";
        const char send_data2[] = "0123456789\r\n";
        uint16_t sent = 0;
        uint32_t timeout = 60000;
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_UDP, RemoteIP, RemotePort, 0) == ESP_WIFI_STATUS_OK);

        while (1) {
            xRet = ESP_WIFI_Send2(LinkID, (uint8_t *) send_data1, sizeof(send_data1) - 1, &sent, timeout);
            if (xRet == ESP_WIFI_STATUS_OK) {
                ESP_ASSERT(sent == sizeof(send_data1) - 1);
            } else if (xRet == ESP_WIFI_STATUS_LINKCLOSED) {
                ESP_ASSERT(sent == 0);
                break;
            } else {
                ESP_ASSERT(false);
            }

             ESP_Plat_Wait_MS(5000);

            xRet = ESP_WIFI_Send2(LinkID, (uint8_t *) send_data2, sizeof(send_data2) - 1, &sent, timeout);
            if (xRet == ESP_WIFI_STATUS_OK) {
                ESP_ASSERT(sent == sizeof(send_data2) - 1);
            } else if (xRet == ESP_WIFI_STATUS_LINKCLOSED) {
                ESP_ASSERT(sent == 0);
                break;
            } else if (xRet == ESP_WIFI_STATUS_TIMEOUT) {
                ESP_ASSERT(sent == 0);
            } else {
                ESP_ASSERT(false);
            }

            ESP_Plat_Wait_MS(5000);
        }

        ESP_ASSERT(ESP_WIFI_CloseClientConnection(LinkID) == ESP_WIFI_STATUS_OK);
    }

    /* UDP open/receive/close */
    if (1) {
        ESP_WIFI_Status_t xRet;
        uint16_t recvd = 0;
        uint32_t timeout = 60000;
        ESP_ASSERT(ESP_WIFI_OpenClientConnection(LinkID, ESP_WIFI_UDP, RemoteIP, RemotePort, UdpLocalPort) == ESP_WIFI_STATUS_OK);

        while (1) {
            xRet = ESP_WIFI_Recv2(LinkID, NULL, 0, &recvd, timeout);
            if (xRet == ESP_WIFI_STATUS_OK) {
                ESP_LOG_INFO("recvd=%d\r\n", recvd);
            } else if (xRet == ESP_WIFI_STATUS_LINKCLOSED) {
                ESP_ASSERT(recvd == 0);
                break;
            } else if (xRet == ESP_WIFI_STATUS_TIMEOUT) {
                ESP_ASSERT(recvd == 0);
            } else {
                ESP_ASSERT(false);
            }
        }

        ESP_ASSERT(ESP_WIFI_CloseClientConnection(LinkID) == ESP_WIFI_STATUS_OK);
    }

    ESP_ASSERT(ESP_WIFI_Disconnect() == ESP_WIFI_STATUS_OK);
    
    while (1);
}


static void my_esp_datarecv_handler(void *Ctx, uint8_t LinkID, uint8_t *Data, uint16_t Size)
{
    ESP_LOG_INFO("Ctx(%p), LinkID(%d), Data(%p), Size(%d)\r\n", Ctx, LinkID, Data, Size);
    
    if (Data) {
        my_dump_hex(Data, Size);
    }
}

static void my_dump_hex(uint8_t *data, uint16_t size)
{
    uint8_t *data_ind = data;
    uint16_t rmn = size;
    bool end_newline = false;

    /* Change color for dumped data */
    ESP_CFG_LOG_OUT(ESP_LOGLVLCLR_V);

    ESP_CFG_LOG_OUT("========Dump %d bytes hex data...========\r\n", size);

    while (rmn >= 8) {
        ESP_CFG_LOG_OUT("%02x %02x %02x %02x %02x %02x %02x %02x \r\n", 
                        data_ind[0], data_ind[1], data_ind[2], data_ind[3],
                        data_ind[4], data_ind[5], data_ind[6], data_ind[7]);
        data_ind += 8;
        rmn -= 8;
    }
    
    if (rmn >= 4) {
        ESP_CFG_LOG_OUT("%02x %02x %02x %02x ",
                        data_ind[0], data_ind[1], data_ind[2], data_ind[3]);
        data_ind += 4;
        rmn -= 4;
        end_newline = true;
    }
    
    if (rmn >= 2) {
        ESP_CFG_LOG_OUT("%02x %02x ", data_ind[0], data_ind[1]);
        data_ind += 2;
        rmn -= 2;
        end_newline = true;
    }

    if (rmn) {
        ESP_CFG_LOG_OUT("%02x ", *data_ind ++);
        rmn -= 1;
        end_newline = true;
    }

    if (end_newline) {
        ESP_CFG_LOG_OUT("\r\n");
    }

    ESP_CFG_LOG_OUT("========Dump %d bytes hex data...END========\r\n", size);
    
    /* Change color back */
    ESP_CFG_LOG_OUT(ESP_LOGLVLCLR_NONE);
}
