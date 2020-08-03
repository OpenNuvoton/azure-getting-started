/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demo Azure RTOS ThreadX for M480 MCU.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/* ESP8266 includes */
#include "esp8266_wifi.h"
#include "netx_esp8266_network.h"

/* Azure RTOS includes */
#include "tx_api.h"
#include "nx_api.h"

/* User includes */
#include "myapp_config.h"
#include "myapp_utils.h"

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


#define THREAD_0_STACK_SIZE         4096

/* Define the ThreadX object control blocks...  */

TX_THREAD               thread_0;


/* Define thread prototypes.  */

void    thread_0_entry(ULONG thread_input);

UCHAR thread_0_stack[THREAD_0_STACK_SIZE];

/* Define what the initial system looks like.  */

void    tx_application_define(void *first_unused_memory)
{
    UINT status = NX_SUCCESS;

    NX_PARAMETER_NOT_USED(first_unused_memory);

    /* Create the main thread.  */
    MYAPP_CHK_STATUS(tx_thread_create(&thread_0, "thread 0", thread_0_entry, 0,  
                                      thread_0_stack, THREAD_0_STACK_SIZE, 
                                      4, 4, TX_NO_TIME_SLICE, TX_AUTO_START));

    /* Success return */
    return;

clean_up:

    /* Failure return */
    return;
}

/* Define user threads.  */

/* HTTP server */
#define HTTP_SERVER_NAME    "www.ifconfig.io"
#define HTTP_SERVER_PORT    80
ULONG host_ip_address;

ULONG timeout_ticks = 1000;
const char http_request[] =     "GET /method HTTP/1.1\r\n"
                                "Host: ifconfig.io\r\n"
                                "Connection: close\r\n"
                                "\r\n";
char http_response[1024];

NX_TCP_SOCKET           tcp_client_socket;
bool                    tcp_client_socket_created = false;
NX_PACKET *             packet_0 = NULL;

void    thread_0_entry(ULONG thread_input)
{
    UINT status = NX_SUCCESS;

    NX_PARAMETER_NOT_USED(thread_input);

    MYAPP_CHK_STATUS(netx_esp8266_network_initialize(WIFI_SSID, WIFI_PASSWORD, WIFI_MODE));

    /* Enable TCP */
    MYAPP_CHK_STATUS(nx_tcp_enable(&nx_ip));

    /* Enable UDP */
    MYAPP_CHK_STATUS(nx_udp_enable(&nx_ip));

    /* Get HTTP server IP address through DNS */
    MYAPP_CHK_STATUS(nx_dns_host_by_name_get(&nx_dns_client, (UCHAR *)HTTP_SERVER_NAME, &host_ip_address, timeout_ticks));
    printf(HTTP_SERVER_NAME " IP address: %lu.%lu.%lu.%lu\r\n",
           (host_ip_address >> 24) & 0xFF,
           (host_ip_address >> 16) & 0xFF,                   
           (host_ip_address >> 8) & 0xFF,
           host_ip_address & 0xFF);

    /* Create a socket */
    MYAPP_CHK_STATUS(nx_tcp_socket_create(&nx_ip, &tcp_client_socket, "TCP Client Socket",
                                          NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 200,
                                          NX_NULL, NX_NULL));
    tcp_client_socket_created = true;

    /* Bind the socket */
    MYAPP_CHK_STATUS(nx_tcp_client_socket_bind(&tcp_client_socket, NX_ANY_PORT, timeout_ticks));

    /* Attempt to connect the socket */
    MYAPP_CHK_STATUS(nx_tcp_client_socket_connect(&tcp_client_socket, host_ip_address, HTTP_SERVER_PORT, timeout_ticks));

    /* Print HTTP request */
    printf("HTTP request:\r\n");
    /* Print data */
    printf(http_request);

    /* Allocate a packet  */
    packet_0 = NULL;
    MYAPP_CHK_STATUS(nx_packet_allocate(&nx_pool, &packet_0, NX_TCP_PACKET, timeout_ticks));
    MYAPP_CHK_BOOL(packet_0);

    /* Write HTTP request into the packet payload */
    MYAPP_CHK_STATUS(nx_packet_data_append(packet_0, (void *) http_request, sizeof(http_request) - 1, &nx_pool, timeout_ticks));

    /* Send HTTP request packet */
    MYAPP_CHK_STATUS(nx_tcp_socket_send(&tcp_client_socket, packet_0, timeout_ticks));
    packet_0 = NULL;    // Has transferred its ownership

    while (1) {
        /* Receive HTTP response into packet */
        MYAPP_CHK_BOOL(!packet_0);
        packet_0 = NULL;
        status = nx_tcp_socket_receive(&tcp_client_socket, &packet_0, timeout_ticks);
        if (status == NX_SUCCESS) {
            MYAPP_CHK_BOOL(packet_0);
        } else if (status == NX_NO_PACKET) {
            MYAPP_CHK_BOOL(!packet_0);
            continue;
        } else if (status == NX_NOT_CONNECTED) {
            MYAPP_CHK_BOOL(!packet_0);
            printf("Closed by remote\r\n");
            break;
        } else {
            MYAPP_CHK_BOOL(!packet_0);
            /* Unexpected error */
            MYAPP_CHK_STATUS(status);
        }

        if (packet_0) {
            ULONG packet_length, bytes_copied;

            /* Retrieve data from packet */
            MYAPP_CHK_STATUS(nx_packet_length_get(packet_0, &packet_length));
            MYAPP_CHK_BOOL((packet_length + 1) <= sizeof(http_response));
            MYAPP_CHK_STATUS(nx_packet_data_retrieve(packet_0, http_response, &bytes_copied));
            MYAPP_CHK_BOOL(packet_length == bytes_copied);

            /* Print HTTP response */
            printf("HTTP response:\r\n");
            /* Print data */
            printf(http_response);

            /* Release packet_0 */
            MYAPP_CHK_STATUS(nx_packet_release(packet_0));
            packet_0 = NULL;
        }
    }

    /* Disconnect this socket */
    MYAPP_CHK_STATUS(nx_tcp_socket_disconnect(&tcp_client_socket, timeout_ticks));

    /* Unbind the socket */
    MYAPP_CHK_STATUS(nx_tcp_client_socket_unbind(&tcp_client_socket));

clean_up:

    /* NOTE: No MYAPP_CHK_XXX(...) here */

    if (tcp_client_socket_created) {
        /* Delete the socket */
        nx_tcp_socket_delete(&tcp_client_socket);
        tcp_client_socket_created = false;
    }

    if (packet_0) {
        nx_packet_release(packet_0);
        packet_0 = NULL;
    }

    /* Hold here */
    while (1) {
        tx_thread_sleep(1000);
        //tx_thread_relinquish();
    }
}
