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

TX_SEMAPHORE            semaphore_0;
NX_TCP_SOCKET           tcp_client_socket;
NX_UDP_SOCKET           udp_client_socket;

void    tx_application_define(void *first_unused_memory)
{
    UINT status = NX_SUCCESS;

    NX_PARAMETER_NOT_USED(first_unused_memory);

    /* Create semaphore_0 */
    MYAPP_CHK_BOOL(tx_semaphore_create(&semaphore_0, "semaphore_0", 0) == TX_SUCCESS);

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

/* Define the test threads.  */

uint8_t remote_ip[4] = {192, 168, 8, 101};
uint16_t remote_port = 21;
uint16_t local_port = 20;
ULONG timeout_ticks = 1000;
const char send_data_0[] = "abcdefghijk\r\n";
const char send_data_1[] = "0123456789\r\n";
NX_PACKET *packet_0 = NULL;
NX_PACKET *packet_1 = NULL;

static VOID tcp_receive_notify(NX_TCP_SOCKET *socket_ptr);
static VOID udp_receive_notify(NX_UDP_SOCKET *socket_ptr);

void    thread_0_entry(ULONG thread_input)
{
    UINT status = NX_SUCCESS;

    NX_PARAMETER_NOT_USED(thread_input);

    MYAPP_CHK_STATUS(netx_esp8266_network_initialize(WIFI_SSID, WIFI_PASSWORD, WIFI_MODE));

    /* Enable TCP */
    MYAPP_CHK_STATUS(nx_tcp_enable(&nx_ip));
    
    /* Enable UDP */
    MYAPP_CHK_STATUS(nx_udp_enable(&nx_ip));

    /* TCP open/close */
    if (0) {
        uint16_t rounds = 8;

        /* Create a socket */
        MYAPP_CHK_STATUS(nx_tcp_socket_create(&nx_ip, &tcp_client_socket, "TCP Client Socket",
                                              NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 200,
                                              NX_NULL, NX_NULL));
                             
        /* Bind the socket */
        MYAPP_CHK_STATUS(nx_tcp_client_socket_bind(&tcp_client_socket, local_port, timeout_ticks));
        
        while (rounds --) {
            /* Attempt to connect the socket */
            MYAPP_CHK_STATUS(nx_tcp_client_socket_connect(&tcp_client_socket, IP_ADDRESS(remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3]), remote_port, timeout_ticks));

            /* Disconnect this socket */
            MYAPP_CHK_STATUS(nx_tcp_socket_disconnect(&tcp_client_socket, timeout_ticks));
        }

        /* Unbind the socket */
        MYAPP_CHK_STATUS(nx_tcp_client_socket_unbind(&tcp_client_socket));

        /* Delete the socket */
        MYAPP_CHK_STATUS(nx_tcp_socket_delete(&tcp_client_socket));
    }

    /* TCP open/send/close */
    if (0) {
        /* Create a socket */
        MYAPP_CHK_STATUS(nx_tcp_socket_create(&nx_ip, &tcp_client_socket, "TCP Client Socket",
                                              NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 200,
                                              NX_NULL, NX_NULL));
                             
        /* Bind the socket */
        MYAPP_CHK_STATUS(nx_tcp_client_socket_bind(&tcp_client_socket, local_port, timeout_ticks));
        
        
        /* Attempt to connect the socket */
        MYAPP_CHK_STATUS(nx_tcp_client_socket_connect(&tcp_client_socket, IP_ADDRESS(remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3]), remote_port, timeout_ticks));

        while (1) {
            /* Allocate packet_0  */
            packet_0 = NULL;
            MYAPP_CHK_STATUS(nx_packet_allocate(&nx_pool, &packet_0, NX_TCP_PACKET, timeout_ticks));
            MYAPP_CHK_BOOL(packet_0);

            /* Write send_data_0 into the packet payload */
            MYAPP_CHK_STATUS(nx_packet_data_append(packet_0, (void *) send_data_0, sizeof(send_data_0) - 1, &nx_pool, timeout_ticks));

            /* Send packet_0 */
            status = nx_tcp_socket_send(&tcp_client_socket, packet_0, timeout_ticks);
            if (status == NX_SUCCESS) {
                /* Do nothing */
            } else if (status == NX_NOT_CONNECTED) {
                /* Release packet_0 */
                MYAPP_CHK_STATUS(nx_packet_release(packet_0));
                break;
            } else {
                /* Release packet_0 */
                MYAPP_CHK_STATUS(nx_packet_release(packet_0));

                /* Unexpected error */
                MYAPP_CHK_STATUS(status);
            }

            /* Allocate packet_1  */
            packet_1 = NULL;
            MYAPP_CHK_STATUS(nx_packet_allocate(&nx_pool, &packet_1, NX_TCP_PACKET, timeout_ticks));
            MYAPP_CHK_BOOL(packet_1);

            /* Write send_data_1 into the packet payload */
            MYAPP_CHK_STATUS(nx_packet_data_append(packet_1, (void *) send_data_1, sizeof(send_data_1) - 1, &nx_pool, timeout_ticks));

            /* Send packet_1 */
            status = nx_tcp_socket_send(&tcp_client_socket, packet_1, timeout_ticks);
            if (status == NX_SUCCESS) {
                /* Do nothing */
            } else if (status == NX_NOT_CONNECTED) {
                /* Release packet_1 */
                MYAPP_CHK_STATUS(nx_packet_release(packet_1));
                break;
            } else {
                /* Release packet_1 */
                MYAPP_CHK_STATUS(nx_packet_release(packet_1));

                /* Unexpected error */
                MYAPP_CHK_STATUS(status);
            }

            /* Wait for 5s */
            tx_thread_sleep(500);
        }

        /* Disconnect this socket */
        MYAPP_CHK_STATUS(nx_tcp_socket_disconnect(&tcp_client_socket, timeout_ticks));

        /* Unbind the socket */
        MYAPP_CHK_STATUS(nx_tcp_client_socket_unbind(&tcp_client_socket));

        /* Delete the socket */
        MYAPP_CHK_STATUS(nx_tcp_socket_delete(&tcp_client_socket));
    }

    /* TCP open/proactive receive/close */
    if (0) {
        /* Create a socket */
        MYAPP_CHK_STATUS(nx_tcp_socket_create(&nx_ip, &tcp_client_socket, "TCP Client Socket",
                                              NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 200,
                                              NX_NULL, NX_NULL));
                             
        /* Bind the socket */
        MYAPP_CHK_STATUS(nx_tcp_client_socket_bind(&tcp_client_socket, local_port, timeout_ticks));
        
        
        /* Attempt to connect the socket */
        MYAPP_CHK_STATUS(nx_tcp_client_socket_connect(&tcp_client_socket, IP_ADDRESS(remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3]), remote_port, timeout_ticks));

        while (1) {
            /* Receive into packet_0 */
            packet_0 = NULL;
            status = nx_tcp_socket_receive(&tcp_client_socket, &packet_0, timeout_ticks);
            if (status == NX_SUCCESS) {
                MYAPP_CHK_BOOL(packet_0);
            } else if (status == NX_NO_PACKET) {
                MYAPP_CHK_BOOL(!packet_0);
                continue;
            } else if (status == NX_NOT_CONNECTED) {
                MYAPP_CHK_BOOL(!packet_0);
                break;
            } else {
                MYAPP_CHK_BOOL(!packet_0);
                /* Unexpected error */
                MYAPP_CHK_STATUS(status);
            }

            if (packet_0) {
                /* Dump packet_0 */
                MYAPP_LOG_OUT("Dump packet_0:\r\n");
                MYAPP_CHK_STATUS(myapp_dump_packet(packet_0));

                /* Release packet_0 */
                MYAPP_CHK_STATUS(nx_packet_release(packet_0));
                packet_0 = NULL;
            }
        }

        /* Disconnect this socket */
        MYAPP_CHK_STATUS(nx_tcp_socket_disconnect(&tcp_client_socket, timeout_ticks));

        /* Unbind the socket */
        MYAPP_CHK_STATUS(nx_tcp_client_socket_unbind(&tcp_client_socket));

        /* Delete the socket */
        MYAPP_CHK_STATUS(nx_tcp_socket_delete(&tcp_client_socket));
    }
    
    /* TCP open/reactive receive/close */
    if (0) {
        /* Create a socket */
        MYAPP_CHK_STATUS(nx_tcp_socket_create(&nx_ip, &tcp_client_socket, "TCP Client Socket",
                                              NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 200,
                                              NX_NULL, NX_NULL));
                             
        /* Bind the socket */
        MYAPP_CHK_STATUS(nx_tcp_client_socket_bind(&tcp_client_socket, local_port, timeout_ticks));
        
        
        /* Attempt to connect the socket */
        MYAPP_CHK_STATUS(nx_tcp_client_socket_connect(&tcp_client_socket, IP_ADDRESS(remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3]), remote_port, timeout_ticks));

        /* Reactive receive socket data into packet_1 */
        MYAPP_CHK_STATUS(nx_tcp_socket_receive_notify(&tcp_client_socket, tcp_receive_notify));

        while (1) {      
            /* Get semaphore_0 with suspension */
            //MYAPP_CHK_BOOL(tx_semaphore_get(&semaphore_0, TX_WAIT_FOREVER) == TX_SUCCESS);
            tx_thread_sleep(1000);
        }

        /* Disconnect this socket */
        MYAPP_CHK_STATUS(nx_tcp_socket_disconnect(&tcp_client_socket, timeout_ticks));

        /* Unbind the socket */
        MYAPP_CHK_STATUS(nx_tcp_client_socket_unbind(&tcp_client_socket));

        /* Delete the socket */
        MYAPP_CHK_STATUS(nx_tcp_socket_delete(&tcp_client_socket));
    }

    /* UDP open/send/close */
    if (0) {
        /* Create a socket */
        MYAPP_CHK_STATUS(nx_udp_socket_create(&nx_ip, &udp_client_socket, "UDP Client Socket",
                                              NX_IP_NORMAL, NX_FRAGMENT_OKAY, 0x80, 5));
                             
        /* Bind the socket */
        MYAPP_CHK_STATUS(nx_udp_socket_bind(&udp_client_socket, local_port, timeout_ticks));
        
        while (1) {
            /* Allocate packet_0  */
            packet_0 = NULL;
            MYAPP_CHK_STATUS(nx_packet_allocate(&nx_pool, &packet_0, NX_UDP_PACKET, timeout_ticks));
            MYAPP_CHK_BOOL(packet_0);

            /* Write send_data_0 into the packet payload */
            MYAPP_CHK_STATUS(nx_packet_data_append(packet_0, (void *) send_data_0, sizeof(send_data_0) - 1, &nx_pool, timeout_ticks));

            /* Send packet_0 */
            status = nx_udp_socket_send(&udp_client_socket, packet_0, IP_ADDRESS(remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3]), remote_port);
            if (status == NX_SUCCESS) {
                /* Do nothing */
            } else if (status == NX_NOT_CONNECTED) {
                /* Release packet_0 */
                MYAPP_CHK_STATUS(nx_packet_release(packet_0));
                break;
            } else {
                /* Release packet_0 */
                MYAPP_CHK_STATUS(nx_packet_release(packet_0));

                /* Unexpected error */
                MYAPP_CHK_STATUS(status);
            }

            /* Wait for 5s */
            tx_thread_sleep(500);

            /* Allocate packet_1  */
            packet_1 = NULL;
            MYAPP_CHK_STATUS(nx_packet_allocate(&nx_pool, &packet_1, NX_UDP_PACKET, timeout_ticks));
            MYAPP_CHK_BOOL(packet_1);

            /* Write send_data_1 into the packet payload */
            MYAPP_CHK_STATUS(nx_packet_data_append(packet_1, (void *) send_data_1, sizeof(send_data_1) - 1, &nx_pool, timeout_ticks));

            /* Send packet_1 */
            status = nx_udp_socket_send(&udp_client_socket, packet_1, IP_ADDRESS(remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3]), remote_port);
            if (status == NX_SUCCESS) {
                /* Do nothing */
            } else if (status == NX_NOT_CONNECTED) {
                /* Release packet_1 */
                MYAPP_CHK_STATUS(nx_packet_release(packet_1));
                break;
            } else {
                /* Release packet_1 */
                MYAPP_CHK_STATUS(nx_packet_release(packet_1));

                /* Unexpected error */
                MYAPP_CHK_STATUS(status);
            }

            /* Wait for 5s */
            tx_thread_sleep(500);
        }

        /* Unbind the socket */
        MYAPP_CHK_STATUS(nx_udp_socket_unbind(&udp_client_socket));

        /* Delete the socket */
        MYAPP_CHK_STATUS(nx_udp_socket_delete(&udp_client_socket));
    }

    /* UDP open/proactive receive/close */
    if (0) {
        /* Create a socket */
        MYAPP_CHK_STATUS(nx_udp_socket_create(&nx_ip, &udp_client_socket, "UDP Client Socket",
                                              NX_IP_NORMAL, NX_FRAGMENT_OKAY, 0x80, 5));
                             
        /* Bind the socket */
        MYAPP_CHK_STATUS(nx_udp_socket_bind(&udp_client_socket, local_port, timeout_ticks));
        
        while (1) {
            /* Receive into packet_0 */
            packet_0 = NULL;
            status = nx_udp_socket_receive(&udp_client_socket, &packet_0, timeout_ticks);
            if (status == NX_SUCCESS) {
                MYAPP_CHK_BOOL(packet_0);
            } else if (status == NX_NO_PACKET) {
                MYAPP_CHK_BOOL(!packet_0);
                continue;
            } else if (status == NX_NOT_CONNECTED) {
                MYAPP_CHK_BOOL(!packet_0);
                break;
            } else {
                MYAPP_CHK_BOOL(!packet_0);
                /* Unexpected error */
                MYAPP_CHK_STATUS(status);
            }

            if (packet_0) {
                /* Dump packet_0 */
                MYAPP_LOG_OUT("Dump packet_0:\r\n");
                MYAPP_CHK_STATUS(myapp_dump_packet(packet_0));

                /* Release packet_0 */
                MYAPP_CHK_STATUS(nx_packet_release(packet_0));
                packet_0 = NULL;
            }
        }

        /* Unbind the socket */
        MYAPP_CHK_STATUS(nx_udp_socket_unbind(&udp_client_socket));

        /* Delete the socket */
        MYAPP_CHK_STATUS(nx_udp_socket_delete(&udp_client_socket));
    }

    /* UDP open/reactive receive/close */
    if (1) {
        /* Create a socket */
        MYAPP_CHK_STATUS(nx_udp_socket_create(&nx_ip, &udp_client_socket, "UDP Client Socket",
                                              NX_IP_NORMAL, NX_FRAGMENT_OKAY, 0x80, 5));
                             
        /* Bind the socket */
        MYAPP_CHK_STATUS(nx_udp_socket_bind(&udp_client_socket, local_port, timeout_ticks));
        
        /* Reactive receive socket data into packet_1 */
        MYAPP_CHK_STATUS(nx_udp_socket_receive_notify(&udp_client_socket, udp_receive_notify));

        while (1) {      
            /* Get semaphore_0 with suspension */
            //MYAPP_CHK_BOOL(tx_semaphore_get(&semaphore_0, TX_WAIT_FOREVER) == TX_SUCCESS);
            tx_thread_sleep(1000);
        }

        /* Unbind the socket */
        MYAPP_CHK_STATUS(nx_udp_socket_unbind(&udp_client_socket));

        /* Delete the socket */
        MYAPP_CHK_STATUS(nx_udp_socket_delete(&udp_client_socket));
    }

    while (1) {
        tx_thread_sleep(1000);
        //tx_thread_relinquish();
    }

clean_up:

    while (1) {
        tx_thread_sleep(1000);
        //tx_thread_relinquish();
    }
}

static VOID tcp_receive_notify(NX_TCP_SOCKET *socket_ptr)
{
    UINT status = NX_SUCCESS;

    /* Receive into packet_1 */
    packet_1 = NULL;
    MYAPP_CHK_STATUS(nx_tcp_socket_receive(socket_ptr, &packet_1, timeout_ticks));
    MYAPP_CHK_BOOL(packet_1);

    /* Dump packet_1 */
    MYAPP_LOG_OUT("Dump packet_1:\r\n");
    MYAPP_CHK_STATUS(myapp_dump_packet(packet_1));

    /* Release packet_1 */
    MYAPP_CHK_STATUS(nx_packet_release(packet_1));
    packet_1 = NULL;

    /* Release semaphore_0 */
    //MYAPP_CHK_BOOL(tx_semaphore_put(&semaphore_0) == TX_SUCCESS);
    
clean_up:

    return ;
}

static VOID udp_receive_notify(NX_UDP_SOCKET *socket_ptr)
{
    UINT status = NX_SUCCESS;

    /* Receive into packet_1 */
    packet_1 = NULL;
    MYAPP_CHK_STATUS(nx_udp_socket_receive(socket_ptr, &packet_1, timeout_ticks));
    MYAPP_CHK_BOOL(packet_1);

    /* Dump packet_1 */
    MYAPP_LOG_OUT("Dump packet_1:\r\n");
    MYAPP_CHK_STATUS(myapp_dump_packet(packet_1));

    /* Release packet_1 */
    MYAPP_CHK_STATUS(nx_packet_release(packet_1));
    packet_1 = NULL;

    /* Release semaphore_0 */
    //MYAPP_CHK_BOOL(tx_semaphore_put(&semaphore_0) == TX_SUCCESS);
    
clean_up:

    return ;
}
