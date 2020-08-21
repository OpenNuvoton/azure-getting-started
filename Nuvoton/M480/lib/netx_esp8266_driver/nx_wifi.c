/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** NetX Component                                                        */
/**                                                                       */
/**   NetX Porting layer for ESP8266 WiFi                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#include "tx_api.h"
#include "nx_api.h"
/* ESP8266 */
#include "esp8266_wifi.h"
#include "nx_ip.h"
#include "nx_tcp.h"
#include "tx_thread.h"
#include "nx_wifi.h"

#define WIFI_WRITE_TIMEOUT          100
#define WIFI_READ_TIMEOUT           100

#ifndef WIFI_THREAD_PERIOD
#define WIFI_THREAD_PERIOD          100
#endif /* WIFI_THREAD_PERIOD  */

/* Define the default thread priority, stack size, etc. The user can override this 
   via -D command line option or via project settings.  */

/* ESP8266: Enlarge from 1024 to 2048 */
#ifndef NX_WIFI_STACK_SIZE
#define NX_WIFI_STACK_SIZE          (2048)
#endif /* NX_WIFI_STACK_SIZE  */

#ifndef NX_WIFI_THREAD_PRIORITY
#define NX_WIFI_THREAD_PRIORITY     (1)
#endif /* NX_WIFI_THREAD_PRIORITY  */

/* Define the stack for X-WARE WIFI.  */
static UCHAR                        nx_wifi_thread_stack[NX_WIFI_STACK_SIZE];

/* Define the prototypes for X-WARE.  */
static TX_THREAD                    nx_wifi_thread;
static NX_PACKET_POOL               *nx_wifi_pool;
static NX_IP                        *nx_wifi_ip;

/* Define the socket type, TCP socket or UDP socket.  */
#define NX_WIFI_TCP_SOCKET          0
#define NX_WIFI_UDP_SOCKET          1

/* ESP8266: Reserve some packets for applications, such as HTTP, etc.
 * 
 * Enlarge to avoid missing ESP proactive receive packets.
 */
#ifndef NX_WIFI_PACKET_RESERVED
#define NX_WIFI_PACKET_RESERVED     1
#endif /* NX_WIFI_PACKET_RESERVED  */

/* ESP8266: Intermediate receive buffer
 *
 * Disabling this buffer, we can spare ESP_CFG_IPDBUF_SIZE bytes. But we need to control
 * NX_PACKET struct directly.
 * 
 */
#define NX_WIFI_IM_RECV_BUFFER      0

/* Define the WIFI socket structure.  */
typedef struct NX_WIFI_SOCKET_STRUCT
{

    /* Define socket pointer.  */
    VOID       *nx_wifi_socket_ptr;
    
    /* Define socket flag.  */
    CHAR        nx_wifi_socket_valid;
    
    /* Define socket type.  TCP or UDP.  */
    CHAR        nx_wifi_socket_type;
    
    /* Define the connected flag.  */
    CHAR        nx_wifi_socket_connected;

    /* ESP8266: Maintain socket notify count */
#if 0
    /* Reserved.  */
    CHAR        reserved;
#else
    CHAR        nx_wifi_tcp_socket_disconnect_notify;
    CHAR        nx_wifi_socket_receive_notify;
#endif
    
    /* Define the deferred packet processing queue.  */
    NX_PACKET   *nx_wifi_received_packet_head,
                *nx_wifi_received_packet_tail;

}NX_WIFI_SOCKET;

/* ESP8266: Change from 8 to 5 */
#ifndef NX_WIFI_SOCKET_COUNTER
#define NX_WIFI_SOCKET_COUNTER          ESP_CFG_SOCKET_COUNT
#endif /* NX_WIFI_SOCKET_COUNTER  */

/* Define the TCP socket and UDP socket.  */
static NX_WIFI_SOCKET               nx_wifi_socket[NX_WIFI_SOCKET_COUNTER];

/* Define the SOCKET ID.  */
static CHAR                         nx_wifi_socket_counter;

#if NX_WIFI_IM_RECV_BUFFER
/* ESP8266: Define the buffer to receive data from wifi.  */
static CHAR                         nx_wifi_buffer[ESP_CFG_IPDBUF_SIZE];
#endif

/* Define the wifi thread.  */
static void    nx_wifi_thread_entry(ULONG thread_input);

/* ESP8266: Link close callback */
static void nxesp_close_handler(void *Ctx, uint8_t LinkID);

/* ESP8266: Data receive callback */
static void nxesp_datarecv_handler(void *Ctx, uint8_t LinkID, uint8_t *Data, uint16_t Size);

/* ESP8266: Notify tcp socket disconnect */
static void nx_notify_tcp_socket_disconnect(uint8_t entry_index);

/* ESP8266: Notify socket receive */
static void nx_notify_socket_receive(uint8_t entry_index);

/* ESP8266: UDP client connect */
static UINT nx_wifi_udp_client_socket_connect(NX_UDP_SOCKET *socket_ptr,
                                              UCHAR entry_index,
                                              NXD_ADDRESS *server_ip,
                                              UINT server_port);

/* ESP8266: Link close callback context */
typedef struct {
    uint8_t dummy;      // Avoid null struct
} NXESP_CLOSE_CTX;
NXESP_CLOSE_CTX nxesp_close_ctx;

/* ESP8266: Data receive callback context */
typedef struct {
    uint8_t dummy;      // Avoid null struct
} NXESP_DATARECV_CTX;
NXESP_DATARECV_CTX nxesp_datarecv_ctx;

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_initialize                                  PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function initializes the NetX Wifi.                            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    ip_ptr                                Pointer to IP control block   */
/*    packet_pool                           Packet pool pointer           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UINT nx_wifi_initialize(NX_IP *ip_ptr, NX_PACKET_POOL *packet_pool)
{
  
UINT    status;
        
    /* ESP8266 */
    {
        ESP_WIFI_Status_t esp_status;

        /* Should have connected to AP */
        if (!ESP_WIFI_IsConnected()) {
            ESP_LOG_CRIT("ESP WiFi not connected to AP\r\n");
            status = NX_NOT_SUCCESSFUL;
            return NX_NOT_SUCCESSFUL;
        }

        /* Install link close callback */
        memset(&nxesp_close_ctx, 0x00, sizeof(nxesp_close_ctx));
        esp_status = ESP_WIFI_InstallCloseCallback(nxesp_close_handler, &nxesp_close_ctx);
        if (esp_status != ESP_WIFI_STATUS_OK) {
            ESP_LOG_CRIT("Fail to install link close callback (%d)\r\n", esp_status);
            status = NX_NOT_SUCCESSFUL;
            return NX_NOT_SUCCESSFUL;
        }

        /* Install data receive callback */
        memset(&nxesp_datarecv_ctx, 0x00, sizeof(nxesp_datarecv_ctx));
        esp_status = ESP_WIFI_InstallDataRecvCallback(nxesp_datarecv_handler, &nxesp_datarecv_ctx);
        if (esp_status != ESP_WIFI_STATUS_OK) {
            ESP_LOG_CRIT("Fail to install data receive callback (%d)\r\n", esp_status);
            status = NX_NOT_SUCCESSFUL;
            return NX_NOT_SUCCESSFUL;
        }
    }

    /* Set the IP.  */
    nx_wifi_ip = ip_ptr;
    
    /* Set the pool.  */
    nx_wifi_pool = packet_pool;
    
    /* Initialize the wifi.  */
    memset(nx_wifi_socket, 0, (NX_WIFI_SOCKET_COUNTER * sizeof(NX_WIFI_SOCKET)));
    
    /* Initialize the socket id.  */
    nx_wifi_socket_counter = 0;
    
    /* Create the wifi thread.  */
    status = tx_thread_create(&nx_wifi_thread, "Wifi Thread", nx_wifi_thread_entry, 0,  
                              nx_wifi_thread_stack, NX_WIFI_STACK_SIZE, 
                              NX_WIFI_THREAD_PRIORITY, NX_WIFI_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START); 
     
    /* Check for thread create errors.  */
    if (status)
        return(status);
    
    return(NX_SUCCESS);
}
 
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_thread_entry                                PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function is the entry point for NetX Wifi helper thread.  The  */
/*    Wifi helper thread is responsible for receiving packet.             */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    thread_input                          Pointer to IP control block   */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
void nx_wifi_thread_entry(ULONG thread_input)
{
  
/* ESP8266: Move to data receive callback */
//TX_INTERRUPT_SAVE_AREA

/* ESP8266: type */
UINT            i;
UINT            socket_counter;
uint16_t        size;
ESP_WIFI_Status_t   status; 
/* ESP8266: Move to data receive callback */
//NX_PACKET       *packet_ptr;
//NX_TCP_SOCKET   *tcp_socket;
//NX_UDP_SOCKET   *udp_socket;

    while(1)
    {
      
        /* Obtain the IP internal mutex before processing the IP event.  */
        tx_mutex_get(&(nx_wifi_ip -> nx_ip_protection), TX_WAIT_FOREVER);
        
        socket_counter = 0;
        
        /* Check if have socket.  */
        if (nx_wifi_socket_counter != 0)
        {
          
            /* Loop to receive the data from */
            for (i = 0; (i < NX_WIFI_SOCKET_COUNTER) && (socket_counter < nx_wifi_socket_counter); i++)
            {
              
                /* Check if the socket is valid and connected.  */
                if ((nx_wifi_socket[i].nx_wifi_socket_valid == 0) || (nx_wifi_socket[i].nx_wifi_socket_connected == 0))
                    continue;                                           
                
                /* Update the socket counter.  */
                socket_counter++;
                
                /* Loop to receive the data from wifi for current socket.  */
                status = ESP_WIFI_STATUS_OK;
                do
                {

                    /* ESP8266: Make sure there is enough space to store the data before receiving data from WIFI.  */
                    if ((nx_wifi_pool -> nx_packet_pool_available * nx_wifi_pool -> nx_packet_pool_payload_size) < 
                        (ESP_CFG_IPDBUF_SIZE + (NX_WIFI_PACKET_RESERVED * nx_wifi_pool -> nx_packet_pool_payload_size)))
                        break;

                    /* ESP8266: Receive the data in WIFI_READ_TIMEOUT ms.  */
                    if (ESP_WIFI_IsProactRecv(i)) {
                        /* ESP8266: To avoid OOM, we don't proactive receive +CIPRECVDATA data in the background.
                         *          Still, we pass null buffer to go "+IPD" to get notified of incoming data. */
                        status = ESP_WIFI_Recv2(i, NULL, 0, NULL, WIFI_READ_TIMEOUT);
                    } else {
                        /* ESP8266: Reactive receive +IPD data */
                        status = ESP_WIFI_Recv2(i, NULL, 0, NULL, WIFI_READ_TIMEOUT);
                    }

                    /* ESP8266: Notify tcp socket disconnect */
                    nx_notify_tcp_socket_disconnect(i);

                    /* ESP8266: Notify socket receive */
                    nx_notify_socket_receive(i);

                    /* ESP8266: Check status.  */
                    if ((status != ESP_WIFI_STATUS_OK) || (size == 0))
                        break;

                    /* ESP8266: Queue the packet.  */
                }while (status == ESP_WIFI_STATUS_OK);  
            }
        }
        
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        
        /* Sleep some ticks to next loop.  */
        tx_thread_sleep(WIFI_THREAD_PERIOD);
    }
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_tick_convert_ms                             PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function converts the ticks to milliseconds.                   */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    tick                                  Tick value                    */
/*    millisecond                           Destination to millisecond    */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
static VOID  nx_wifi_tick_convert_ms(ULONG tick, ULONG *millisecond)
{
  
UINT    factor = 1000/NX_IP_PERIODIC_RATE;
    

    /* Check the wait_option.  */
    if (tick)
    {
      
        /* Change ticks to milliseconds to ticks.  */
        if (tick >= NX_WAIT_FOREVER/factor)
            *millisecond = NX_WAIT_FOREVER;
        else
            *millisecond = (tick * factor);
    }
    else
    {
        *millisecond = 0;
    }    
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_socket_entry_find                           PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function finds an available entry.                             */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    socket_ptr                            Socket pointer                */
/*    entry_index                           Destination to entry          */
/*    entry_find                            Find flag                     */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
static UINT  nx_wifi_socket_entry_find(void *socket_ptr, UCHAR *entry_index, UCHAR entry_find)
{
    
UINT    i;
UCHAR   empty_index = NX_WIFI_SOCKET_COUNTER;

    /* Loop to find an empty entry.  */
    for (i = 0; i < NX_WIFI_SOCKET_COUNTER; i++)
    {
        
        /* Check the valid flag.  */
        if (nx_wifi_socket[i].nx_wifi_socket_valid)
        {
          
            /* Check if the entry already exist.  */          
            if (nx_wifi_socket[i].nx_wifi_socket_ptr == socket_ptr)
            {
              
                /* Check if find the entry.  */
                if (entry_find)
                {                
                    *entry_index = i;
                    return(NX_SUCCESS);
                }
                else
                {
                    return(NX_NOT_SUCCESSFUL);
                }
            }
        }
        else
        {
            
            /* Set the empty index.  */
            if (empty_index > i)
                empty_index = i;
        }
    }
    
    /* Check if have empty entry.  */
    if (empty_index >= NX_WIFI_SOCKET_COUNTER)
        return (NX_NOT_SUCCESSFUL);
     
    (*entry_index) = empty_index;
    return(NX_SUCCESS);
}



/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_socket_reset                                PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function resets the entry and release the packet.              */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    socket_ptr                            Socket pointer                */
/*    entry_index                           Destination to entry          */
/*    entry_find                            Find flag                     */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
static void  nx_wifi_socket_reset(UCHAR entry_index)
{
     
NX_PACKET *next_packet;
NX_PACKET *current_packet;   

    /* Check if this is an valid entry.  */
    if (nx_wifi_socket[entry_index].nx_wifi_socket_valid == 0)
        return;    

    /* Setup next packet to queue head.  */
    next_packet = nx_wifi_socket[entry_index].nx_wifi_received_packet_head;

    /* Release any packets queued up.  */
    while (next_packet)
    {

        /* Setup the current packet pointer.  */
        current_packet =  next_packet;

        /* Move to the next packet.  */
        next_packet =  next_packet -> nx_packet_queue_next;

        /* Release the current packet.  */
        nx_packet_release(current_packet);
    }
    
    /* Reset the entry.  */
    memset(&nx_wifi_socket[entry_index], 0, sizeof(NX_WIFI_SOCKET));    
    nx_wifi_socket_counter--;
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_socket_receive                              PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function receives packet for wifi socket                       */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    socket_ptr                            Socket pointer                */
/*    packet_ptr                            Pointer to received packet    */
/*    wait_option                           Suspension option             */
/*    socket_type                           Socket type                   */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
static UINT  nx_wifi_socket_receive_shortwait(VOID *socket_ptr, NX_PACKET **packet_ptr, ULONG wait_option, UINT socket_type);
static UINT  nx_wifi_socket_receive(VOID *socket_ptr, NX_PACKET **packet_ptr, ULONG wait_option, UINT socket_type)
{
    UINT    nx_status;
    UINT    start_time, now_time;
    UINT    shortwait_time = WIFI_READ_TIMEOUT * NX_IP_PERIODIC_RATE / 1000;

    /* Get the start time */
    start_time = tx_time_get();

    /* Divide long-wait receive into short-wait receives. This is to avoid 
     * locking ESP8266 modem by one socket and to relinquish to other sockets. */
    do {
        /* Short-wait receive */
        nx_status = nx_wifi_socket_receive_shortwait(socket_ptr, packet_ptr, shortwait_time, socket_type);
        if (nx_status == NX_SUCCESS) {
            /* Exit on receiving at least one packet */
            break;
        } else if (nx_status == NX_NO_PACKET) {
            /* Check timeout on receiving no packet yet */
        } else {
            /* Exit on error */
            break;
        }

        /* Get the current time */
        now_time = tx_time_get();

        /* FIXME: Take tick wrap-around into consideration */
    } while ((wait_option == NX_WAIT_FOREVER) || (wait_option >= (now_time - start_time)));

    return nx_status;
}

static UINT  nx_wifi_socket_receive_shortwait(VOID *socket_ptr, NX_PACKET **packet_ptr, ULONG wait_option, UINT socket_type)
{

TX_INTERRUPT_SAVE_AREA
/* ESP8266: Type */
ESP_WIFI_Status_t   status;
UCHAR   entry_index;
ULONG   total_millisecond;
ULONG   wait_millisecond;
UINT    start_time;
ULONG   millisecond;
uint16_t            size;
UINT    received_packet = NX_FALSE;

    /* Obtain the IP internal mutex before processing the IP event.  */
    tx_mutex_get(&(nx_wifi_ip -> nx_ip_protection), TX_WAIT_FOREVER);     
    
    /* Find an avaiable entry.  */
    if (nx_wifi_socket_entry_find((void *)socket_ptr, &entry_index, 1))
    {
      
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }

    /* Check if the socket is connected.  */
    if (nx_wifi_socket[entry_index].nx_wifi_socket_connected == 0)
    {
      
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    } 
    
    /* Convert the tick to millisecond.  */
    nx_wifi_tick_convert_ms(wait_option, &total_millisecond); 

    /* ESP8266: No data for reactive receive link */
    if (!nx_wifi_socket[entry_index].nx_wifi_received_packet_head &&
        !ESP_WIFI_IsProactRecv(entry_index)) {
        /* Reactive receive +IPD data through installed receive callback.
         * Ignore return code. */
        wait_millisecond = total_millisecond;
        ESP_WIFI_Recv2(entry_index, NULL, 0, NULL, wait_millisecond);
    }

    /* Disable interrupts.  */
    TX_DISABLE
          
    /* Receive the packet from queue.  */
    if (nx_wifi_socket[entry_index].nx_wifi_received_packet_head)
    {
                
        /* Remove the first packet and process it!  */
          
        /* Pickup the first packet.  */
        *packet_ptr =  nx_wifi_socket[entry_index].nx_wifi_received_packet_head;

        /* Move the head pointer to the next packet.  */
        nx_wifi_socket[entry_index].nx_wifi_received_packet_head =  (*packet_ptr) -> nx_packet_queue_next;

        /* Check for end of deferred processing queue.  */
        if (nx_wifi_socket[entry_index].nx_wifi_received_packet_head == NX_NULL)
        {

            /* Yes, the queue is empty.  Set the tail pointer to NULL.  */
            nx_wifi_socket[entry_index].nx_wifi_received_packet_tail =  NX_NULL;
        }

        /* Restore interrupts.  */
        TX_RESTORE      
        
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_SUCCESS);
    }
    else
    {
        
        /* Restore interrupts.  */
        TX_RESTORE            
        
        /* Get the start time.  */
        start_time = tx_time_get();
        
        /* Loop to receive a packet.  */
        while(total_millisecond)
        {
            
            /* ESP8266: No limit on timeout value */
            wait_millisecond = total_millisecond;

            /* ESP8266: Receive the data within a specified time.  */ 
            /* ESP8266: Proactive receive +CIPRECVDATA data */
            if (ESP_WIFI_IsProactRecv(entry_index)) {
#if NX_WIFI_IM_RECV_BUFFER
                status = ESP_WIFI_Recv2(entry_index, (uint8_t*)nx_wifi_buffer, 
                                       sizeof(nx_wifi_buffer), &size, wait_millisecond);
#else
                /* Allocate one packet to store the data. */
                if (nx_packet_allocate(nx_wifi_pool, packet_ptr,  NX_RECEIVE_PACKET, NX_NO_WAIT))
                {
                    ESP_LOG_CRIT("Cannot receive due to failed nx_packet_allocate(...)\r\n");
            
                    status = ESP_WIFI_STATUS_ERROR;
                    break;
                }

                /* Just use single NX_PACKET struct (no packet chain). */
                status = ESP_WIFI_Recv2(entry_index, (uint8_t*) (*packet_ptr)->nx_packet_append_ptr, 
                                       (*packet_ptr)->nx_packet_data_end - (*packet_ptr)->nx_packet_append_ptr, &size, wait_millisecond);
                if (status == ESP_WIFI_STATUS_OK && size) {
                    (*packet_ptr)->nx_packet_append_ptr += size;
                    (*packet_ptr)->nx_packet_length += size;
                } else {
                    /* Release the packet.  */
                    nx_packet_release(*packet_ptr);
                }
#endif    
            }

            /* ESP8266: Check status.  */
            if ((status != ESP_WIFI_STATUS_OK) || (size == 0))
            {
                /* ESP8266: Propagate link close error code */
                if (status == ESP_WIFI_STATUS_LINKCLOSED) {
                    break;
                }

                /* Convert the tick to millisecond.  */
                nx_wifi_tick_convert_ms((tx_time_get() - start_time), &millisecond); 
                
                /* Update the remaining millisecond.  */
                if (millisecond >= total_millisecond)
                    total_millisecond = 0;
                else
                    total_millisecond -=millisecond;

                continue;
            }
            else
            {
                received_packet = NX_TRUE;
                break;
            }
        }    
        
        /* Check if receive a packet.  */
        if (received_packet != NX_TRUE)
        {
            
            /* Release the IP internal mutex before processing the IP event.  */
            tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
            /* ESP8266: Propagate link close error code */
            return (status == ESP_WIFI_STATUS_LINKCLOSED) ? (NX_NOT_CONNECTED) : (NX_NO_PACKET);
        }

#if NX_WIFI_IM_RECV_BUFFER
        /* Allocate one packet to store the data.  */
        if (nx_packet_allocate(nx_wifi_pool, packet_ptr,  NX_RECEIVE_PACKET, NX_NO_WAIT))
        {
            /* ESP8266 */
            ESP_LOG_CRIT("Data lost due to failed nx_packet_allocate(...)\r\n");
            
            /* Release the IP internal mutex before processing the IP event.  */
            tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
            return(NX_NOT_SUCCESSFUL);
        }

        /* Set the data.  */
        if (nx_packet_data_append(*packet_ptr, nx_wifi_buffer, size, nx_wifi_pool, NX_NO_WAIT))
        {          
            /* ESP8266 */
            ESP_LOG_CRIT("Data lost due to failed nx_packet_data_append(...)\r\n");

            /* Release the packet.  */
            nx_packet_release(*packet_ptr);
            
            /* Release the IP internal mutex before processing the IP event.  */
            tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
            return(NX_NOT_SUCCESSFUL);
        }
#endif

        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_SUCCESS);
    }
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_tcp_client_socket_connect                   PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function sends wifi connection command                         */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    socket_ptr                            Socket pointer                */
/*    packet_ptr                            Pointer to received packet    */
/*    wait_option                           Suspension option             */
/*    socket_type                           Socket type                   */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UINT  nx_wifi_tcp_client_socket_connect(NX_TCP_SOCKET *socket_ptr,
                                        NXD_ADDRESS *server_ip,
                                        UINT server_port,
                                        ULONG wait_option)
{

/* ESP8266: Type */
ESP_WIFI_Status_t    status ;
UCHAR   entry_index;
  
    
    /* Obtain the IP internal mutex before processing the IP event.  */
    tx_mutex_get(&(nx_wifi_ip -> nx_ip_protection), TX_WAIT_FOREVER);
    
    /* Find an avaiable entry.  */
    if (nx_wifi_socket_entry_find((void *)socket_ptr, &entry_index, 0))
    {
      
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }
    
    /* Set the entry info.  */
    nx_wifi_socket[entry_index].nx_wifi_socket_ptr = (void *)socket_ptr;
    nx_wifi_socket[entry_index].nx_wifi_socket_valid = 1;
    nx_wifi_socket[entry_index].nx_wifi_socket_type = NX_WIFI_TCP_SOCKET; 
    nx_wifi_socket[entry_index].nx_wifi_socket_connected = 0;
    /* ESP8266: Reset socket notify count */
    nx_wifi_socket[entry_index].nx_wifi_tcp_socket_disconnect_notify = 0;
    nx_wifi_socket[entry_index].nx_wifi_socket_receive_notify = 0;
    nx_wifi_socket_counter++;
    
    /* Swap the address.  */
    NX_CHANGE_ULONG_ENDIAN(server_ip -> nxd_ip_address.v4);
  
    /* ESP8266: Wifi connect.  */
    {
        ULONG   wait_millisecond;

        nx_wifi_tick_convert_ms(wait_option, &wait_millisecond); 
        ESP_LOG_WARN("Ignore TCP link local port(%d)\r\n", socket_ptr->nx_tcp_socket_port);
        if (wait_millisecond) {
            status = ESP_WIFI_OpenClientConnection2(entry_index, ESP_WIFI_TCP, (uint8_t *) &server_ip -> nxd_ip_address.v4, server_port, 0, wait_millisecond);
        } else {
            /* No support for non-blocking connect, deteriorate to timed-wait of default timeout */
            ESP_LOG_WARN("No support for non-blocking connect, deteriorate to timed-wait\r\n");
            status = ESP_WIFI_OpenClientConnection(entry_index, ESP_WIFI_TCP, (uint8_t *) &server_ip -> nxd_ip_address.v4, server_port, 0);
        }
    }

    /* Swap the address.  */
    NX_CHANGE_ULONG_ENDIAN(server_ip -> nxd_ip_address.v4);
    
    /* ESP8266 */
    if(status == ESP_WIFI_STATUS_OK)
    {     
            
        /* Update the connect flag.  */
        nx_wifi_socket[entry_index].nx_wifi_socket_connected = 1;   
        
        /* Update the address.  */
        socket_ptr -> nx_tcp_socket_connect_ip.nxd_ip_version = NX_IP_VERSION_V4;
        socket_ptr -> nx_tcp_socket_connect_ip.nxd_ip_address.v4 = server_ip -> nxd_ip_address.v4;
        socket_ptr -> nx_tcp_socket_state =  NX_TCP_ESTABLISHED;  
        
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        
#ifndef NX_DISABLE_EXTENDED_NOTIFY_SUPPORT
        /* If registered with the TCP socket, call the application's connection completion callback function.  */
        if (socket_ptr -> nx_tcp_establish_notify) {
            /* Call the application's establish callback function.    */
            socket_ptr -> nx_tcp_establish_notify(socket_ptr);
        }
#endif

        return(NX_SUCCESS); 
    }
    else
    {
      
        /* Reset the entry.  */
        nx_wifi_socket_reset(entry_index);
        
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_tcp_socket_disconnect                       PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function sends wifi disconnect command                         */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    socket_ptr                            Pointer to TCP client socket  */
/*    wait_option                           Suspension option             */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UINT  nx_wifi_tcp_socket_disconnect(NX_TCP_SOCKET *socket_ptr, ULONG wait_option)
{
  
UCHAR   entry_index;


    /* Obtain the IP internal mutex before processing the IP event.  */
    tx_mutex_get(&(nx_wifi_ip -> nx_ip_protection), TX_WAIT_FOREVER);
    
    /* Check if the entry already exist.  */
    if (nx_wifi_socket_entry_find((void *)socket_ptr, &entry_index, 1))
    {
      
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }
    
    /* Check if the socket is connected.  */
    if (nx_wifi_socket[entry_index].nx_wifi_socket_connected == 0)
    {
      
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }
    
    /* ESP8266: Close connection.  */
    {
        ULONG   wait_millisecond;

        nx_wifi_tick_convert_ms(wait_option, &wait_millisecond); 
        if (wait_millisecond) {
            ESP_WIFI_CloseClientConnection2(entry_index, wait_millisecond);
        } else {
            /* No support for non-blocking disconnect, deteriorate to timed-wait of default timeout */
            ESP_LOG_WARN("No support for non-blocking disconnect, deteriorate to timed-wait\r\n");
            ESP_WIFI_CloseClientConnection(entry_index);
        }

#ifndef NX_DISABLE_EXTENDED_NOTIFY_SUPPORT
        /* Is a disconnect complete callback registered with the TCP socket? */
        if (socket_ptr->nx_tcp_disconnect_complete_notify) {
            /* Notify the application through the socket disconnect_complete callback */
            socket_ptr->nx_tcp_disconnect_complete_notify(socket_ptr);
        }
#endif

    }

    /* Reset the entry.  */   
    socket_ptr -> nx_tcp_socket_state = NX_TCP_CLOSED;  
      
    /* Reset the entry.  */
    nx_wifi_socket_reset(entry_index);
        
    /* Release the IP internal mutex before processing the IP event.  */
    tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
    
    /* Return success.  */
    return(NX_SUCCESS);
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_tcp_socket_send                             PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function sends a TCP packet.                                   */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    socket_ptr                            Pointer to socket             */
/*    packet_ptr                            Pointer to packet to send     */
/*    wait_option                           Suspension option             */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UINT  nx_wifi_tcp_socket_send(NX_TCP_SOCKET *socket_ptr, NX_PACKET *packet_ptr, ULONG wait_option)
{

/* ESP8266: Type */
ESP_WIFI_Status_t   status ;
UCHAR       entry_index;
uint16_t      send_data_length;
ULONG       packet_size;
NX_PACKET   *current_packet;

    
    /* Obtain the IP internal mutex before processing the IP event.  */
    tx_mutex_get(&(nx_wifi_ip -> nx_ip_protection), TX_WAIT_FOREVER);
        
    /* Find an avaiable entry.  */
    if (nx_wifi_socket_entry_find((void *)socket_ptr, &entry_index, 1))
    {
      
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }
    
    /* Check if the socket is connected.  */
    if (nx_wifi_socket[entry_index].nx_wifi_socket_connected == 0)
    {
      
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }
    
    /* Initialize the current packet to the input packet pointer.  */
    current_packet =  packet_ptr;
    
    /* Loop to send the packet.  */
    while(current_packet)
    {
      
        /* Calculate current packet size. */
        packet_size = (ULONG)(current_packet -> nx_packet_append_ptr - current_packet -> nx_packet_prepend_ptr);
      
        /* ESP8266: Send data.  */        
        status = ESP_WIFI_Send2(entry_index, current_packet -> nx_packet_prepend_ptr, 
                               packet_size, &send_data_length, WIFI_WRITE_TIMEOUT);

        /* ESP8266: Check status.  */
        if ((status != ESP_WIFI_STATUS_OK) || (send_data_length != packet_size))
        {
          
            /* Release the IP internal mutex before processing the IP event.  */
            tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
            /* ESP8266: Propagate link close error code */
            return (status == ESP_WIFI_STATUS_LINKCLOSED) ? (NX_NOT_CONNECTED) : (NX_NOT_SUCCESSFUL);
        }
         
#ifndef NX_DISABLE_PACKET_CHAIN
        /* We have crossed the packet boundary.  Move to the next packet
           structure.  */
        current_packet =  current_packet -> nx_packet_next;
#else

        /* End the loop.  */
        current_packet = NX_NULL;
#endif /* NX_DISABLE_PACKET_CHAIN */
    }
     
    /* Release the packet.  */
    nx_packet_release(packet_ptr);
    
    /* Release the IP internal mutex before processing the IP event.  */
    tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
    
    return (NX_SUCCESS);      
}

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_tcp_socket_receive                          PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function receives a TCP packet.                                */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    socket_ptr                            Pointer to socket             */
/*    packet_ptr                            Pointer to packet pointer     */
/*    wait_option                           Suspension option             */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UINT  nx_wifi_tcp_socket_receive(NX_TCP_SOCKET *socket_ptr, NX_PACKET **packet_ptr, ULONG wait_option)
{
    return(nx_wifi_socket_receive((VOID*)socket_ptr, packet_ptr, wait_option, NX_WIFI_TCP_SOCKET));
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_udp_socket_bind                             PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function binds UDP socket.                                     */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    socket_ptr                            Pointer to UDP socket         */
/*    port                                  16-bit UDP port number        */
/*    wait_option                           Suspension option             */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UINT  nx_wifi_udp_socket_bind(NX_UDP_SOCKET *socket_ptr, UINT  port, ULONG wait_option)
{
UCHAR       entry_index;

    /* Obtain the IP internal mutex before processing the IP event.  */
    tx_mutex_get(&(nx_wifi_ip -> nx_ip_protection), TX_WAIT_FOREVER);
    
    /* Find an avaiable entry.  */
    if (nx_wifi_socket_entry_find((void *)socket_ptr, &entry_index, 0))
    {
      
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }
    
    /* Set the entry info.  */
    nx_wifi_socket[entry_index].nx_wifi_socket_ptr = (void *)socket_ptr;
    nx_wifi_socket[entry_index].nx_wifi_socket_valid = 1;
    nx_wifi_socket[entry_index].nx_wifi_socket_type = NX_WIFI_UDP_SOCKET;
    nx_wifi_socket[entry_index].nx_wifi_socket_connected = 0;
    /* ESP8266: Reset socket notify count */
    nx_wifi_socket[entry_index].nx_wifi_tcp_socket_disconnect_notify = 0;
    nx_wifi_socket[entry_index].nx_wifi_socket_receive_notify = 0;
    nx_wifi_socket_counter++;

    /* ESP8266: Do link connection before bind/send/receive */

    /* Check if already open the connection.  */
    if (nx_wifi_udp_client_socket_connect(socket_ptr, entry_index, NULL, 0))
    {
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }

    /* Release the IP internal mutex before processing the IP event.  */
    tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        
    return(NX_SUCCESS);
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_udp_socket_unbind                           PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function unbinds UDP socket.                                   */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    socket_ptr                            Pointer to UDP socket         */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UINT  nx_wifi_udp_socket_unbind(NX_UDP_SOCKET *socket_ptr)
{

UCHAR   entry_index;

    /* Obtain the IP internal mutex before processing the IP event.  */
    tx_mutex_get(&(nx_wifi_ip -> nx_ip_protection), TX_WAIT_FOREVER);
        
    /* Check if the entry already exist.  */
    if (nx_wifi_socket_entry_find((void *)socket_ptr, &entry_index, 1))
    {
      
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }
    
    /* Check if the socket is connected.  */
    if (nx_wifi_socket[entry_index].nx_wifi_socket_connected == 0)
    {
      
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }
    
    /* ESP8266: Close connection.  */
    ESP_WIFI_CloseClientConnection(entry_index);

    /* Reset the entry.  */
    nx_wifi_socket_reset(entry_index);
        
    /* Release the IP internal mutex before processing the IP event.  */
    tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
    
    /* Return success.  */
    return(NX_SUCCESS);
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_udp_socket_send                             PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function sends UDP packet.                                     */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    socket_ptr                            Pointer to UDP socket         */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UINT  nx_wifi_udp_socket_send(NX_UDP_SOCKET *socket_ptr, NX_PACKET *packet_ptr, 
                              NXD_ADDRESS *ip_address, UINT port)
{

/* ESP8266: Type */
ESP_WIFI_Status_t   status ;
UCHAR       entry_index;
uint16_t    send_data_length;
ULONG       packet_size;
NX_PACKET   *current_packet;

    /* Obtain the IP internal mutex before processing the IP event.  */
    tx_mutex_get(&(nx_wifi_ip -> nx_ip_protection), TX_WAIT_FOREVER);
    
    /* Find an avaiable entry.  */
    if (nx_wifi_socket_entry_find((void *)socket_ptr, &entry_index, 1))
    {
      
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }

    /* ESP8266: Do link connection before bind/send/receive */

    /* Check if already open the connection.  */
    if (nx_wifi_udp_client_socket_connect(socket_ptr, entry_index, NULL, 0))
    {
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }
        
    /* Initialize the current packet to the input packet pointer.  */
    current_packet =  packet_ptr;
    
    /* Loop to send the packet.  */
    while(current_packet)
    {
      
        /* Calculate current packet size. */
        packet_size = (ULONG)(current_packet -> nx_packet_append_ptr - current_packet -> nx_packet_prepend_ptr);

        /* Swap the address.  */
        NX_CHANGE_ULONG_ENDIAN(ip_address -> nxd_ip_address.v4);

        /* ESP8266: Loop to send data.  */ 
        status = ESP_WIFI_SendTo2(entry_index, (uint8_t *)current_packet-> nx_packet_prepend_ptr, 
                               packet_size, &send_data_length, 
                               (uint8_t *) &ip_address -> nxd_ip_address.v4, port,
                               WIFI_WRITE_TIMEOUT);

        /* Swap the address.  */
        NX_CHANGE_ULONG_ENDIAN(ip_address -> nxd_ip_address.v4);

        /* ESP8266: Check status.  */
        if ((status != ESP_WIFI_STATUS_OK) || (send_data_length != packet_size))
        {
          
            /* Release the IP internal mutex before processing the IP event.  */
            tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
            return(NX_NOT_SUCCESSFUL);
        }

#ifndef NX_DISABLE_PACKET_CHAIN
        /* We have crossed the packet boundary.  Move to the next packet
           structure.  */
        current_packet =  current_packet -> nx_packet_next;
#else

        /* End the loop.  */
        current_packet = NX_NULL;
#endif /* NX_DISABLE_PACKET_CHAIN */
    }
        
    /* Release the packet.  */
    nx_packet_release(packet_ptr);
    
    /* Release the IP internal mutex before processing the IP event.  */
    tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
    return(NX_SUCCESS);
} 


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    nx_wifi_udp_socket_receive                          PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */ 
/*                                                                        */
/*    This function receives UDP packet.                                  */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */
/*    socket_ptr                            Pointer to UDP socket         */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                Completion status             */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UINT  nx_wifi_udp_socket_receive(NX_UDP_SOCKET *socket_ptr, NX_PACKET **packet_ptr, ULONG wait_option)
{
UCHAR       entry_index;

    /* Obtain the IP internal mutex before processing the IP event.  */
    tx_mutex_get(&(nx_wifi_ip -> nx_ip_protection), TX_WAIT_FOREVER);
    
    /* Find an avaiable entry.  */
    if (nx_wifi_socket_entry_find((void *)socket_ptr, &entry_index, 1))
    {
      
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }

    /* ESP8266: Do link connection before bind/send/receive */

    /* Check if already open the connection.  */
    if (nx_wifi_udp_client_socket_connect(socket_ptr, entry_index, NULL, 0))
    {
        /* Release the IP internal mutex before processing the IP event.  */
        tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
        return(NX_NOT_SUCCESSFUL);
    }

    /* Release the IP internal mutex before processing the IP event.  */
    tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
     
    return(nx_wifi_socket_receive((VOID*)socket_ptr, packet_ptr, wait_option, NX_WIFI_UDP_SOCKET));
} 

/* ESP8266: Link close callback */
static void nxesp_close_handler(void *Ctx, uint8_t LinkID)
{
    //TX_INTERRUPT_SAVE_AREA
    UCHAR entry_index = LinkID;
    NXESP_CLOSE_CTX *close_ctx = (NXESP_CLOSE_CTX *) Ctx;

    ESP_ASSERT(close_ctx);
    ESP_ASSERT(close_ctx == &nxesp_close_ctx);

    /* Check link ID */
    if (LinkID >= NX_WIFI_SOCKET_COUNTER) {
        ESP_LOG_CRIT("Parameter error in link close callback: LinkID(%d)\r\n", LinkID);
        return;
    }

    /* Defer tcp socket disconnect notify to outside; Otherwise, recursive into ESP8266 driver. */
    nx_wifi_socket[entry_index].nx_wifi_tcp_socket_disconnect_notify ++;

    return;
}

/* ESP8266: Data receive callback */
static void nxesp_datarecv_handler(void *Ctx, uint8_t LinkID, uint8_t *Data, uint16_t Size)
{
    TX_INTERRUPT_SAVE_AREA
    NX_PACKET *packet_ptr = NULL;
    UCHAR entry_index = LinkID;
    NXESP_DATARECV_CTX *datarecv_ctx = (NXESP_DATARECV_CTX *) Ctx;

    ESP_ASSERT(datarecv_ctx);
    ESP_ASSERT(datarecv_ctx == &nxesp_datarecv_ctx);

    /* Check link ID */
    if (LinkID >= NX_WIFI_SOCKET_COUNTER) {
        ESP_LOG_CRIT("Parameter error in data receive callback: LinkID(%d)\r\n", LinkID);
        return;
    }

    ESP_ASSERT(nx_wifi_pool);

    /* ESP8266: Null Data/Size, inform that proactive receive link +CIPRECVDATA data is expected */
    if (!Data || !Size) {
        /* Defer socket receive notify to outside; Otherwise, recursive into ESP8266 driver. */
        nx_wifi_socket[entry_index].nx_wifi_socket_receive_notify ++;
        return;
    }

    /* Allocate one packet to store the data.  */
    if (nx_packet_allocate(nx_wifi_pool, &packet_ptr,  NX_RECEIVE_PACKET, NX_NO_WAIT)) {
        ESP_LOG_WARN("Receive data lost due to failed nx_packet_allocate(...)\r\n");
        return;
    }
    ESP_ASSERT(packet_ptr);
       
    /* Set the data.  */
    if (nx_packet_data_append(packet_ptr, Data, Size, nx_wifi_pool, NX_NO_WAIT)) {
        ESP_LOG_WARN("Receive data lost due to failed nx_packet_data_append(...)\r\n");
        nx_packet_release(packet_ptr);
        return;
    }

    /* Disable interrupts.  */
    TX_DISABLE
          
    /* Check to see if the deferred processing queue is empty.  */
    if (nx_wifi_socket[entry_index].nx_wifi_received_packet_head)
    {

        /* Not empty, just place the packet at the end of the queue.  */
        (nx_wifi_socket[entry_index].nx_wifi_received_packet_tail) -> nx_packet_queue_next =  packet_ptr;
        packet_ptr -> nx_packet_queue_next =  NX_NULL;
        nx_wifi_socket[entry_index].nx_wifi_received_packet_tail =  packet_ptr;

        /* Restore interrupts.  */
        TX_RESTORE
    }
    else
    {

        /* Empty deferred receive processing queue.  Just setup the head pointers and
           set the event flags to ensure the IP helper thread looks at the deferred processing
           queue.  */
        nx_wifi_socket[entry_index].nx_wifi_received_packet_head =  packet_ptr;
        nx_wifi_socket[entry_index].nx_wifi_received_packet_tail =  packet_ptr;
        packet_ptr -> nx_packet_queue_next =             NX_NULL;

        /* Restore interrupts.  */
        TX_RESTORE

        /* Defer socket receive notify to outside; Otherwise, recursive into ESP8266 driver. */
        nx_wifi_socket[entry_index].nx_wifi_socket_receive_notify ++;
    }
}

/* ESP8266: Notify tcp socket disconnect */
static void nx_notify_tcp_socket_disconnect(uint8_t entry_index)
{
    NX_TCP_SOCKET   *tcp_socket;

    if (nx_wifi_socket[entry_index].nx_wifi_tcp_socket_disconnect_notify) {
        nx_wifi_socket[entry_index].nx_wifi_tcp_socket_disconnect_notify = 0;
    } else {
        return;
    }

    /* Check the socket type.  */
    if (nx_wifi_socket[entry_index].nx_wifi_socket_type == NX_WIFI_TCP_SOCKET)
    {
                            
        /* Get the tcp socket.  */
        tcp_socket = (NX_TCP_SOCKET *)nx_wifi_socket[entry_index].nx_wifi_socket_ptr;
        
        /* Determine if there is a tcp socket disconnect notification function specified.  */
        if (tcp_socket -> nx_tcp_disconnect_callback)
        {

            /* Yes, notification is requested.  Call the application's disconnect notification
               function for this socket.  */
            (tcp_socket -> nx_tcp_disconnect_callback)(tcp_socket);
        }
    }
}

/* ESP8266: Notify socket receive */
static void nx_notify_socket_receive(uint8_t entry_index)
{
    NX_TCP_SOCKET   *tcp_socket;
    NX_UDP_SOCKET   *udp_socket;

    if (nx_wifi_socket[entry_index].nx_wifi_socket_receive_notify) {
        nx_wifi_socket[entry_index].nx_wifi_socket_receive_notify = 0;
    } else {
        return;
    }

    /* Check the socket type.  */
    if (nx_wifi_socket[entry_index].nx_wifi_socket_type == NX_WIFI_TCP_SOCKET)
    {
                            
        /* Get the tcp socket.  */
        tcp_socket = (NX_TCP_SOCKET *)nx_wifi_socket[entry_index].nx_wifi_socket_ptr;
        
        /* Determine if there is a socket receive notification function specified.  */
        if (tcp_socket -> nx_tcp_receive_callback)
        {

            /* Yes, notification is requested.  Call the application's receive notification
               function for this socket.  */
               (tcp_socket -> nx_tcp_receive_callback)(tcp_socket);
        }
    }
    else
    {
                          
        /* Get the udp socket.  */
        udp_socket = (NX_UDP_SOCKET *)nx_wifi_socket[entry_index].nx_wifi_socket_ptr;
                            
        /* Determine if there is a socket receive notification function specified.  */
        if (udp_socket -> nx_udp_receive_callback)
        {

            /* Yes, notification is requested.  Call the application's receive notification
               function for this socket.  */
            (udp_socket -> nx_udp_receive_callback)(udp_socket);
        }                     
    }
}

/* ESP8266: UDP client connect */
static UINT nx_wifi_udp_client_socket_connect(NX_UDP_SOCKET *socket_ptr,
                                              UCHAR entry_index,
                                              NXD_ADDRESS *server_ip,
                                              UINT server_port)
{
    ESP_WIFI_Status_t   status ;

    /* Obtain the IP internal mutex before processing the IP event.  */
    tx_mutex_get(&(nx_wifi_ip -> nx_ip_protection), TX_WAIT_FOREVER);

    /* Check if already open the connection.  */
    if (nx_wifi_socket[entry_index].nx_wifi_socket_connected == 0)
    {
        
        if (server_ip) {
            /* Swap the address.  */
            NX_CHANGE_ULONG_ENDIAN(server_ip -> nxd_ip_address.v4);

            /* ESP8266: Open connection, remote changeable */
            status = ESP_WIFI_OpenClientConnection(entry_index, ESP_WIFI_UDP, (uint8_t *) &server_ip -> nxd_ip_address.v4, server_port, (2 << 16) | socket_ptr -> nx_udp_socket_port);

            /* Swap the address.  */
            NX_CHANGE_ULONG_ENDIAN(server_ip -> nxd_ip_address.v4);
        }
        else {
            ULONG dummy_ip_address = 0;
            USHORT dummy_port = 0;

            /* Swap the address.  */
            NX_CHANGE_ULONG_ENDIAN(dummy_ip_address);

            /* ESP8266: Open connection, remote changeable */
            status = ESP_WIFI_OpenClientConnection(entry_index, ESP_WIFI_UDP, (uint8_t *) &dummy_ip_address, dummy_port, (2 << 16) | socket_ptr -> nx_udp_socket_port);

            /* Swap the address.  */
            NX_CHANGE_ULONG_ENDIAN(dummy_ip_address);
        }

        /* Check status.  */
        if(status)
        {
            /* Reset the entry.  */
            nx_wifi_socket_reset(entry_index);

            /* Release the IP internal mutex before processing the IP event.  */
            tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
            return(NX_NOT_SUCCESSFUL);
        }

        /* Update the connect flag.  */
        nx_wifi_socket[entry_index].nx_wifi_socket_connected = 1;   
    }

    /* Release the IP internal mutex before processing the IP event.  */
    tx_mutex_put(&(nx_wifi_ip -> nx_ip_protection));
    return(NX_SUCCESS);
}
