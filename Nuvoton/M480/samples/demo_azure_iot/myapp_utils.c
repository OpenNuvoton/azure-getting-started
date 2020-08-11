/**************************************************************************//**
 * @file     myapp_utils.h.c
 * @version  V1.00
 * @brief    My application utility source file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

/* Standard includes */
#include <stdbool.h>

/* Azure RTOS includes */
#include "nx_api.h"

/* My application includes */
#include "myapp_utils.h"

UINT myapp_dump_packet(NX_PACKET *packet_ptr)
{
    UINT status = NX_SUCCESS;
    NX_PACKET *current_packet = packet_ptr;

    while (current_packet) {
        uint8_t *data = (uint8_t *) current_packet->nx_packet_prepend_ptr;
        uint32_t size = (uint32_t) (current_packet->nx_packet_append_ptr - current_packet->nx_packet_prepend_ptr);

        MYAPP_CHK_BOOL(data);

        myapp_dump_hex(data, size);
     
#ifndef NX_DISABLE_PACKET_CHAIN
        /* We have crossed the packet boundary. Move to the next packet structure. */
        current_packet =  current_packet->nx_packet_next;
#else
        /* End the loop */
        current_packet = NX_NULL;
#endif /* NX_DISABLE_PACKET_CHAIN */
    }

clean_up:

    return status;
}

void myapp_dump_hex(uint8_t *data, uint32_t size)
{
    uint8_t *data_ind = data;
    uint16_t rmn = size;
    bool end_newline = false;

    //printf("========Dump %d bytes hex data...========\r\n", size);

    while (rmn >= 16) {
        printf("%02x %02x %02x %02x %02x %02x %02x %02x "
               "%02x %02x %02x %02x %02x %02x %02x %02x " "\r\n",
               data_ind[0], data_ind[1], data_ind[2], data_ind[3],
               data_ind[4], data_ind[5], data_ind[6], data_ind[7],
               data_ind[8], data_ind[9], data_ind[10], data_ind[11],
               data_ind[12], data_ind[13], data_ind[14], data_ind[15]);
        data_ind += 16;
        rmn -= 16;
    }

    if (rmn >= 8) {
        printf("%02x %02x %02x %02x %02x %02x %02x %02x ", 
               data_ind[0], data_ind[1], data_ind[2], data_ind[3],
               data_ind[4], data_ind[5], data_ind[6], data_ind[7]);
        data_ind += 8;
        rmn -= 8;
        end_newline = true;
    }
    
    if (rmn >= 4) {
        printf("%02x %02x %02x %02x ",
               data_ind[0], data_ind[1], data_ind[2], data_ind[3]);
        data_ind += 4;
        rmn -= 4;
        end_newline = true;
    }
    
    if (rmn >= 2) {
        printf("%02x %02x ", data_ind[0], data_ind[1]);
        data_ind += 2;
        rmn -= 2;
        end_newline = true;
    }

    if (rmn) {
        printf("%02x ", *data_ind ++);
        rmn -= 1;
        end_newline = true;
    }

    if (end_newline) {
        printf("\r\n");
    }

    //printf("========Dump %d bytes hex data...END========\r\n", size);
}
