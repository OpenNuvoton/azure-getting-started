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
/** USBX Component                                                        */ 
/**                                                                       */
/**   Slave Simulator Controller Driver                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */
#include "NuMicro.h"
#include "ux_api.h"
#include "ux_dcd_m480_slave.h"

#define USBD_MAX_DMA_LEN		0x1000


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                RELEASE       */
/*                                                                        */
/*    _ux_dcd_m480_slave_transfer_request_hs                PORTABLE C     */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will initiate a transfer to a specific endpoint.      */
/*    If the endpoint is IN, the endpoint register will be set to accept  */
/*    the request.                                                        */
/*                                                                        */
/*    If the endpoint is IN, the endpoint FIFO will be filled with the    */
/*    buffer and the endpoint register set.                               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_m480_slave                         Pointer to device controller  */
/*    transfer_request                      Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_semaphore_get             Get semaphore                 */ 
/*    _ux_dcd_m480_slave_transfer_abort      Abort transfer                */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Slave Simulator Controller Driver                                   */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
void MSC_BulkOut(uint32_t ep_index, uint32_t u32Len, UX_SLAVE_TRANSFER *transfer_request, UX_DCD_M480_SLAVE_ED  *ed);
void MSC_BulkIn(uint32_t ep_index, uint32_t u32Len, uint32_t u32EpMaxPacketSize, UX_SLAVE_TRANSFER *transfer_request, UX_DCD_M480_SLAVE_ED  *ed);
UINT MSC_ActiveDMA(uint32_t u32Addr, uint32_t u32Len, UX_SLAVE_TRANSFER *transfer_request, UX_DCD_M480_SLAVE_ED  *ed);

uint8_t g_hsusbd_ShortPacket;

volatile UX_DCD_M480_SLAVE *g_dcd_m480_slave;

UINT  _ux_dcd_m480_slave_transfer_request_hs(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_TRANSFER *transfer_request)
{

	UX_SLAVE_ENDPOINT       *endpoint;
	UX_DCD_M480_SLAVE_ED     *ed;
	UINT                    status=0;
	UINT    fifo_length;
	UINT    ep_index, i, cnt;
	UCHAR u8Value;
	UINT	 	u32Len;
	UINT   *pu32Src;
  UCHAR   *pu8Src;
  UINT   div, rem;
	UINT   endpoint_attr;


		g_dcd_m480_slave = dcd_m480_slave;
	
    /* Get the pointer to the logical endpoint from the transfer request.  */
    endpoint =  transfer_request -> ux_slave_transfer_request_endpoint;

    /* Get the slave endpoint.  */
    ed = (UX_DCD_M480_SLAVE_ED *) endpoint -> ux_slave_endpoint_ed;
    
    /* We have a request for a OUT or IN transaction from the host.
       If the endpoint is a Control endpoint, all this is happening under Interrupt and there is no
       thread to suspend.  */
    if (ed -> ux_m480_slave_ed_index != 0)
    {
// Ray added
			ep_index = EPA + (ed -> ux_m480_slave_ed_index -1 );
			endpoint_attr = endpoint->ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE;
		  if ( transfer_request -> ux_slave_transfer_request_phase == UX_TRANSFER_PHASE_DATA_OUT )
			{
				fifo_length = transfer_request->ux_slave_transfer_request_requested_length;
				switch (endpoint_attr)
				{
					case UX_INTERRUPT_ENDPOINT :
				while ( fifo_length != 0 )
				{
					if ( fifo_length > endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize )
						u32Len =  endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize;
					else
						u32Len = fifo_length;
					if ( dcd_m480_slave -> ux_dcd_m480_slave_defined == HID_MOUSE_USER_DEFINED )
					{
						*(UCHAR *)(transfer_request ->ux_slave_transfer_request_current_data_pointer) = 0;
					}
#if 0
					pu32Src = (UINT *)transfer_request ->ux_slave_transfer_request_current_data_pointer;
					div = u32Len >> 2;
					rem = u32Len % 4;
					for (i=0; i< div; i++)
					{
						HSUSBD->EP[ep_index].EPDAT= *pu32Src++;
					}
					pu8Src = (UCHAR *)pu32Src;
					for (i=0; i<rem; i++)
		  		{
							HSUSBD->EP[ep_index].EPDAT_BYTE = *(UCHAR *)*pu8Src++;
					}
					transfer_request ->ux_slave_transfer_request_current_data_pointer+= u32Len;
#else
					for (i=0; i<u32Len; i++)
		  		{
							HSUSBD->EP[ep_index].EPDAT_BYTE = *(UCHAR *)(transfer_request ->ux_slave_transfer_request_current_data_pointer++);
					}
#endif

//					if ( u32Len < endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize || fifo_length == 0 )
//						HSUSBD->EP[ep_index].EPRSPCTL = HSUSBD_EP_RSPCTL_SHORTTXEN;
				  HSUSBD->EP[ep_index].EPTXCNT = u32Len;
					HSUSBD_ENABLE_EP_INT(ep_index, HSUSBD_EPINTEN_INTKIEN_Msk);
					transfer_request -> ux_slave_transfer_request_in_transfer_length -= u32Len;
					fifo_length -= u32Len;
//printf("Len= %d\n", u32Len);
//					for (i=0; i<650; i++); // OK
// 					for (i=0; i<1100; i++); // OK   
//					_ux_utility_delay_ms(1);
				 /* Set the ED to TRANSFER status.  */
					ed -> ux_m480_slave_ed_status |= UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER;
        /* We should wait for the semaphore to wake us up.  */
//printf("wait IN sema %d\n", transfer_request -> ux_slave_transfer_request_semaphore.tx_semaphore_count);
					status =  _ux_utility_semaphore_get(&transfer_request -> ux_slave_transfer_request_semaphore,
                                            transfer_request -> ux_slave_transfer_request_timeout);


					ed -> ux_m480_slave_ed_status &= ~(ULONG)UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER;	
//printf("exit IN sema %d\n", status);
//					while ( ( HSUSBD->EP[ep_index].EPDATCNT & 0x0000FFFF ) != u32Len);
					
					if (status != UX_SUCCESS)
					{
	          _ux_dcd_m480_slave_transfer_abort(dcd_m480_slave, transfer_request);
            transfer_request -> ux_slave_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;
            return(status);
					}
   
        /* Check the transfer request completion code. We may have had a BUS reset or
           a device disconnection.  */
					if (transfer_request -> ux_slave_transfer_request_completion_code != UX_SUCCESS)
            return(transfer_request -> ux_slave_transfer_request_completion_code);
				
				}
				  break;
					case UX_BULK_ENDPOINT:
// bulk in
						MSC_BulkIn(ep_index+1, fifo_length, endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize,transfer_request, ed);
						transfer_request -> ux_slave_transfer_request_in_transfer_length -= u32Len;
						transfer_request -> ux_slave_transfer_request_actual_length += u32Len;	
						 /* Set the ED to TRANSFER status.  */
	
						break;
					case UX_ISOCHRONOUS_ENDPOINT:
					//isochronous in
						break;
					default:
						printf("endpoint error\n");
						break;
				}

				return (UX_SUCCESS);
			}
			else if ( transfer_request -> ux_slave_transfer_request_phase == UX_TRANSFER_PHASE_DATA_IN )
			{

				fifo_length = transfer_request->ux_slave_transfer_request_requested_length;
				switch (endpoint_attr)
				{
					case UX_INTERRUPT_ENDPOINT :
				while ( fifo_length != 0)
				{
					ed -> ux_m480_slave_ed_status |= UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER;
        /* We should wait for the semaphore to wake us up.  */
//printf("wait OUT sema %d\n", transfer_request -> ux_slave_transfer_request_semaphore.tx_semaphore_count);
					status =  _ux_utility_semaphore_get(&transfer_request -> ux_slave_transfer_request_semaphore,
                                            transfer_request -> ux_slave_transfer_request_timeout);
//printf("exit OUT sema %d\n", status );
        /* Reset the ED to TRANSFER status.  */
					ed -> ux_m480_slave_ed_status &= ~(ULONG)UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER;
					if (status != UX_SUCCESS)
					{
            _ux_dcd_m480_slave_transfer_abort(dcd_m480_slave, transfer_request);
            transfer_request -> ux_slave_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;
            return(status);
					}
   
        /* Check the transfer request completion code. We may have had a BUS reset or
           a device disconnection.  */
					if (transfer_request -> ux_slave_transfer_request_completion_code != UX_SUCCESS)
            return(transfer_request -> ux_slave_transfer_request_completion_code);

					//Get data length from FIFO
					u32Len = HSUSBD->EP[ep_index].EPDATCNT & 0xFFFF;
					if ( u32Len < endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize )
					{
						fifo_length = 0;
					}
					else
					{
						fifo_length -= u32Len;
					}
#if 1
					for (i=0; i< u32Len; i++)
						*(UCHAR *)(transfer_request ->ux_slave_transfer_request_current_data_pointer++) = HSUSBD->EP[ep_index].EPDAT_BYTE;
#else
					pu32Src = (UINT *)transfer_request ->ux_slave_transfer_request_current_data_pointer;
					div = u32Len >> 2;
					rem = u32Len % 4;
					for (i=0; i< div; i++)
					{
						*pu32Src++ = HSUSBD->EP[ep_index].EPDAT;
					}
					pu8Src = (UCHAR *)pu32Src;
					for (i=0; i<rem; i++)
		  		{
							*(UCHAR *)pu8Src++ = HSUSBD->EP[ep_index].EPDAT_BYTE;
					}
					transfer_request ->ux_slave_transfer_request_current_data_pointer+= u32Len;	
#endif
					HSUSBD_ENABLE_EP_INT(ep_index, HSUSBD_EPINTEN_RXPKIEN_Msk);	
					transfer_request -> ux_slave_transfer_request_in_transfer_length -= u32Len;
					transfer_request -> ux_slave_transfer_request_actual_length += u32Len;
				}
					break;
					case UX_BULK_ENDPOINT:
						// bulk out
					//Get data length from FIFO
					// u32Len = HSUSBD->EP[ep_index].EPDATCNT & 0xFFFF;
					  if ( fifo_length == 64 )  // CBW
						{
							u32Len = 31; // CBW
							HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_DMADONEIEN_Msk|HSUSBD_BUSINTEN_SUSPENDIEN_Msk|HSUSBD_BUSINTEN_RSTIEN_Msk|HSUSBD_BUSINTEN_VBUSDETIEN_Msk);
						}
						else
							u32Len = fifo_length;
						MSC_BulkOut(ep_index+1, u32Len,transfer_request,ed);
						transfer_request -> ux_slave_transfer_request_in_transfer_length -= u32Len;
						transfer_request -> ux_slave_transfer_request_actual_length += u32Len;												
						break;
					case UX_ISOCHRONOUS_ENDPOINT:
						break;
					default:
						printf("endpoint error\n");
						break;
				}				
	//			printf("Iso./Int/Bulk out\n");
				return(UX_SUCCESS);
			}		
        /* Set the ED to TRANSFER status.  */
       ed -> ux_m480_slave_ed_status |= UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER;
        /* We should wait for the semaphore to wake us up.  */

        status =  _ux_utility_semaphore_get(&transfer_request -> ux_slave_transfer_request_semaphore,
                                            transfer_request -> ux_slave_transfer_request_timeout);


        /* Reset the ED to TRANSFER status.  */
        ed -> ux_m480_slave_ed_status &= ~(ULONG)UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER;
            
        /* Check the completion code. */
        if (status != UX_SUCCESS)
        {
            _ux_dcd_m480_slave_transfer_abort(dcd_m480_slave, transfer_request);
            transfer_request -> ux_slave_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;
            return(status);
        }
   
        /* Check the transfer request completion code. We may have had a BUS reset or
           a device disconnection.  */
        if (transfer_request -> ux_slave_transfer_request_completion_code != UX_SUCCESS)
            return(transfer_request -> ux_slave_transfer_request_completion_code);
    }
		else
		{
		  if ( transfer_request -> ux_slave_transfer_request_phase == UX_TRANSFER_PHASE_DATA_OUT )
			{
				// trasnfer size is the times of packet size
				if ( (transfer_request ->ux_slave_transfer_request_in_transfer_length % endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize) == 0 )
					ed -> ux_m480_slave_ed_control_zero = 1;
			}
		}
    /* Return to caller with success.  */
    return(UX_SUCCESS);
}

void MSC_BulkOut(uint32_t ep_index, uint32_t u32Len, UX_SLAVE_TRANSFER *transfer_request, UX_DCD_M480_SLAVE_ED  *ed)
{
    uint32_t u32Loop;
    uint32_t i;
		uint32_t u32Addr;
	
	  u32Addr = (uint32_t) transfer_request->ux_slave_transfer_request_current_data_pointer;
	
    /* bulk out, dma write, epnum = 2 */
    HSUSBD_SET_DMA_WRITE(ep_index);
    g_hsusbd_ShortPacket = 0;

    u32Loop = u32Len / USBD_MAX_DMA_LEN;
    for (i=0; i<u32Loop; i++)
    {
        MSC_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, USBD_MAX_DMA_LEN, transfer_request, ed);
    }

    u32Loop = u32Len % USBD_MAX_DMA_LEN;
    if (u32Loop)
    {
        MSC_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, u32Loop, transfer_request, ed);
    }
}

void MSC_BulkIn(uint32_t ep_index, uint32_t u32Len, uint32_t u32EpMaxPacketSize, UX_SLAVE_TRANSFER *transfer_request, UX_DCD_M480_SLAVE_ED  *ed)
{
    uint32_t u32Loop;
    uint32_t i, addr, count;
		uint32_t u32Addr;
	
	  u32Addr = (uint32_t) transfer_request->ux_slave_transfer_request_current_data_pointer;

    /* bulk in, dma read, epnum = 1 */
    HSUSBD_SET_DMA_READ(ep_index);

    u32Loop = u32Len / USBD_MAX_DMA_LEN;
    for (i=0; i<u32Loop; i++)
    {
        HSUSBD_ENABLE_EP_INT(ep_index-1, HSUSBD_EPINTEN_TXPKIEN_Msk);
        g_hsusbd_ShortPacket = 0;
        while(1)
        {
            if (HSUSBD_GET_EP_INT_FLAG(ep_index -1) & HSUSBD_EPINTSTS_BUFEMPTYIF_Msk)
            {
                MSC_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, USBD_MAX_DMA_LEN, transfer_request, ed);
                break;
            }
        }
    }

    addr = u32Addr + i * USBD_MAX_DMA_LEN;
    u32Loop = u32Len % USBD_MAX_DMA_LEN;
    if (u32Loop)
    {
        count = u32Loop / u32EpMaxPacketSize;
        if (count)
        {
            HSUSBD_ENABLE_EP_INT(ep_index-1, HSUSBD_EPINTEN_TXPKIEN_Msk);
            g_hsusbd_ShortPacket = 0;
            while(1)
            {
                if (HSUSBD_GET_EP_INT_FLAG(ep_index-1) & HSUSBD_EPINTSTS_BUFEMPTYIF_Msk)
                {
                    MSC_ActiveDMA(addr, count * u32EpMaxPacketSize, transfer_request, ed);
                    break;
                }
            }
            addr += (count * u32EpMaxPacketSize);
        }
        count = u32Loop % u32EpMaxPacketSize;
        if (count)
        {
            HSUSBD_ENABLE_EP_INT(ep_index-1, HSUSBD_EPINTEN_TXPKIEN_Msk);
            g_hsusbd_ShortPacket = 1;
            while(1)
            {
                if (HSUSBD_GET_EP_INT_FLAG(ep_index-1) & HSUSBD_EPINTSTS_BUFEMPTYIF_Msk)
                {
                    MSC_ActiveDMA(addr, count, transfer_request, ed);
                    break;
                }
            }
        }
    }
}

UINT MSC_ActiveDMA(uint32_t u32Addr, uint32_t u32Len, UX_SLAVE_TRANSFER *transfer_request, UX_DCD_M480_SLAVE_ED  *ed)
{
	UINT status=0;
    /* Enable BUS interrupt */
    HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_DMADONEIEN_Msk|HSUSBD_BUSINTEN_SUSPENDIEN_Msk|HSUSBD_BUSINTEN_RSTIEN_Msk|HSUSBD_BUSINTEN_VBUSDETIEN_Msk);

    HSUSBD_SET_DMA_ADDR(u32Addr);
    HSUSBD_SET_DMA_LEN(u32Len);
    HSUSBD_ENABLE_DMA();
		ed -> ux_m480_slave_ed_status |= UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER;
        /* We should wait for the semaphore to wake us up.  */
		status =  _ux_utility_semaphore_get(&transfer_request -> ux_slave_transfer_request_semaphore,
                                            transfer_request -> ux_slave_transfer_request_timeout);
//printf("exit OUT sema %d\n", status );
		ed -> ux_m480_slave_ed_status &= ~(ULONG)UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER;
		if (status != UX_SUCCESS)
		{
       _ux_dcd_m480_slave_transfer_abort(g_dcd_m480_slave, transfer_request);
       transfer_request -> ux_slave_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;
       return(status);
		}
   
   	if (transfer_request -> ux_slave_transfer_request_completion_code != UX_SUCCESS)
       return(transfer_request -> ux_slave_transfer_request_completion_code);

}
