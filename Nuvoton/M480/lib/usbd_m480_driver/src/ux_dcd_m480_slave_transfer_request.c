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


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                RELEASE       */
/*                                                                        */
/*    _ux_dcd_m480_slave_transfer_request                   PORTABLE C     */
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
UINT  _ux_dcd_m480_slave_transfer_request(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_TRANSFER *transfer_request)
{

	UX_SLAVE_ENDPOINT       *endpoint;
	UX_DCD_M480_SLAVE_ED     *ed;
	UINT                    status=0;
	UINT    fifo_length;
	UINT    addr, ep_index;
	UCHAR   *pu8Src;
	UINT	 	u32Len;

	
		if ( _ux_system_slave -> ux_system_slave_speed == UX_HIGH_SPEED_DEVICE )
		{
			status = _ux_dcd_m480_slave_transfer_request_hs(dcd_m480_slave, transfer_request);		
			return(status);
		}
		
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
			ep_index = EP1 + ed -> ux_m480_slave_ed_index;
		  if ( transfer_request -> ux_slave_transfer_request_phase == UX_TRANSFER_PHASE_DATA_OUT )
			{
#if 1
				fifo_length = transfer_request->ux_slave_transfer_request_requested_length;
				while ( fifo_length != 0 )
				{
					if ( fifo_length > endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize )
						u32Len =  endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize;
					else
						u32Len = fifo_length;
					addr = USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(ep_index);
					USBD_MemCopy((UCHAR *)addr, (UCHAR *)transfer_request->ux_slave_transfer_request_current_data_pointer, u32Len);
					if ( dcd_m480_slave -> ux_dcd_m480_slave_defined == HID_MOUSE_USER_DEFINED )
					{
						pu8Src = (UCHAR *)addr;
						*pu8Src++ = 0;
					}
					USBD_SET_PAYLOAD_LEN(ep_index, u32Len);
					transfer_request ->ux_slave_transfer_request_current_data_pointer += u32Len;
					transfer_request -> ux_slave_transfer_request_in_transfer_length -= u32Len;

				  transfer_request -> ux_slave_transfer_request_actual_length += u32Len;
					fifo_length -= u32Len;
				 /* Set the ED to TRANSFER status.  */
					ed -> ux_m480_slave_ed_status |= UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER;
        /* We should wait for the semaphore to wake us up.  */
//printf("wait IN sema %d\n", transfer_request -> ux_slave_transfer_request_semaphore.tx_semaphore_count);
					status =  _ux_utility_semaphore_get(&transfer_request -> ux_slave_transfer_request_semaphore,
                                            transfer_request -> ux_slave_transfer_request_timeout);
					ed -> ux_m480_slave_ed_status &= ~(ULONG)UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER;	
//printf("exit IN sema %d\n", status);
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

				return (UX_SUCCESS);
#else
					fifo_length = transfer_request->ux_slave_transfer_request_requested_length;
					if ( fifo_length > endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize )
						fifo_length =  endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize;
				  addr = USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(ep_index);
					USBD_MemCopy((UCHAR *)addr, (UCHAR *)transfer_request->ux_slave_transfer_request_current_data_pointer, fifo_length);				
					if ( dcd_m480_slave -> ux_dcd_m480_slave_defined == HID_MOUSE_USER_DEFINED )
					{
						pu8Src = (UCHAR *)addr;
						*pu8Src++ = 0;
					}
					USBD_SET_PAYLOAD_LEN(ep_index, fifo_length);
					transfer_request ->ux_slave_transfer_request_current_data_pointer += fifo_length;
					transfer_request -> ux_slave_transfer_request_in_transfer_length -= fifo_length;

				  transfer_request -> ux_slave_transfer_request_actual_length += fifo_length;
#endif	


        /* Reset the ED to TRANSFER status.  */
    
			}
			else if ( transfer_request -> ux_slave_transfer_request_phase == UX_TRANSFER_PHASE_DATA_IN )
			{
				fifo_length = transfer_request->ux_slave_transfer_request_requested_length;
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

  				addr = USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(ep_index);
					u32Len = USBD_GET_PAYLOAD_LEN(ep_index);
					if ( u32Len < endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize )
					{
						fifo_length = 0;
					}
					else
					{
						fifo_length -= u32Len;
					}
					USBD_SET_PAYLOAD_LEN(ep_index, endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize );
					USBD_MemCopy((UCHAR *)transfer_request->ux_slave_transfer_request_current_data_pointer, (UCHAR *)addr, u32Len);
					transfer_request ->ux_slave_transfer_request_current_data_pointer += u32Len;
					transfer_request -> ux_slave_transfer_request_in_transfer_length -= u32Len;
					transfer_request -> ux_slave_transfer_request_actual_length += u32Len;
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
//        if ( device -> ux_slave_device_state == UX_DEVICE_SUSPENDED)
//						return(UX_SUCCESS);    
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

				fifo_length = transfer_request->ux_slave_transfer_request_requested_length;
				if ( fifo_length > endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize )
					fifo_length =  endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize;
				USBD_SET_DATA1(EP0);
				addr = USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0);
				USBD_MemCopy((UCHAR *)addr, (UCHAR *)transfer_request->ux_slave_transfer_request_current_data_pointer, fifo_length);
				USBD_SET_PAYLOAD_LEN(EP0, fifo_length);
				transfer_request ->ux_slave_transfer_request_current_data_pointer += fifo_length;
				transfer_request -> ux_slave_transfer_request_in_transfer_length -= fifo_length;
			}
			else if ( transfer_request -> ux_slave_transfer_request_phase == UX_TRANSFER_PHASE_DATA_IN )
			{
//				printf("device out\n");
			}		
		}
    /* Return to caller with success.  */
    return(UX_SUCCESS);
}

