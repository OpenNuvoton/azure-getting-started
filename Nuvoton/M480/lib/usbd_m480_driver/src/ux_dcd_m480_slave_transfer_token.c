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
/**   STM32 Controller Driver                                             */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */
#include "NuMicro.h"
#include "ux_api.h"
#include "ux_dcd_m480_slave.h"
#include "ux_device_stack.h"
#include "ux_utility.h"

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                RELEASE       */
/*                                                                        */
/*    _ux_dcd_m480_slave_transfer_token                     PORTABLE C     */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is invoked under ISR when an event happens on a       */
/*    specific endpoint.                                                  */
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
/*                    */ 
/*    _ux_device_stack_control_request_process Process control request    */ 
/*    _ux_utility_semaphore_get             Get semaphore                 */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    STM32 Controller Driver                                             */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UINT  _ux_dcd_m480_slave_transfer_token(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_TRANSFER *transfer_request, ULONG token_stage)
{

	UX_SLAVE_ENDPOINT       *endpoint;
	UX_DCD_M480_SLAVE_ED     *ed;

	ULONG                   fifo_length;
	ULONG			               endpoint_index;
	UINT                     addr, u32Size;
	ULONG                       request;
	ULONG                       request_value;

	
  /* Get the pointer to the logical endpoint from the transfer request.  */
  endpoint =  transfer_request -> ux_slave_transfer_request_endpoint;

  endpoint_index = endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION;
  ed =  (UX_DCD_M480_SLAVE_ED *) endpoint -> ux_slave_endpoint_ed;
    
    /* Get the pointer to the data buffer of the transfer request.  */
 // data_pointer =  transfer_request -> ux_slave_transfer_request_data_pointer;

  if ( endpoint_index == 0 )
  {     
    if (token_stage == SETUP_STAGE)
    {
       transfer_request -> ux_slave_transfer_request_actual_length =  0;
        /* Mark the transfer as successful.  */
       transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;
        
       /* Set the status of the endpoint to not stalled.  */
       ed -> ux_m480_slave_ed_status &= ~UX_DCD_M480_SLAVE_ED_STATUS_STALLED;
			 ed -> ux_m480_slave_ed_ping_pong = 0;
			 ed -> ux_m480_slave_ed_control_zero = 0;
       /* Check if the transaction is IN.  */
       if (*transfer_request -> ux_slave_transfer_request_setup & UX_REQUEST_IN)
       {            
          /* The endpoint is IN.  This is important to memorize the direction for the control endpoint
         in case of a STALL. */
         ed -> ux_m480_slave_ed_direction  = UX_ENDPOINT_IN;

          /* Call the Control Transfer dispatcher.  */
         if ( _ux_device_stack_control_request_process(transfer_request) == UX_SUCCESS )
				 {
					 // stage stage
					 USBD_SET_PAYLOAD_LEN(EP1, endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize);
				 }
				 else
				 {
						// setup error, stall the device
						USBD_SET_EP_STALL(EP0);
						USBD_SET_EP_STALL(EP1);
				 }
       }            
       else
       {

         /* The endpoint is OUT.  This is important to memorize the direction for the control endpoint
           in case of a STALL. */
          ed -> ux_m480_slave_ed_direction  = UX_ENDPOINT_OUT;

         /* Call the Control Transfer dispatcher.  */
          if ( _ux_device_stack_control_request_process(transfer_request) == UX_SUCCESS )
					{
				 // status stage
						USBD_SET_DATA1(EP0);
						USBD_SET_PAYLOAD_LEN(EP0, 0);
					}
					else
					{
						// setup error, stall the device
						USBD_SET_EP_STALL(EP0);
						USBD_SET_EP_STALL(EP1);
					}
       }
    }
    else if ( token_stage == CONTROL_IN_STAGE )
    {
			if (*transfer_request -> ux_slave_transfer_request_setup & UX_REQUEST_IN)
			{
				if (transfer_request -> ux_slave_transfer_request_in_transfer_length)
				{
					fifo_length = transfer_request -> ux_slave_transfer_request_in_transfer_length;
					if (fifo_length > endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize)
					{
          /* Process remained data */
						fifo_length = endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize;
					}
         /* Data size > MXPLD */
					addr = USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0);
					USBD_MemCopy((UCHAR *)addr, (UCHAR *)transfer_request->ux_slave_transfer_request_current_data_pointer, fifo_length);
					USBD_SET_PAYLOAD_LEN(EP0, fifo_length);
					transfer_request ->ux_slave_transfer_request_current_data_pointer += fifo_length;
					transfer_request -> ux_slave_transfer_request_in_transfer_length -= fifo_length;
				}
				else
				{
					if ( ed -> ux_m480_slave_ed_control_zero == 1 )
					{
						ed -> ux_m480_slave_ed_control_zero = 0;
						USBD_SET_PAYLOAD_LEN(EP0, 0);
					}
					// len = 0, and IN toke
				}
		  }
      else
      {
// Device OUT , Set address special processing
				request        =   *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);
        request_value  =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_VALUE);
				if ( request == UX_SET_ADDRESS )
					_ux_dcd_m480_slave_address_set(dcd_m480_slave, request_value);
	
      }
    }    
    else if ( token_stage == CONTROL_OUT_STAGE )
    {
			if ( ed -> ux_m480_slave_ed_ping_pong != (USBD->EPSTS0 & 0xf0))
			{
        ed -> ux_m480_slave_ed_ping_pong = USBD->EPSTS0 & 0xf0;
				
        if (transfer_request -> ux_slave_transfer_request_in_transfer_length)
        {
            u32Size = USBD_GET_PAYLOAD_LEN(EP1);
            addr = USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP1);
            USBD_MemCopy((uint8_t *)transfer_request->ux_slave_transfer_request_current_data_pointer, (uint8_t *)addr, u32Size);
            transfer_request->ux_slave_transfer_request_current_data_pointer += u32Size;
            transfer_request -> ux_slave_transfer_request_in_transfer_length -= u32Size;

            if(transfer_request -> ux_slave_transfer_request_in_transfer_length)
            {
                USBD_SET_PAYLOAD_LEN(EP1, endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize);
            }
        }
			}
			else
			{
        USBD_SET_PAYLOAD_LEN(EP1, endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize);
			}      
    }      
  }
  else
  {

  }

    /* We are done.  */
    return(UX_SUCCESS);
}

