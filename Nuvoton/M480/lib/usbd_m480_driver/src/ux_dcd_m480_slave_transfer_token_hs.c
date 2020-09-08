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
/*    _ux_dcd_m480_slave_transfer_token_hs                  PORTABLE C     */
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
UINT  _ux_dcd_m480_slave_transfer_token_hs(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_TRANSFER *transfer_request, ULONG token_stage)
{
	UX_SLAVE_DEVICE					*device;
	UX_SLAVE_ENDPOINT       *endpoint;
	UX_DCD_M480_SLAVE_ED     *ed;

	ULONG			               endpoint_index;
	ULONG                       request;
	ULONG                       request_value;
    uint32_t volatile i, cnt;
    uint8_t u8Value;

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
       ed -> ux_m480_slave_ed_control_zero = 0;
       /* Set the status of the endpoint to not stalled.  */
       ed -> ux_m480_slave_ed_status &= ~UX_DCD_M480_SLAVE_ED_STATUS_STALLED;
       /* Check if the transaction is IN.  */
       if (*transfer_request -> ux_slave_transfer_request_setup & UX_REQUEST_IN)
       {            
          /* The endpoint is IN.  This is important to memorize the direction for the control endpoint
         in case of a STALL. */
         ed -> ux_m480_slave_ed_direction  = UX_ENDPOINT_IN;

          /* Call the Control Transfer dispatcher.  */
         if ( _ux_device_stack_control_request_process(transfer_request) == UX_SUCCESS )
				 {
					/* Status Stage */
           HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
           HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_INTKIEN_Msk);
				 }
				 else
				 {
						/* Setup error, stall the device */
					 HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
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
            /* Status Stage */
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_STSDONEIEN_Msk);
					}
					else
					{
						/* Setup error, stall the device */
					  HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);	
					}
       }
    }
    else if ( token_stage == CONTROL_IN_STAGE )
    {
			if (*transfer_request -> ux_slave_transfer_request_setup & UX_REQUEST_IN)
			{
				if(transfer_request -> ux_slave_transfer_request_in_transfer_length >= endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize)
				{
        /* Data size > MXPLD */
					cnt = endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize >> 2;
					for (i=0ul; i<cnt; i++)
					{
            HSUSBD->CEPDAT = *(UINT *)transfer_request->ux_slave_transfer_request_current_data_pointer;
            transfer_request->ux_slave_transfer_request_current_data_pointer += 4;
					}
					HSUSBD_START_CEP_IN(endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize);
					transfer_request -> ux_slave_transfer_request_in_transfer_length -= endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize;
				}
				else
				{
        /* Data size <= MXPLD */
					cnt = transfer_request -> ux_slave_transfer_request_in_transfer_length >> 2;
					for (i=0ul; i<cnt; i++)
					{
            HSUSBD->CEPDAT = *(UINT *)transfer_request->ux_slave_transfer_request_current_data_pointer;
            transfer_request->ux_slave_transfer_request_current_data_pointer += 4ul;
					}
					for (i=0ul; i<(transfer_request -> ux_slave_transfer_request_in_transfer_length % 4ul); i++)
					{
            u8Value = *(uint8_t *)(transfer_request->ux_slave_transfer_request_current_data_pointer++);
            outpb(&HSUSBD->CEPDAT, u8Value);
					}
					HSUSBD_START_CEP_IN(transfer_request -> ux_slave_transfer_request_in_transfer_length);
					transfer_request -> ux_slave_transfer_request_in_transfer_length = 0ul;
				}
		  }
      else
      {
// Device OUT , Set address special processing
//				request        =   *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);
//        request_value  =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_VALUE);
//				if ( request == UX_SET_ADDRESS )
//					_ux_dcd_m480_slave_address_set(dcd_sim_slave, request_value);
				printf("Control in error\n");
	
      }
    }    
    else if ( token_stage == CONTROL_OUT_STAGE )
    {
			while(1)
			{
        if ((HSUSBD->CEPINTSTS & HSUSBD_CEPINTSTS_RXPKIF_Msk) == HSUSBD_CEPINTSTS_RXPKIF_Msk)
        {
            for (i=0ul; i<transfer_request -> ux_slave_transfer_request_in_transfer_length; i++)
            {
                *(UCHAR *)(transfer_request->ux_slave_transfer_request_current_data_pointer+i) = inpb(&HSUSBD->CEPDAT);
            }
            HSUSBD->CEPINTSTS = HSUSBD_CEPINTSTS_RXPKIF_Msk;
            break;
        }
			}
    }     
		else if ( token_stage == UPDATE_DEVICE_STATUS )
		{
			request        =   *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);
      request_value  =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_VALUE);
			device = &_ux_system_slave->ux_system_slave_device;
			switch (request)
			{
				case SET_ADDRESS:
				{
  			  _ux_dcd_m480_slave_address_set(dcd_m480_slave, request_value);
				}
				break;
				case SET_CONFIGURATION:
				{
					if ( device->ux_slave_device_configuration_selected == 0ul)
					{
             /* Reset PID DATA0 */
            for (i=0ul; i<HSUSBD_MAX_EP; i++)
            {
                if ((HSUSBD->EP[i].EPCFG & 0x1ul) == 0x1ul)
                {
                    HSUSBD->EP[i].EPRSPCTL = HSUSBD_EP_RSPCTL_TOGGLE;
                }
            }
					}
        }
        break;
			}
		}
  }
  else
  {

  }
    /* We are done.  */
  return(UX_SUCCESS);
}

