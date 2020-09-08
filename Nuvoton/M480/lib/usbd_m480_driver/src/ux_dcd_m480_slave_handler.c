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




/* Include necessary system files.  */
#include "NuMicro.h"
#include "ux_api.h"
#include "ux_device_stack.h"
#include "ux_dcd_m480_slave.h"

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                 RELEASE      */
/*                                                                        */
/*    _ux_dcd_m480_slave_handler                   PORTABLE C    */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function run ISR for USB interrupt                             */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    None                                                                */ 
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_allocate           Allocate memory               */ 
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
UINT  _ux_dcd_m480_slave_handler(VOID)
{
	  UX_SLAVE_DCD            *dcd;
		UX_SLAVE_DEVICE         *device;
		UX_DCD_M480_SLAVE 			*dcd_m480_slave;
		UX_DCD_M480_SLAVE_ED    *ed;
	  UX_SLAVE_ENDPOINT				*endpoint;
	  UX_SLAVE_TRANSFER				*transfer_request;
	
    UINT volatile u32IntSts;
    UINT volatile u32State;
		UINT i;
	  UINT gintsts_epx;
    UINT ep_hw_index;
	  UINT ep_status;

	    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;
	   
	  dcd_m480_slave = (UX_DCD_M480_SLAVE *) dcd -> ux_slave_dcd_controller_hardware;
	
	
	  	u32IntSts = USBD_GET_INT_FLAG();
			u32State = USBD_GET_BUS_STATE();
//------------------------------------------------------------------
			if(u32IntSts & USBD_INTSTS_VBDETIF_Msk)
			{
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_VBDETIF_Msk);

        if (USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();
					  device -> ux_slave_device_state = UX_DEVICE_RESET;
//			printf("attach in\n");
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
			}

//------------------------------------------------------------------
			if(u32IntSts & USBD_INTSTS_BUSIF_Msk)
			{
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUSIF_Msk);

        if(u32State & USBD_ATTR_USBRST_Msk)
        {
            /* Bus reset */
            USBD_ENABLE_USB();

							for(i=0ul; i<USBD_MAX_EP; i++)
							{
									USBD->EP[i].CFG &= ~USBD_CFG_DSQSYNC_Msk;
							}

    /* Reset USB device address */
							USBD_SET_ADDR(0ul);
							_ux_dcd_m480_slave_initialize_complete();
  						if ( device -> ux_slave_device_state != UX_DEVICE_RESET )
									_ux_device_stack_disconnect();
							device -> ux_slave_device_state = UX_DEVICE_RESET;
//		printf("reset\n");
        }
        if(u32State & USBD_ATTR_SUSPEND_Msk)
        {
            /* Enable USB but disable PHY */
          USBD_DISABLE_PHY();
//			printf("suspend\n");
					if ( _ux_system_slave -> ux_system_slave_change_function != UX_NULL )
						_ux_system_slave -> ux_system_slave_change_function(UX_DEVICE_SUSPENDED);
          for (i=1; i< USBD_MAX_EP; i++)  
					{
						ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[i];
						if ( ed != UX_NULL )
						{
							if ( ed -> ux_m480_slave_ed_status & UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER == UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER )
							{
								endpoint = ed->ux_m480_slave_ed_endpoint;
								transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;
		  			    transfer_request -> ux_slave_transfer_request_completion_code = UX_SUCCESS;
			  		    transfer_request -> ux_slave_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;
				  	    _ux_utility_semaphore_put(&transfer_request-> ux_slave_transfer_request_semaphore);
								ed -> ux_m480_slave_ed_status &= ~(ULONG)UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER;
								transfer_request -> ux_slave_transfer_request_semaphore.tx_semaphore_count = 0;
							}
						}
					}
					if ( device -> ux_slave_device_state != UX_DEVICE_RESET )
						_ux_device_stack_disconnect();
//					device->ux_slave_device_state = UX_DEVICE_SUSPENDED;
        }
        if(u32State & USBD_ATTR_RESUME_Msk)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
						if ( _ux_system_slave -> ux_system_slave_change_function != UX_NULL )
							_ux_system_slave -> ux_system_slave_change_function(UX_DEVICE_RESUMED);
        }
			}

//------------------------------------------------------------------
			if(u32IntSts & USBD_INTSTS_WAKEUP)
			{
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_WAKEUP);
			}

			if(u32IntSts & USBD_INTSTS_USBIF_Msk)
			{
        // USB event
				if ( device -> ux_slave_device_state == UX_DEVICE_RESET )
					device -> ux_slave_device_state = UX_DEVICE_ATTACHED;
				
        if(u32IntSts & USBD_INTSTS_SETUP_Msk)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP_Msk);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);
//		printf("setup\n");
					  ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[0];  //Setup endpoint 0
					  endpoint = ed->ux_m480_slave_ed_endpoint;
					  transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;
						USBD_MemCopy(transfer_request->ux_slave_transfer_request_setup, (uint8_t *)USBD_BUF_BASE, UX_SETUP_SIZE);	
						_ux_dcd_m480_slave_transfer_token(dcd_m480_slave, transfer_request, SETUP_STAGE);
        }

        // EP events
        if(u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            // control IN
					  ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[0];
					  endpoint = ed->ux_m480_slave_ed_endpoint;
					  transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;
						_ux_dcd_m480_slave_transfer_token(dcd_m480_slave, transfer_request, CONTROL_IN_STAGE);
	
        }

        if(u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);

            // control OUT
					  ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[0];
					  endpoint = ed->ux_m480_slave_ed_endpoint;
					  transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;
 						_ux_dcd_m480_slave_transfer_token(dcd_m480_slave, transfer_request, CONTROL_OUT_STAGE);
        }
        gintsts_epx = (u32IntSts >> 18) & 0x3F;
        ep_hw_index = 2;
        while (gintsts_epx)
        {
            if(gintsts_epx & 0x01)
            {
                if(ep_hw_index < 8)
                    ep_status = (USBD->EPSTS0 >> (ep_hw_index * 4)) & 0xF;
                else
                    ep_status = (USBD->EPSTS1 >> ((ep_hw_index - 8) * 4)) & 0xF;
                /* Clear event flag */
                USBD_CLR_INT_FLAG(1 << (ep_hw_index + 16));

                if(ep_status == 0x02 || ep_status == 0x06 || (ep_status == 0x07 && (ep_hw_index & 0x01) == 1))
                {   /* RX, OUT token */
//		printf("OUT token  %d\n", ep_hw_index-1);
									ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[ep_hw_index-1];
									endpoint = ed->ux_m480_slave_ed_endpoint;
									transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;
									transfer_request -> ux_slave_transfer_request_completion_code = UX_SUCCESS;
									transfer_request -> ux_slave_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;
									_ux_utility_semaphore_put(&transfer_request-> ux_slave_transfer_request_semaphore);
                }
                else if(ep_status == 0x00 || ep_status == 0x07)
                {   /* TX , IN token*/
//		printf("IN token  %x\n", ep_status);
									ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[ep_hw_index-1];
									endpoint = ed->ux_m480_slave_ed_endpoint;
									transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;
									transfer_request -> ux_slave_transfer_request_completion_code = UX_SUCCESS;
									transfer_request -> ux_slave_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;
									_ux_utility_semaphore_put(&transfer_request-> ux_slave_transfer_request_semaphore);
                }
            }
            gintsts_epx = gintsts_epx >> 1;
            ep_hw_index++;
        }
			}
	  return(UX_SUCCESS);
}

