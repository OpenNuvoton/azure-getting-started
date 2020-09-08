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
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_dcd_m480_slave_state_change                      PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will set the state of the controller to the desired   */
/*    value.                                                              */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_m480_slave                         Pointer to device controller  */
/*    state                                 Desired state                 */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
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
extern volatile UINT g_u32UsedBufferSize;

UINT  _ux_dcd_m480_slave_state_change(UX_DCD_M480_SLAVE *dcd_m480_slave, ULONG state)
{
#if 1
	UX_SLAVE_ENDPOINT       *endpoint;
	UX_SLAVE_DEVICE         *device;
	UX_SLAVE_INTERFACE			*interface;
	ULONG										 endpoint_number,value;
	int                      ep_index, addr_base;
  uint32_t	direction;
  static int setting=1;
#endif        
    UX_PARAMETER_NOT_USED(dcd_m480_slave);
    UX_PARAMETER_NOT_USED(state);
	if (_ux_system_slave->ux_system_slave_speed == UX_FULL_SPEED_DEVICE) 
	{
		addr_base = g_u32UsedBufferSize;
	}
	else
	{
		addr_base = g_u32UsedBufferSize;
	}
  if ( (state == UX_DEVICE_CONFIGURED ) && setting == 1 ) 
	{
		device =  &_ux_system_slave -> ux_system_slave_device;
		setting = 0;
		endpoint_number = device ->ux_slave_device_endpoints_pool_number;
  	interface = device->ux_slave_device_first_interface;
		endpoint = interface->ux_slave_interface_first_endpoint;
//		endpoint = device->ux_slave_device_endpoints_pool;
//		while (  endpoint_number != 0 )
    while (endpoint != UX_NULL)
		{
			ep_index = endpoint->ux_slave_endpoint_descriptor.bEndpointAddress;
//			ep_size = endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize;
//			ep_attr = endpoint->ux_slave_endpoint_descriptor.bmAttributes;
					    /*****************************************************/
			if (_ux_system_slave->ux_system_slave_speed == UX_FULL_SPEED_DEVICE) 
			{
				if (ep_index & UX_ENDPOINT_DIRECTION)
				{
					ep_index &=  (~UX_ENDPOINT_DIRECTION);
					value = USBD_CFG_EPMODE_IN | ep_index;
				}
				else 
				{
					value = USBD_CFG_EPMODE_OUT | ep_index ;
				}
				if ( (endpoint->ux_slave_endpoint_descriptor.bmAttributes & 0x03) == UX_ISOCHRONOUS_ENDPOINT)
					value |= USBD_CFG_TYPE_ISO;
    		USBD_CONFIG_EP(EP1+ep_index, value);
				USBD_SET_EP_BUF_ADDR(EP1+ep_index, addr_base);
				addr_base += (((endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize+7)>>3)<<3);
				if ( (endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_OUT ) 
					USBD_SET_PAYLOAD_LEN(EP1+ep_index,endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize);
			}
			else
			{
				// high speed
				if (ep_index & UX_ENDPOINT_DIRECTION)
				{
					ep_index &=  (~UX_ENDPOINT_DIRECTION);
					direction = HSUSBD_EP_CFG_DIR_IN;
				}
				else 
				{
					direction = HSUSBD_EP_CFG_DIR_OUT;
				}
				if ( (endpoint->ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE ) == UX_ISOCHRONOUS_ENDPOINT)
					value = HSUSBD_EP_CFG_TYPE_ISO;
				else if ( (endpoint->ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT)
					value = HSUSBD_EP_CFG_TYPE_BULK;
				else if ( (endpoint->ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_INTERRUPT_ENDPOINT )
 					value = HSUSBD_EP_CFG_TYPE_INT;   
        HSUSBD->EP[ep_index-1].EPRSPCTL = HSUSBD_EPRSPCTL_FLUSH_Msk;
				HSUSBD_SetEpBufAddr(ep_index-1, addr_base, endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize);
				HSUSBD_SET_MAX_PAYLOAD(ep_index-1, ((endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize+7)>>3)<<3);
				HSUSBD_ConfigEp(ep_index-1, ep_index, value, direction);
				addr_base += (((endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize+7)>>3)<<3);
				if ( (endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_OUT ) 
				{
					if ( value == HSUSBD_EP_CFG_TYPE_ISO )
					{
						HSUSBD_ENABLE_EP_INT(ep_index-1, HSUSBD_EPINTEN_RXPKIEN_Msk|HSUSBD_EPINTEN_SHORTRXIEN_Msk);
					}
					else
					{
						HSUSBD_ENABLE_EP_INT(ep_index-1, HSUSBD_EPINTEN_RXPKIEN_Msk);	
					}
				}
				else
				{
					// IN
					if ( value == HSUSBD_EP_CFG_TYPE_ISO )
					{
						HSUSBD_ENABLE_EP_INT(ep_index-1, HSUSBD_EPINTEN_TXPKIEN_Msk);
					}
				}
				HSUSBD->GINTEN |= (1 << (ep_index+1));
			}
			endpoint_number--;
			endpoint = endpoint->ux_slave_endpoint_next_endpoint;
		}

	}
    /* Nothing to do in simulation mode.  */
    return(UX_SUCCESS);         
}

