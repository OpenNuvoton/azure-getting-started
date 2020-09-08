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
/*  FUNCTION                                              RELEASE         */
/*                                                                        */
/*    _ux_dcd_m480_slave_address_set                      PORTABLE C       */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will set the address of the device after we have      */
/*    received a SET_ADDRESS command from the host.                       */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_m480_slave                         Pointer to device controller  */
/*    address                               Address to set                */
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
UINT _ux_dcd_m480_slave_address_set(UX_DCD_M480_SLAVE *dcd_m480_slave,
		ULONG address) {
	ULONG addr;
	if (_ux_system_slave->ux_system_slave_speed == UX_FULL_SPEED_DEVICE) {
		addr = USBD_GET_ADDR();
		if ((addr != address) && (addr == 0ul)) 
		{
			USBD_SET_ADDR(address);
		}
	} else if (_ux_system_slave->ux_system_slave_speed == UX_HIGH_SPEED_DEVICE) {
		HSUSBD_SET_ADDR(address);
	}
	/* This function always succeeds.  */
	return (UX_SUCCESS);
}

