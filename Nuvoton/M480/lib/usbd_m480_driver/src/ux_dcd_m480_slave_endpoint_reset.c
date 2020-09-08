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
/*  FUNCTION                                                 RELEASE      */
/*                                                                        */
/*    _ux_dcd_m480_slave_endpoint_reset                      PORTABLE C    */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will reset a physical endpoint.                       */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_m480_slave                         Pointer to device controller  */
/*    endpoint                              Pointer to endpoint container */
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
UINT  _ux_dcd_m480_slave_endpoint_reset(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_ENDPOINT *endpoint)
{

		UX_DCD_M480_SLAVE_ED     *ed;

    UX_PARAMETER_NOT_USED(dcd_m480_slave);

    /* Get the physical endpoint address in the endpoint container.  */
    ed =  (UX_DCD_M480_SLAVE_ED *) endpoint -> ux_slave_endpoint_ed;

    /* Set the state of the endpoint to not stalled.  */
    ed -> ux_m480_slave_ed_status &=  ~(ULONG)UX_DCD_M480_SLAVE_ED_STATUS_STALLED;
	  if ( _ux_system_slave -> ux_system_slave_speed == UX_HIGH_SPEED_DEVICE )
		{
			HSUSBD_ClearStall(ed->ux_m480_slave_ed_index);
		}
		else
		{
			USBD_ClearStall(ed->ux_m480_slave_ed_index);
		}
    /* This function never fails.  */
    return(UX_SUCCESS);         
}

