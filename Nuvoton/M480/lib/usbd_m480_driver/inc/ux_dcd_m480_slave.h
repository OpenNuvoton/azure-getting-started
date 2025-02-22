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


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_dcd_m480_slave.h                                  PORTABLE C      */ 
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX slave simulator. It is designed to work ONLY with the USBX     */ 
/*    host simulator.                                                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/

#ifndef UX_DCD_M480_SLAVE_H
#define UX_DCD_M480_SLAVE_H

	//Ray added
#define DEFAULT_DEVICE					0
#define HID_MOUSE_USER_DEFINED				1
#define AUDIO_DEVICE					2
#define AUDIO_NUVOTON					3
	
//Ray added 2020-07-20
#define SETUP_STAGE							1
#define CONTROL_IN_STAGE				2
#define CONTROL_OUT_STAGE				3
#define DATA_IN_STAGE						4
#define DATA_OUT_STAGE					5
#define UPDATE_DEVICE_STATUS		6


/* Define USB slave simulator major equivalences.  */

#define UX_DCD_M480_SLAVE_SLAVE_CONTROLLER                       98
#define UX_DCD_M480_SLAVE_MAX_ED                                 16


/* Define USB slave simulator error code register bits.  */

#define UX_DCD_M480_SLAVE_ERROR_TRANSMISSION_OK                  0x00000001
#define UX_DCD_M480_SLAVE_ERROR_CODE_MASK                        0x0000000e
#define UX_DCD_M480_SLAVE_ERROR_CODE_SHIFT                       0x00000001
#define UX_DCD_M480_SLAVE_ERROR_CODE_PID_ERROR                   0x00000001
#define UX_DCD_M480_SLAVE_ERROR_CODE_PID_UNKNOWN                 0x00000002
#define UX_DCD_M480_SLAVE_ERROR_CODE_UNEXPECTED_PACKET           0x00000003
#define UX_DCD_M480_SLAVE_ERROR_CODE_TOKEN_CRC                   0x00000004
#define UX_DCD_M480_SLAVE_ERROR_CODE_DATA_CRC                    0x00000005
#define UX_DCD_M480_SLAVE_ERROR_CODE_TIME_OUT                    0x00000006
#define UX_DCD_M480_SLAVE_ERROR_CODE_BABBLE                      0x00000007
#define UX_DCD_M480_SLAVE_ERROR_CODE_UNEXPECTED_EOP              0x00000008
#define UX_DCD_M480_SLAVE_ERROR_CODE_NAK                         0x00000009
#define UX_DCD_M480_SLAVE_ERROR_CODE_STALLED                     0x0000000a
#define UX_DCD_M480_SLAVE_ERROR_CODE_OVERFLOW                    0x0000000b
#define UX_DCD_M480_SLAVE_ERROR_CODE_EMPTY_PACKET                0x0000000c
#define UX_DCD_M480_SLAVE_ERROR_CODE_BIT_STUFFING                0x0000000d
#define UX_DCD_M480_SLAVE_ERROR_CODE_SYNC_ERROR                  0x0000000e
#define UX_DCD_M480_SLAVE_ERROR_CODE_DATA_TOGGLE                 0x0000000f


/* Define USB slave simulator physical endpoint status definition.  */

#define UX_DCD_M480_SLAVE_ED_STATUS_UNUSED                       0
#define UX_DCD_M480_SLAVE_ED_STATUS_USED                         1
#define UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER                     2
#define UX_DCD_M480_SLAVE_ED_STATUS_STALLED                      4


/* Define USB slave simulator physical endpoint structure.  */

typedef struct UX_DCD_M480_SLAVE_ED_STRUCT 
{

    ULONG           ux_m480_slave_ed_status;
    ULONG           ux_m480_slave_ed_index;
    ULONG           ux_m480_slave_ed_payload_length;
    ULONG           ux_m480_slave_ed_ping_pong;
    ULONG           ux_m480_slave_ed_status_register;
    ULONG           ux_m480_slave_ed_configuration_value;
	  ULONG           ux_m480_slave_ed_direction;
	// Ray added
	ULONG		ux_m480_slave_ed_control_zero;
    struct UX_SLAVE_ENDPOINT_STRUCT             
                    *ux_m480_slave_ed_endpoint;
} UX_DCD_M480_SLAVE_ED;


/* Define USB slave simulator DCD structure definition.  */

typedef struct UX_DCD_M480_SLAVE_STRUCT
{                                 
    struct UX_SLAVE_DCD_STRUCT                 
                    *ux_dcd_m480_slave_dcd_owner;
    struct UX_DCD_M480_SLAVE_ED_STRUCT              
                    ux_dcd_m480_slave_ed[UX_DCD_M480_SLAVE_MAX_ED];
  
		UINT            (*ux_dcd_m480_slave_dcd_control_request_process_hub)(UX_SLAVE_TRANSFER *transfer_request);
    ULONG	    	ux_dcd_m480_slave_defined;
    ULONG 	    ux_dcd_m480_ep0_packet_size;		
   
} UX_DCD_M480_SLAVE;


/* Define slave simulator function prototypes.  */

UINT    _ux_dcd_m480_slave_address_set(UX_DCD_M480_SLAVE *dcd_m480_slave, ULONG address);
UINT    _ux_dcd_m480_slave_endpoint_create(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_dcd_m480_slave_endpoint_destroy(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_dcd_m480_slave_endpoint_reset(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_dcd_m480_slave_endpoint_stall(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_dcd_m480_slave_endpoint_status(UX_DCD_M480_SLAVE *dcd_m480_slave, ULONG endpoint_index);
UINT    _ux_dcd_m480_slave_frame_number_get(UX_DCD_M480_SLAVE *dcd_m480_slave, ULONG *frame_number);
UINT    _ux_dcd_m480_slave_function(UX_SLAVE_DCD *dcd, UINT function, VOID *parameter);
UINT    _ux_dcd_m480_slave_initialize(ULONG usb_type,  ULONG CtrlMaxPktSize, ULONG defineddevice);
UINT    _ux_dcd_m480_slave_initialize_complete(VOID);
UINT    _ux_dcd_m480_slave_state_change(UX_DCD_M480_SLAVE *dcd_m480_slave, ULONG state);
UINT    _ux_dcd_m480_slave_transfer_request(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_TRANSFER *transfer_request);
UINT    _ux_dcd_m480_slave_transfer_abort(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_TRANSFER *transfer_request);
UINT    _ux_dcd_m480_slave_transfer_token(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_TRANSFER *transfer_request, ULONG token_stage);
UINT    _ux_dcd_m480_slave_transfer_token_hs(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_TRANSFER *transfer_request, ULONG token_stage);
UINT    _ux_dcd_m480_slave_transfer_request_hs(UX_DCD_M480_SLAVE *dcd_m480_slave, UX_SLAVE_TRANSFER *transfer_request);

/* Define Device Simulator Class API prototypes.  */

#define ux_dcd_m480_slave_initialize                 _ux_dcd_m480_slave_initialize
#endif

