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
#include "ux_device_stack.h"
#include "ux_dcd_m480_slave.h"

extern uint8_t g_hsusbd_ShortPacket;
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                 RELEASE      */
/*                                                                        */
/*    _ux_dcd_m480_slave_handler                            PORTABLE C    */
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
UINT  _ux_dcd_m480_slave_handler_hs(VOID)
{
	  UX_SLAVE_DCD            *dcd;
		UX_SLAVE_DEVICE         *device;
		UX_DCD_M480_SLAVE 			*dcd_m480_slave;
		UX_DCD_M480_SLAVE_ED    *ed;
	  UX_SLAVE_ENDPOINT				*endpoint;
	  UX_SLAVE_TRANSFER				*transfer_request;
	
    uint32_t IrqStL, IrqSt;
		UINT i;
		uint32_t gintsts_epx;
    uint32_t ep_hw_index;
    uint32_t ep_logic_index;
		uint32_t volatile epxintsts;
	  uint32_t ep_index;

    IrqStL = HSUSBD->GINTSTS & HSUSBD->GINTEN;    /* get interrupt status */
    gintsts_epx = IrqStL >> 2;     /* EPA, EPB, EPC, ... EPL interrupts */
    ep_hw_index = 0;
    ep_logic_index = 0;
    if (!IrqStL)    
			return(UX_SUCCESS);
		
	    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;
	   
	  dcd_m480_slave = (UX_DCD_M480_SLAVE *) dcd -> ux_slave_dcd_controller_hardware;

   /* USB interrupt */
    if (IrqStL & HSUSBD_GINTSTS_USBIF_Msk)
    {
        IrqSt = HSUSBD->BUSINTSTS & HSUSBD->BUSINTEN;

        if (IrqSt & HSUSBD_BUSINTSTS_SOFIF_Msk)
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_SOFIF_Msk);

        if (IrqSt & HSUSBD_BUSINTSTS_RSTIF_Msk)
        {
 //           HSUSBD_SwReset();
					   /* Reset USB device address */
            HSUSBD_ResetDMA();
						HSUSBD_SET_ADDR(0ul);

            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk);
            HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk|HSUSBD_BUSINTEN_RESUMEIEN_Msk|HSUSBD_BUSINTEN_SUSPENDIEN_Msk);
            HSUSBD_CLR_SE0();
					
					  HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_RSTIF_Msk);
            HSUSBD_CLR_CEP_INT_FLAG(0x1ffc);
						_ux_dcd_m480_slave_initialize_complete();
  					if ( device -> ux_slave_device_state != UX_DEVICE_RESET )
  						_ux_device_stack_disconnect();
						device -> ux_slave_device_state = UX_DEVICE_RESET;
        }

        if (IrqSt & HSUSBD_BUSINTSTS_RESUMEIF_Msk)
        {
            HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk|HSUSBD_BUSINTEN_SUSPENDIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_RESUMEIF_Msk);
        }

        if (IrqSt & HSUSBD_BUSINTSTS_SUSPENDIF_Msk)
        {
            HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_RESUMEIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_SUSPENDIF_Msk);
						if ( _ux_system_slave -> ux_system_slave_change_function != UX_NULL )
							_ux_system_slave -> ux_system_slave_change_function(UX_DEVICE_SUSPENDED);
						for (i=1; i<=HSUSBD_MAX_EP; i++)  //
						{
							ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[i];
							if ( ed != UX_NULL )
							{
								if ( (ed -> ux_m480_slave_ed_status & UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER) == UX_DCD_M480_SLAVE_ED_STATUS_TRANSFER )
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
				}

        if (IrqSt & HSUSBD_BUSINTSTS_HISPDIF_Msk)
        {
						HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPTKIEN_Msk|HSUSBD_CEPINTEN_SETUPPKIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_HISPDIF_Msk);
        }

        if (IrqSt & HSUSBD_BUSINTSTS_DMADONEIF_Msk)
        {
//	printf("DMA done\n");
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_DMADONEIF_Msk);
						ep_index = HSUSBD->DMACTL & 0x00F;
						ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[ep_index];
						endpoint = ed->ux_m480_slave_ed_endpoint;
						transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;
						transfer_request -> ux_slave_transfer_request_completion_code = UX_SUCCESS;
						transfer_request -> ux_slave_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;
						_ux_utility_semaphore_put(&transfer_request-> ux_slave_transfer_request_semaphore);
            if (!(HSUSBD->DMACTL & HSUSBD_DMACTL_DMARD_Msk))
            {
               HSUSBD_ENABLE_EP_INT(ep_index-1, HSUSBD_EPINTEN_RXPKIEN_Msk);
            }

            if (HSUSBD->DMACTL & HSUSBD_DMACTL_DMARD_Msk)
            {
                if (g_hsusbd_ShortPacket == 1)
                {
                   HSUSBD->EP[ep_index-1].EPRSPCTL = (HSUSBD->EP[ep_index-1].EPRSPCTL & 0x10) | HSUSBD_EP_RSPCTL_SHORTTXEN;    // packet end
                   g_hsusbd_ShortPacket = 0;
//printf("packet end\n");
                }
            }
        }

        if (IrqSt & HSUSBD_BUSINTSTS_PHYCLKVLDIF_Msk)
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_PHYCLKVLDIF_Msk);

        if (IrqSt & HSUSBD_BUSINTSTS_VBUSDETIF_Msk)
        {
            if (HSUSBD_IS_ATTACHED())
            {
                /* USB Plug In */
                HSUSBD_ENABLE_USB();
								device -> ux_slave_device_state = UX_DEVICE_RESET;
            }
            else
            {
                /* USB Un-plug */
                HSUSBD_DISABLE_USB();
            }
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_VBUSDETIF_Msk);
        }
    }

    if (IrqStL & HSUSBD_GINTSTS_CEPIF_Msk)
    {
        IrqSt = HSUSBD->CEPINTSTS & HSUSBD->CEPINTEN;

        if (IrqSt & HSUSBD_CEPINTSTS_SETUPTKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_SETUPTKIF_Msk);
			/*************************** Disable Buffer ***************************/
            HSUSBD_SetEpBufAddr(CEP, 0, 1);
            return(UX_SUCCESS);
        }

        if (IrqSt & HSUSBD_CEPINTSTS_SETUPPKIF_Msk)
        {
//					printf("setup\n");
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_SETUPPKIF_Msk);
						if ( device -> ux_slave_device_state == UX_DEVICE_RESET )
							device -> ux_slave_device_state = UX_DEVICE_ATTACHED;
						ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[0];  //Setup endpoint 0
						endpoint = ed->ux_m480_slave_ed_endpoint;
						transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;
					  (uint16_t)*(transfer_request->ux_slave_transfer_request_setup) = (uint16_t)HSUSBD->SETUP1_0;
						(uint16_t)*(transfer_request->ux_slave_transfer_request_setup+2) = (uint16_t)HSUSBD->SETUP3_2;
            (uint16_t)*(transfer_request->ux_slave_transfer_request_setup+4) = (uint16_t)HSUSBD->SETUP5_4;
					  (uint16_t)*(transfer_request->ux_slave_transfer_request_setup+6) = (uint16_t)HSUSBD->SETUP7_6;
						_ux_dcd_m480_slave_transfer_token_hs(dcd_m480_slave, transfer_request, SETUP_STAGE);
            return(UX_SUCCESS);
        }

        if (IrqSt & HSUSBD_CEPINTSTS_OUTTKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_OUTTKIF_Msk);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            return(UX_SUCCESS);
        }
				/* IN token packet */
        if (IrqSt & HSUSBD_CEPINTSTS_INTKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
            if (!(IrqSt & HSUSBD_CEPINTSTS_STSDONEIF_Msk))
            {
                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_TXPKIF_Msk);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_TXPKIEN_Msk);
 //               HSUSBD_CtrlIn();
							            // control IN
								ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[0];
								endpoint = ed->ux_m480_slave_ed_endpoint;
								transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;
								_ux_dcd_m480_slave_transfer_token_hs(dcd_m480_slave, transfer_request, CONTROL_IN_STAGE);
            }
            else
            {
                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_TXPKIF_Msk);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_TXPKIEN_Msk|HSUSBD_CEPINTEN_STSDONEIEN_Msk);
					/*************************** Disable Buffer ***************************/
								HSUSBD_SetEpBufAddr(CEP, 0, 1);
            }
            return(UX_SUCCESS);
        }

        if (IrqSt & HSUSBD_CEPINTSTS_PINGIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_PINGIF_Msk);
            return(UX_SUCCESS);
        }

        if (IrqSt & HSUSBD_CEPINTSTS_TXPKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
						ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[0];
						endpoint = ed->ux_m480_slave_ed_endpoint;
						transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;
					  if (transfer_request -> ux_slave_transfer_request_in_transfer_length)
            {
                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_INTKIEN_Msk);
            }
            else
            {
							  if ( ed -> ux_m480_slave_ed_control_zero == 1 )
                    HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_ZEROLEN);
                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk|HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            }
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_TXPKIF_Msk);
            return(UX_SUCCESS);
        }

        if (IrqSt & HSUSBD_CEPINTSTS_RXPKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_RXPKIF_Msk);
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk|HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            return(UX_SUCCESS);
        }

        if (IrqSt & HSUSBD_CEPINTSTS_NAKIF_Msk)
        {
					      /*************************** Enable Buffer ***************************/
            HSUSBD_SetEpBufAddr(CEP, 0, dcd_m480_slave -> ux_dcd_m480_ep0_packet_size);
            HSUSBD->CEPINTEN = HSUSBD->CEPINTEN & ~HSUSBD_CEPINTEN_NAKIEN_Msk;

            if(((uint8_t) (HSUSBD->SETUP1_0 & 0xff)) == 0x00 &&
               ((uint8_t) (HSUSBD->SETUP1_0 >> 8) & 0xff) == 0x07 &&
               ((uint16_t) HSUSBD->SETUP3_2) == 0x0100 &&
               ((uint16_t) HSUSBD->SETUP5_4) == 0x0000 &&
               ((uint16_t) HSUSBD->SETUP7_6) == 0x0012)
               HSUSBD_SetStall(0);
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_NAKIF_Msk);
            return(UX_SUCCESS);
        }

        if (IrqSt & HSUSBD_CEPINTSTS_STALLIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STALLIF_Msk);
            return(UX_SUCCESS);
        }

        if (IrqSt & HSUSBD_CEPINTSTS_ERRIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_ERRIF_Msk);
            return(UX_SUCCESS);
        }

        if (IrqSt & HSUSBD_CEPINTSTS_STSDONEIF_Msk)
        {
						ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[0];
						endpoint = ed->ux_m480_slave_ed_endpoint;
						transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;
						_ux_dcd_m480_slave_transfer_token_hs(dcd_m480_slave, transfer_request, UPDATE_DEVICE_STATUS);
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk);
            return(UX_SUCCESS);
        }

        if (IrqSt & HSUSBD_CEPINTSTS_BUFFULLIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_BUFFULLIF_Msk);
            return(UX_SUCCESS);
        }

        if (IrqSt & HSUSBD_CEPINTSTS_BUFEMPTYIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_BUFEMPTYIF_Msk);
            return(UX_SUCCESS);
        }
    }

    while (gintsts_epx)
    {
        if(gintsts_epx & 0x01)
        {
            epxintsts = HSUSBD->EP[ep_hw_index].EPINTSTS & HSUSBD->EP[ep_hw_index].EPINTEN;
             
            ep_logic_index = ep_hw_index+1;
//printf("INT %x, ep=%x\n",epxintsts, ep_hw_index);
            HSUSBD_CLR_EP_INT_FLAG(ep_hw_index, epxintsts);

            /* Buffer Full */
            if (epxintsts & HSUSBD_EPINTSTS_BUFFULLIF_Msk)
            {
            }

            /* Buffer Empty */
            if (epxintsts & HSUSBD_EPINTSTS_BUFEMPTYIF_Msk)
            {
            }

            /* Short Packet Transferred */
            if (epxintsts & HSUSBD_EPINTSTS_SHORTTXIF_Msk)
            {
            }
#if 0
            /* Data Packet Transmitted */
            if (epxintsts & HSUSBD_EPINTSTS_TXPKIF_Msk)
            {
//printf("EP TX\n");
            }

            /* Data Packet Received */
            if (epxintsts & HSUSBD_EPINTSTS_RXPKIF_Msk)
            {
//printf("EP RX\n");
            }
#endif
            /* OUT token packet */
            if ( (epxintsts & HSUSBD_EPINTSTS_OUTTKIF_Msk) || (epxintsts & HSUSBD_EPINTSTS_RXPKIF_Msk) )
            {
//printf("INT OUT\n");
                HSUSBD->EP[ep_hw_index].EPINTEN &= ~(HSUSBD_EPINTEN_RXPKIEN_Msk |HSUSBD_EPINTSTS_OUTTKIF_Msk);
								ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[ep_logic_index];
								endpoint = ed->ux_m480_slave_ed_endpoint;
								if ( (endpoint->ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_INTERRUPT_ENDPOINT )
								{
									transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;
									transfer_request -> ux_slave_transfer_request_completion_code = UX_SUCCESS;
									transfer_request -> ux_slave_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;
									_ux_utility_semaphore_put(&transfer_request-> ux_slave_transfer_request_semaphore);
								}
            }

            /* IN token packet */
            if ( (epxintsts & HSUSBD_EPINTSTS_INTKIF_Msk) || (epxintsts & HSUSBD_EPINTSTS_TXPKIF_Msk))
            {
//printf("INT IN\n");
                HSUSBD->EP[ep_hw_index].EPINTEN &= ~(HSUSBD_EPINTEN_TXPKIEN_Msk |HSUSBD_EPINTSTS_INTKIF_Msk);
								ed = &dcd_m480_slave->ux_dcd_m480_slave_ed[ep_logic_index];
								endpoint = ed->ux_m480_slave_ed_endpoint;
								if ( (endpoint->ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_INTERRUPT_ENDPOINT )
								{
									transfer_request = &endpoint -> ux_slave_endpoint_transfer_request;
									transfer_request -> ux_slave_transfer_request_completion_code = UX_SUCCESS;
									transfer_request -> ux_slave_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;
									_ux_utility_semaphore_put(&transfer_request-> ux_slave_transfer_request_semaphore);
								}
            }

            /* PING packet */
            if (epxintsts & HSUSBD_EPINTSTS_PINGIF_Msk)
            {
							
            }

            /* NAK handshake packet sent to Host */
            if (epxintsts & HSUSBD_EPINTSTS_NAKIF_Msk)
            {

            }

            /* STALL handshake packet sent to Host */
            if (epxintsts & HSUSBD_EPINTSTS_STALLIF_Msk)
            {

            }

            /* NYET handshake packet sent to Host */
            if (epxintsts & HSUSBD_EPINTSTS_NYETIF_Msk)
            {

            }

            /* ERR packet sent to Host */
            if (epxintsts & HSUSBD_EPINTSTS_ERRIF_Msk)
            {
            }

            /* Bulk Out Short Packet Received */
            if (epxintsts & HSUSBD_EPINTSTS_SHORTRXIF_Msk)
            {
            }
        }
        gintsts_epx = gintsts_epx >> 1;
        ep_hw_index++;
    }	
	
    return(UX_SUCCESS);
}

