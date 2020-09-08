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
/*    _ux_dcd_m480_slave_initialize                          PORTABLE C    */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the USB simulation slave controller.      */
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
void SYS_Init(ULONG usb_type, ULONG defineddevice);
UINT DeviceClass_Init(ULONG usb_type, ULONG CtrlMaxPktSize, ULONG defineddevice);

extern UINT  _ux_dcd_m480_slave_handler(VOID);
extern UINT  _ux_dcd_m480_slave_handler_hs(VOID);

volatile UINT g_u32UsedBufferSize;

void USBD_IRQHandler(void)
{
	 _ux_dcd_m480_slave_handler();
}

void USBD20_IRQHandler(void)
{
	 _ux_dcd_m480_slave_handler_hs();
}

UINT  _ux_dcd_m480_slave_initialize(ULONG usb_type, ULONG CtrlMaxPktSize, ULONG defineddevice)
{

		UX_SLAVE_DCD            *dcd;
		UX_DCD_M480_SLAVE        *dcd_m480_slave;

		SYS_Init(usb_type, defineddevice);
	
	  UART_Open(UART0, 115200);
			// Set USB type , full speed or high speed
		_ux_system_slave->ux_system_slave_speed = usb_type;
    /* Get the pointer to the DCD.  */
    dcd = &_ux_system_slave -> ux_system_slave_dcd;

    /* The controller initialized here is of Slave simulation type.  */
    dcd -> ux_slave_dcd_controller_type =  UX_DCD_M480_SLAVE_SLAVE_CONTROLLER;
    
    /* Allocate memory for this Slave simulation DCD instance.  */
    dcd_m480_slave =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_DCD_M480_SLAVE));

    /* Check if memory was properly allocated.  */
    if(dcd_m480_slave == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Set the pointer to the Slave simulation DCD.  */
    dcd -> ux_slave_dcd_controller_hardware =  (VOID *) dcd_m480_slave;

    /* Set the generic DCD owner for the Slave simulation DCD.  */
    dcd_m480_slave -> ux_dcd_m480_slave_dcd_owner =  dcd;
		
		dcd_m480_slave -> ux_dcd_m480_slave_defined = defineddevice;
		dcd_m480_slave -> ux_dcd_m480_ep0_packet_size = CtrlMaxPktSize;
    /* Initialize the function collector for this DCD.  */
    dcd -> ux_slave_dcd_function =  _ux_dcd_m480_slave_function;

    /* Set the state of the controller to OPERATIONAL now.  */
    dcd -> ux_slave_dcd_status =  UX_DCD_STATUS_OPERATIONAL;

		g_u32UsedBufferSize = DeviceClass_Init(usb_type, CtrlMaxPktSize, defineddevice);
		
    /* This operation completed with success. */
    return(UX_SUCCESS);
}

#define PLL_CLOCK           192000000
#define CRYSTAL_LESS        1
#define TRIM_INIT           (SYS_BASE+0x10C)


UINT DeviceClass_Init(ULONG usb_type,ULONG CtrlMaxPktSize, ULONG defineddevice)
{
	UINT addr;
	addr = 0;
	if ( usb_type == UX_FULL_SPEED_DEVICE )
	{
	   /* Initial USB engine */
    USBD->ATTR = 0x6D0ul;
    /* Force SE0 */
    USBD_SET_SE0();
	
    /* Init setup packet buffer */
    /* Buffer range for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = addr;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
		addr += 8;
    USBD_SET_EP_BUF_ADDR(EP0, addr);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
		addr += CtrlMaxPktSize;
    USBD_SET_EP_BUF_ADDR(EP1, addr);
		addr += CtrlMaxPktSize;
		/* Initial USB engine */
    USBD->ATTR = 0x6D0ul;
    /* Force SE0 */
    USBD_SET_SE0();
	  
 		   /* Disable software-disconnect function */
    USBD_CLR_SE0();
    USBD->ATTR = 0x7D0ul;

    /* Clear USB-related interrupts before enable interrupt */
    USBD_CLR_INT_FLAG(USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);

    /* Enable USB-related interrupts. */
    USBD_ENABLE_INT(USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);
	  NVIC_EnableIRQ(USBD_IRQn); 
 
	}
	else
	{
	  /* Initial USB engine */
    HSUSBD->PHYCTL |= (HSUSBD_PHYCTL_PHYEN_Msk | HSUSBD_PHYCTL_DPPUEN_Msk);
    /* wait PHY clock ready */
    while (1)
    {
        HSUSBD->EP[0].EPMPS = 0x20ul;
        if (HSUSBD->EP[0].EPMPS == 0x20ul)
        {
            break;
        }
    }
    /* Force SE0, and then clear it to connect*/
    HSUSBD_SET_SE0();
		
	
	    /* Configure USB controller */
    /* Enable USB BUS, CEP and EPA global interrupt */
    HSUSBD_ENABLE_USB_INT(HSUSBD_GINTEN_USBIEN_Msk|HSUSBD_GINTEN_CEPIEN_Msk);
    /* Enable BUS interrupt */
    HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_DMADONEIEN_Msk|HSUSBD_BUSINTEN_RESUMEIEN_Msk|HSUSBD_BUSINTEN_RSTIEN_Msk|HSUSBD_BUSINTEN_VBUSDETIEN_Msk);

    HSUSBD_SET_ADDR(0);

    /*****************************************************/
    /* Control endpoint */
    HSUSBD_SetEpBufAddr(CEP, addr, CtrlMaxPktSize);
		addr += CtrlMaxPktSize;
    HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk|HSUSBD_CEPINTEN_STSDONEIEN_Msk);
		NVIC_EnableIRQ(USBD20_IRQn);

   /* Start transaction */
		HSUSBD_CLR_SE0(); 
	}
	return (addr);
}

void SYS_Init(ULONG usb_type, ULONG defineddevice)
{
	int i;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

	if ( usb_type == UX_FULL_SPEED_DEVICE )  //
	{
    /* Select IP clock source for full speed USB*/
    /* M480LD support crystal-less */
    if (((SYS->CSERVER & SYS_CSERVER_VERSION_Msk) == 0x1) && (CRYSTAL_LESS))
    {
        CLK->PWRCTL |= CLK_PWRCTL_HIRC48MEN_Msk;
        /* Select IP clock source */
        CLK->CLKSEL0 &= ~CLK_CLKSEL0_USBSEL_Msk;
    }
    else
    {
        /* Select IP clock source */
        CLK->CLKSEL0 |= CLK_CLKSEL0_USBSEL_Msk;
        CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USBDIV_Msk) | CLK_CLKDIV0_USB(4);
    }
    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk;
     /* Set PA.12 ~ PA.14 to input mode */
    PA->MODE &= ~(GPIO_MODE_MODE12_Msk | GPIO_MODE_MODE13_Msk | GPIO_MODE_MODE14_Msk);
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk|SYS_GPA_MFPH_PA13MFP_Msk|SYS_GPA_MFPH_PA14MFP_Msk|SYS_GPA_MFPH_PA15MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA12MFP_USB_VBUS|SYS_GPA_MFPH_PA13MFP_USB_D_N|SYS_GPA_MFPH_PA14MFP_USB_D_P|SYS_GPA_MFPH_PA15MFP_USB_OTG_ID);
    /* Enable IP clock */
    CLK_EnableModuleClock(USBD_MODULE);
  }
	else
	{
		SYS->USBPHY &= ~SYS_USBPHY_HSUSBROLE_Msk;            /* select HSUSBD */
    /* Enable USB PHY */
    SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk)) | SYS_USBPHY_HSUSBEN_Msk;
    for (i=0; i<0x1000; i++);      // delay > 10 us
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

    /* Enable IP clock */
    CLK_EnableModuleClock(HSUSBD_MODULE);
		
	}

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    /* Lock protected registers */
//    SYS_LockReg();
}

