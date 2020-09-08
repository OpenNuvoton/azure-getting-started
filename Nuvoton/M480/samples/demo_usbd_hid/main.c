/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demo Azure RTOS ThreadX for M480 MCU.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "tx_api.h"
#include "ux_api.h"
#include "ux_system.h"
#include "ux_utility.h"
#include "ux_device_class_hid.h"
#include "ux_dcd_m480_slave.h"
#include "hid_mouse.h"


#define PLL_CLOCK           192000000
#define CRYSTAL_LESS        1
#define TRIM_INIT           (SYS_BASE+0x10C)

void tx_demo_thread_entry(ULONG arg);
VOID tx_demo_hid_instance_activate(VOID *hid_instance);
VOID tx_demo_hid_instance_deactivate(VOID *hid_instance);
VOID  error_handler(void);
UINT demo_thread_hid_callback(UX_SLAVE_CLASS_HID *hid, UX_SLAVE_CLASS_HID_EVENT *hid_event);

UX_SLAVE_CLASS_HID *hid_slave;
UX_SLAVE_CLASS_HID_EVENT  hid_event;
TX_THREAD  tx_demo_thread;

signed char  mouse_table[] = { -16, -16, -16, 0, 16, 16, 16, 0};
UCHAR volatile mouse_idx = 0;
UCHAR volatile move_len;
UCHAR volatile mouse_mode = 1;


/* Port of Azure RTOS ThreadX demo code onto Nuvoton platform
 *
 * https://github.com/azure-rtos/threadx/blob/master/samples/demo_threadx.c
 *
 * 1. Enable stdio output console
 * 2. Add/refine sleep cycles and output messages in threads
 */



int main()
{
	  
     /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("Demo Azure RTOS USBX...\n");
    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}


#define UX_DEMO_STACK_SIZE      1024*5   //4096
#define UX_DEMO_BUFFER_SIZE     2048
#define UX_DEMO_RUN             1
#define UX_DEMO_MEMORY_SIZE     (32*1024)

#define DEMO_BYTE_POOL_SIZE     1024*10
#define DEMO_BLOCK_POOL_SIZE    100
#define DEMO_QUEUE_SIZE         100

TX_BYTE_POOL            byte_pool_0;
TX_BLOCK_POOL           block_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];
UCHAR           				demo_memory[UX_DEMO_MEMORY_SIZE];

/* Define the counters used in the demo application...  */

ULONG                           thread_0_counter;
ULONG                           thread_1_counter;
ULONG                           error_counter;
#define HID_DEVICE_MOUSE_LENGTH		52
#ifdef __ICCARM__
#pragma data_alignment=4
UCHAR HID_MouseReportDescriptor[] =
#else
UCHAR HID_MouseReportDescriptor[] __attribute__((aligned(4))) =
#endif
{
    0x05, 0x01,     /* Usage Page(Generic Desktop Controls) */
    0x09, 0x02,     /* Usage(Mouse) */
    0xA1, 0x01,     /* Collection(Application) */
    0x09, 0x01,     /* Usage(Pointer) */
    0xA1, 0x00,     /* Collection(Physical) */
    0x05, 0x09,     /* Usage Page(Button) */
    0x19, 0x01,     /* Usage Minimum(0x1) */
    0x29, 0x03,     /* Usage Maximum(0x3) */
    0x15, 0x00,     /* Logical Minimum(0x0) */
    0x25, 0x01,     /* Logical Maximum(0x1) */
    0x75, 0x01,     /* Report Size(0x1) */
    0x95, 0x03,     /* Report Count(0x3) */
    0x81, 0x02,     /* Input(3 button bit) */
    0x75, 0x05,     /* Report Size(0x5) */
    0x95, 0x01,     /* Report Count(0x1) */
    0x81, 0x01,     /* Input(5 bit padding) */
    0x05, 0x01,     /* Usage Page(Generic Desktop Controls) */
    0x09, 0x30,     /* Usage(X) */
    0x09, 0x31,     /* Usage(Y) */
    0x09, 0x38,     /* Usage(Wheel) */
    0x15, 0x81,     /* Logical Minimum(0x81)(-127) */
    0x25, 0x7F,     /* Logical Maximum(0x7F)(127) */
    0x75, 0x08,     /* Report Size(0x8) */
    0x95, 0x03,     /* Report Count(0x3) */
    0x81, 0x06,     /* Input(1 byte wheel) */
    0xC0,           /* End Collection */
    0xC0            /* End Collection */
};

#define DEVICE_FRAMEWORK_LENGTH_FULL_SPEED 52
#ifdef __ICCARM__
#pragma data_alignment=4
UCHAR device_framework_full_speed[] = {
#else
UCHAR device_framework_full_speed[] __attribute__((aligned(4))) = {
#endif	

    /* Device descriptor */   // 0x12
     LEN_DEVICE,     /* bLength */
    DESC_DEVICE,    /* bDescriptorType */
    0x10, 0x01,     /* bcdUSB */
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    EP0_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF,
    ((USBD_VID & 0xFF00) >> 8),
    /* idProduct */
    USBD_PID & 0x00FF,
    ((USBD_PID & 0xFF00) >> 8),
    0x00, 0x00,     /* bcdDevice */
    0x01,           /* iManufacture */
    0x02,           /* iProduct */
    0x03,           /* iSerialNumber - no serial */
    0x01,            /* bNumConfigurations */                                 

    /* Configuration descriptor */
     LEN_CONFIG,     /* bLength */
    DESC_CONFIG,    /* bDescriptorType */
    /* wTotalLength */
    LEN_CONFIG_AND_SUBORDINATE & 0x00FF,
    ((LEN_CONFIG_AND_SUBORDINATE & 0xFF00) >> 8),
    0x01,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0x80 | (USBD_SELF_POWERED << 6) | (USBD_REMOTE_WAKEUP << 5),/* bmAttributes */
    USBD_MAX_POWER,         /* MaxPower */

    /* Interface descriptor */
    LEN_INTERFACE,  /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x03,           /* bInterfaceClass */
    0x01,           /* bInterfaceSubClass */
    HID_MOUSE,      /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* HID Descriptor */
    LEN_HID,        /* Size of this descriptor in UINT8s. */
    DESC_HID,       /* HID descriptor type. */
    0x10, 0x01,     /* HID Class Spec. release number. */
    0x00,           /* H/W target country. */
    0x01,           /* Number of HID class descriptors to follow. */
    DESC_HID_RPT,   /* Descriptor type. */
    /* Total length of report descriptor. */
    sizeof(HID_MouseReportDescriptor) & 0x00FF,
    ((sizeof(HID_MouseReportDescriptor) & 0xFF00) >> 8),

    /* EP Descriptor: interrupt in. */
    LEN_ENDPOINT,   /* bLength */
    DESC_ENDPOINT,  /* bDescriptorType */
    (INT_IN_EP_NUM | EP_INPUT), /* bEndpointAddress */
    EP_INT,         /* bmAttributes */
    /* wMaxPacketSize */
    EP2_MAX_PKT_SIZE & 0x00FF,
    ((EP2_MAX_PKT_SIZE & 0xFF00) >> 8),
    HID_DEFAULT_INT_IN_INTERVAL     /* bInterval */
		
    };
    
    
#define DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED 62
#ifdef __ICCARM__
#pragma data_alignment=4
UCHAR device_framework_high_speed[] = { 
#else
UCHAR device_framework_high_speed[] __attribute__((aligned(4))) = { 	
#endif

    /* Device descriptor */
     LEN_DEVICE,     /* bLength */
    DESC_DEVICE,    /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    CEP_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF,
    ((USBD_VID & 0xFF00) >> 8),
    /* idProduct */
    USBD_PID & 0x00FF,
    ((USBD_PID & 0xFF00) >> 8),
    0x00, 0x00,     /* bcdDevice */
    0x01,           /* iManufacture */
    0x02,           /* iProduct */
    0x03,           /* iSerialNumber - no serial */
    0x01,            /* bNumConfigurations */                                

    /* Device qualifier descriptor */
    LEN_QUALIFIER,  /* bLength */
    DESC_QUALIFIER, /* bDescriptorType */
    0x10, 0x01,     /* bcdUSB */
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    CEP_OTHER_MAX_PKT_SIZE, /* bMaxPacketSize0 */
    0x01,           /* bNumConfigurations */
    0x00,

    /* Configuration descriptor */
    LEN_CONFIG,     /* bLength */
    DESC_CONFIG,    /* bDescriptorType */
    /* wTotalLength */
    LEN_CONFIG_AND_SUBORDINATE & 0x00FF,
    ((LEN_CONFIG_AND_SUBORDINATE & 0xFF00) >> 8),
    0x01,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0x80 | (USBD_SELF_POWERED << 6) | (USBD_REMOTE_WAKEUP << 5),/* bmAttributes */
    USBD_MAX_POWER,         /* MaxPower */

    /* Interface descriptor */
    LEN_INTERFACE,  /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x03,           /* bInterfaceClass */
    0x01,           /* bInterfaceSubClass */
    HID_MOUSE,      /* bInterfaceProtocol */
    0x00,           /* iInterface */

   /* HID Descriptor */
    LEN_HID,        /* Size of this descriptor in UINT8s. */
    DESC_HID,       /* HID descriptor type. */
    0x10, 0x01,     /* HID Class Spec. release number. */
    0x00,           /* H/W target country. */
    0x01,           /* Number of HID class descriptors to follow. */
    DESC_HID_RPT,   /* Descriptor type. */
    /* Total length of report descriptor. */
    sizeof(HID_MouseReportDescriptor) & 0x00FF,
    ((sizeof(HID_MouseReportDescriptor) & 0xFF00) >> 8),


    /* EP Descriptor: interrupt in. */
    LEN_ENDPOINT,   /* bLength */
    DESC_ENDPOINT,  /* bDescriptorType */
    (INT_IN_EP_NUM | EP_INPUT), /* bEndpointAddress */
    EP_INT,         /* bmAttributes */
    /* wMaxPacketSize */
    EPA_MAX_PKT_SIZE & 0x00FF,
    ((EPA_MAX_PKT_SIZE & 0xFF00) >> 8),
    HID_DEFAULT_INT_IN_INTERVAL     /* bInterval */
		
    };
    
    /* String Device Framework :
     Byte 0 and 1 : Word containing the language ID : 0x0904 for US
     Byte 2       : Byte containing the index of the descriptor
     Byte 3       : Byte containing the length of the descriptor string
    */
   
#define STRING_FRAMEWORK_LENGTH 38
#ifdef __ICCARM__
#pragma data_alignment=4
UCHAR string_framework[] = { 
#else
UCHAR string_framework[] __attribute__((aligned(4))) = { 
#endif

    /* Manufacturer string descriptor : Index 1 */
        0x09, 0x04, 0x01, 0x0c, 
        0x45, 0x78, 0x70, 0x72,0x65, 0x73, 0x20, 0x4c, 
        0x6f, 0x67, 0x69, 0x63,

    /* Product string descriptor : Index 2 */
        0x09, 0x04, 0x02, 0x0c, 
        0x44, 0x61, 0x74, 0x61, 0x50, 0x75, 0x6d, 0x70, 
        0x44, 0x65, 0x6d, 0x6f,  

    /* Serial Number string descriptor : Index 3 */
        0x09, 0x04, 0x03, 0x04, 
        0x30, 0x30, 0x30, 0x31
    };


    /* Multiple languages are supported on the device, to add
       a language besides English, the unicode language code must
       be appended to the language_id_framework array and the length
       adjusted accordingly. */
#define LANGUAGE_ID_FRAMEWORK_LENGTH 2
#ifdef __ICCARM__
#pragma data_alignment=4
UCHAR language_id_framework[] = { 
#else
UCHAR language_id_framework[] __attribute__((aligned(4))) = { 
#endif

    /* English. */
        0x09, 0x04
    };

	

/* Define what the initial system looks like.  */

void    tx_application_define(void *first_unused_memory)
{
	CHAR   *stack_pointer;
//	CHAR   *memory_pointer;
	UINT   status;
	UX_SLAVE_CLASS_HID_PARAMETER   hid_mouse;
    

  /* Create a byte memory pool from which to allocate the thread stacks.  */
    tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);

    /* Put system definition stuff in here, e.g. thread creates and other assorted
       create information.  */

 	 /* Allocate the stack for thread 0.  */
    tx_byte_allocate(&byte_pool_0, (VOID **) &stack_pointer, UX_DEMO_STACK_SIZE , TX_NO_WAIT);
    
    /* Initialize USBX Memory.  */
    status =  ux_system_initialize(demo_memory, UX_DEMO_MEMORY_SIZE, UX_NULL, 0);


    /* Check for error.  */
    if (status != UX_SUCCESS)
      error_handler();

    /* The code below is required for installing the device portion of USBX */
    status =  _ux_device_stack_initialize(device_framework_high_speed, DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED,
                        device_framework_full_speed, DEVICE_FRAMEWORK_LENGTH_FULL_SPEED,
                        string_framework, STRING_FRAMEWORK_LENGTH,
                        language_id_framework, LANGUAGE_ID_FRAMEWORK_LENGTH, UX_NULL);

    /* Check for error.  */
   if (status != UX_SUCCESS)
      error_handler();

	 hid_slave = UX_NULL;
	/* Set the parameters for callback when insertion/extraction of a HID device. */
	 hid_mouse.ux_slave_class_hid_instance_activate = tx_demo_hid_instance_activate;
	 hid_mouse.ux_slave_class_hid_instance_deactivate = tx_demo_hid_instance_deactivate;
/* Initialize the hid class parameters for the device. */

	 hid_mouse.ux_device_class_hid_parameter_report_address = HID_MouseReportDescriptor;
	 hid_mouse.ux_device_class_hid_parameter_report_length = HID_DEVICE_MOUSE_LENGTH;
	 hid_mouse.ux_device_class_hid_parameter_report_id = UX_TRUE;
	 hid_mouse.ux_device_class_hid_parameter_callback = demo_thread_hid_callback;

/* Initialize the device hid class. The class is connected with interface 0 */
	 status = ux_device_stack_class_register(_ux_system_slave_class_hid_name,
				ux_device_class_hid_entry,1,0, (VOID *)&hid_mouse);

    
    /* Check for error.  */
    if (status != UX_SUCCESS)
        error_handler();

  	
        
    /* Create the main demo thread.  */
    status =  tx_thread_create(&tx_demo_thread, "tx demo slave", tx_demo_thread_entry, 0,  
            stack_pointer , UX_DEMO_STACK_SIZE, 
            20, 20, 1, TX_AUTO_START);
      
    /* Check for error.  */
    if (status != TX_SUCCESS)
        error_handler();

}

void tx_demo_thread_entry(ULONG arg)
{
	UINT   status;

			  /* Initialize the simulated device controller.  */
#if defined(USB_FULL_SPEED)
    status =  _ux_dcd_m480_slave_initialize(UX_FULL_SPEED_DEVICE, device_framework_full_speed[7], HID_MOUSE_USER_DEFINED);
#else
    status =  _ux_dcd_m480_slave_initialize(UX_HIGH_SPEED_DEVICE, device_framework_high_speed[7], HID_MOUSE_USER_DEFINED);
#endif
	/* Check for error.  */
    if (status != TX_SUCCESS)
        error_handler();
	
  while(1)
  {
	  tx_thread_sleep(3);  // 3 click, 1 click = 10 ms
    HID_UpdateMouseData();
  }
	
}

VOID tx_demo_hid_instance_activate(VOID *hid_instance)
{
/* Save the HID instance. */
	hid_slave = (UX_SLAVE_CLASS_HID *) hid_instance;
	printf("Hid instance active %x\n", (UINT)hid_slave);
}

VOID tx_demo_hid_instance_deactivate(VOID *hid_instance)
{
/* Reset the HID instance. */
	hid_slave = UX_NULL;
//	printf("Hid instance Deactive\n");
}

UINT demo_thread_hid_callback(UX_SLAVE_CLASS_HID *hid, UX_SLAVE_CLASS_HID_EVENT *hid_event)
{
	// printf("hid call back\n");
 	return (UX_SUCCESS);
}

VOID  error_handler(void)
{

    /* Increment error counter.  */
    error_counter++;

    while(1)
    {
    
        /* Error - just spin here!  Look at call tree in debugger 
           to see where the error occurred.  */
    }
}

// defined function

void HID_UpdateMouseData(void)
{

	  if ( hid_slave != UX_NULL )
    {
        mouse_mode ^= 1;

        if(mouse_mode)
        {
            if(move_len > 14)
            {
                /* Update new report data */
							/* Insert a key into the keyboard event. Length is fixed to 8. */
								hid_event.ux_device_class_hid_event_buffer[0] = mouse_table[mouse_idx & 0x07];
								hid_event.ux_device_class_hid_event_buffer[1] = mouse_table[(mouse_idx + 2) & 0x07];
								hid_event.ux_device_class_hid_event_buffer[2] = 0;

                mouse_idx++;
                move_len = 0;
            }
        }
        else
        {
						hid_event.ux_device_class_hid_event_buffer[0] = 0;
						hid_event.ux_device_class_hid_event_buffer[1] = 0;
						hid_event.ux_device_class_hid_event_buffer[2] = 0;
        }
        move_len++;
				hid_event.ux_device_class_hid_event_length = 3;
				ux_device_class_hid_event_set(hid_slave, &hid_event);
    }
}





