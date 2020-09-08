/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demo Azure RTOS ThreadX for M480 MCU.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "fx_api.h"
#include "tx_api.h"
#include "ux_api.h"
#include "ux_system.h"
#include "ux_utility.h"
#include "ux_device_class_storage.h"
#include "ux_dcd_m480_slave.h"
#include "massstorage.h"


#define PLL_CLOCK           192000000
#define CRYSTAL_LESS        1
#define TRIM_INIT           (SYS_BASE+0x10C)

#define RAM_DISK_SIZE				32*1024
#define RAM_DISK_LAST_LBA		(RAM_DISK_SIZE)/512 -1


void tx_demo_thread_entry(ULONG arg);

UINT media_read(VOID *storage, ULONG lun, UCHAR *data_pointer, ULONG number_blocks, ULONG lba, ULONG *media_status);
UINT media_write(VOID *storage, ULONG lun, UCHAR *data_pointer,ULONG number_blocks, ULONG lba, ULONG *media_status);
ULONG media_status(VOID *storage, ULONG lun, ULONG media_id, ULONG *media_status);

/* Define RAM device driver entry.  */
VOID _fx_ram_driver(FX_MEDIA *media_ptr);

VOID  error_handler(void);

TX_THREAD  tx_demo_thread;
UX_SLAVE_CLASS_STORAGE_PARAMETER		storage_parameter;

FX_MEDIA   ram_disk;
UCHAR      ram_disk_memory[RAM_DISK_SIZE];
UCHAR			 buffer[512];

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


#define UX_DEMO_STACK_SIZE      1024*6   //4096
#define UX_DEMO_BUFFER_SIZE     2048
#define UX_DEMO_RUN             1
#define UX_DEMO_MEMORY_SIZE     (32*1024)

#define DEMO_BYTE_POOL_SIZE     1024*12
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

#define DEVICE_FRAMEWORK_LENGTH_FULL_SPEED 50
#ifdef __ICCARM__
#pragma data_alignment=4
UCHAR device_framework_full_speed[] = {
#else
UCHAR device_framework_full_speed[] __attribute__((aligned(4))) = {
#endif	

    /* Device descriptor */   // 0x12
    LEN_DEVICE,     /* bLength */
    DESC_DEVICE,    /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
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
	
    LEN_CONFIG,                                         // bLength
    DESC_CONFIG,                                        // bDescriptorType
    (LEN_CONFIG+LEN_INTERFACE+LEN_ENDPOINT*2), 0x00,    // wTotalLength
    0x01,                                               // bNumInterfaces
    0x01,                                               // bConfigurationValue
    0x00,                                               // iConfiguration
    0xC0,                                               // bmAttributes
    0x32,                                               // MaxPower

    /* Interface descriptor */
    LEN_INTERFACE,  // bLength
    DESC_INTERFACE, // bDescriptorType
    0x00,           // bInterfaceNumber
    0x00,           // bAlternateSetting
    0x02,           // bNumEndpoints
    0x08,           // bInterfaceClass
    0x05,           // bInterfaceSubClass
    0x50,           // bInterfaceProtocol
    0x00,           // iInterface

    /* EP Descriptor: bulk in. */
    LEN_ENDPOINT,           // bLength
    DESC_ENDPOINT,          // bDescriptorType
    (BULK_IN_EP_NUM | EP_INPUT), // bEndpointAddress
    EP_BULK,                // bmAttributes
    EP2_MAX_PKT_SIZE, 0x00,  // wMaxPacketSize
    0x00,                   // bInterval
		
		/* EP Descriptor: bulk out */
	  LEN_ENDPOINT,           // bLength
    DESC_ENDPOINT,          // bDescriptorType
    BULK_OUT_EP_NUM,                   // bEndpointAddress
    EP_BULK,                // bmAttributes
    EP3_MAX_PKT_SIZE, 0x00,  // wMaxPacketSize
    0x00                    // bInterval	
    };
    
    
#define DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED 60
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
    0x00, 0x02,     /* bcdUSB */
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
    (LEN_CONFIG+LEN_INTERFACE+LEN_ENDPOINT*2), 0x00,
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
    0x02,           /* bNumEndpoints */
    0x08,           /* bInterfaceClass */
    0x05,           /* bInterfaceSubClass */
    0x50,           /* bInterfaceProtocol */
    0x00,           /* iInterface */


    /* EP Descriptor: bulk in. */
    LEN_ENDPOINT,   /* bLength */
    DESC_ENDPOINT,  /* bDescriptorType */
    (BULK_IN_EP_NUM | EP_INPUT),    /* bEndpointAddress */
    EP_BULK,        /* bmAttributes */
    /* wMaxPacketSize */
    EPA_MAX_PKT_SIZE & 0x00FF,
    ((EPA_MAX_PKT_SIZE & 0xFF00) >> 8),
    0x00,           /* bInterval */
		
		/* EP Descriptor: bulk out */
    LEN_ENDPOINT,   /* bLength */
    DESC_ENDPOINT,  /* bDescriptorType */
    (BULK_OUT_EP_NUM | EP_OUTPUT),  /* bEndpointAddress */
    EP_BULK,        /* bmAttributes */
    /* wMaxPacketSize */
    EPB_MAX_PKT_SIZE & 0x00FF,
    ((EPB_MAX_PKT_SIZE & 0xFF00) >> 8),
    0x00        /* bInterval */
		
    };
    
    /* String Device Framework :
     Byte 0 and 1 : Word containing the language ID : 0x0904 for US
     Byte 2       : Byte containing the index of the descriptor
     Byte 3       : Byte containing the length of the descriptor string
    */
   
#define STRING_FRAMEWORK_LENGTH 41
#ifdef __ICCARM__
#pragma data_alignment=4
UCHAR string_framework[] = { 
#else
UCHAR string_framework[] __attribute__((aligned(4))) = { 
#endif

    /* Manufacturer string descriptor : Index 1 */
        0x09, 0x04, 0x01, 0x07, 
				'N', 'u', 'v', 'o', 't', 'o', 'n', 

    /* Product string descriptor : Index 2 */
        0x09, 0x04, 0x02, 0x0A, 
	      'U', 'S', 'B', ' ', 'D', 'e', 'v', 'i', 'c', 'e', 


    /* Serial Number string descriptor : Index 3 */
        0x09, 0x04, 0x03, 0x0C, 
				'A', '0', '0', '0', '2', '0', '2', '0', '1', '1', '0', '4',
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

#ifdef __ICCARM__
#pragma data_alignment=4
UCHAR storage_vendor_id[] = {
#else
UCHAR storage_vendor_id[] __attribute__((aligned(4))) = {
#endif
	'N', 'u', 'v', 'o', 't', 'o', 'n', ' '
};

#ifdef __ICCARM__
#pragma data_alignment=4
UCHAR storage_product_id[] = {
#else
UCHAR storage_product_id[] __attribute__((aligned(4))) = {	
#endif
   'U', 'S', 'B', ' ', 'M', 'a', 's', 's', ' ', 'S', 't', 'o', 'r', 'a', 'g', 'e'	
};

#ifdef __ICCARM__
#pragma data_alignment=4
UCHAR storage_product_rev[] = {
#else
UCHAR storage_product_rev[] __attribute__((aligned(4))) = {
#endif
	 '1', '.', '0', '0'
};

/* Define what the initial system looks like.  */

void    tx_application_define(void *first_unused_memory)
{
	CHAR   *stack_pointer;
//	CHAR   *memory_pointer;
	UINT   status;
    

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

		_ux_utility_memory_set(ram_disk_memory, 0, RAM_DISK_SIZE);
		
		/* Initialize FileX */
		fx_system_initialize();
		
    /* The code below is required for installing the device portion of USBX */
    status =  _ux_device_stack_initialize(device_framework_high_speed, DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED,
                        device_framework_full_speed, DEVICE_FRAMEWORK_LENGTH_FULL_SPEED,
                        string_framework, STRING_FRAMEWORK_LENGTH,
                        language_id_framework, LANGUAGE_ID_FRAMEWORK_LENGTH, UX_NULL);

    /* Check for error.  */
   if (status != UX_SUCCESS)
      error_handler();

		storage_parameter.ux_slave_class_storage_parameter_vendor_id
		= storage_vendor_id;
		storage_parameter.ux_slave_class_storage_parameter_product_id
		= storage_product_id;
		storage_parameter.ux_slave_class_storage_parameter_product_rev
		= storage_product_rev;
	/* Store the number of LUN in this device storage instance: single LUN. */
		storage_parameter.ux_slave_class_storage_parameter_number_lun = 1;
/* Initialize the storage class parameters for reading/writing to the Flash Disk. */
		storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_last_lba = RAM_DISK_LAST_LBA;
		storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_block_length = 512;
		storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_type = 0;
		storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_removable_flag = 0x80;
		storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_read_only_flag
		= UX_FALSE;
		storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_read
		= media_read;
		storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_write
		= media_write;
		storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_status
		= media_status;
/* Initialize the device storage class. The class is connected with interface 0 */
		status = ux_device_stack_class_register(_ux_system_slave_class_storage_name,
				ux_device_class_storage_entry, 1, 0, (VOID *)&storage_parameter);

    
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
	
		   /* Format the ram drive. */
		status =  fx_media_format(&ram_disk, _fx_ram_driver, ram_disk_memory, buffer, 512, "NUVOTON RAM DISK", 1,32, 0, RAM_DISK_SIZE / 512, 512, 4, 1, 1);
/* Check the media format status.  */
    if (status != FX_SUCCESS)
    {
				printf("fail to file format\n");
        /* Error opening media.  Return to caller.  */
        return;
    }

		   /* Open the ram_disk.  */
    status =  fx_media_open(&ram_disk, "NUVOTON RAM DISK", _fx_ram_driver, ram_disk_memory, buffer, 512);

    /* Check the media open status.  */
    if (status != FX_SUCCESS)
    {
				printf("fail to open file\n");
        /* Error opening media.  Return to caller.  */
        return;
    }
			  /* Initialize the simulated device controller.  */
#if defined(USB_FULL_SPEED)
    status =  _ux_dcd_m480_slave_initialize(UX_FULL_SPEED_DEVICE, device_framework_full_speed[7], 0);
#else
    status =  _ux_dcd_m480_slave_initialize(UX_HIGH_SPEED_DEVICE, device_framework_high_speed[7], 0);
#endif
	/* Check for error.  */
    if (status != TX_SUCCESS)
        error_handler();
	  
		
	tx_thread_sleep(10);  // 10 click, 1 click = 10 ms	
  while(1)
  {

  }
	
}

UINT media_read(VOID *storage, ULONG lun, UCHAR *data_pointer, ULONG number_blocks, ULONG lba, ULONG *media_status)
{
	UINT status= UX_SUCCESS;
	while (number_blocks--)
	{
		status = fx_media_read(&ram_disk,lba,data_pointer);
		data_pointer += 512;
		lba++;
	}
	printf("media read\n");
  return(status);	
}

UINT media_write(VOID *storage, ULONG lun, UCHAR *data_pointer,ULONG number_blocks, ULONG lba, ULONG *media_status)
{
	UINT status= UX_SUCCESS;
	while (number_blocks--)
	{
		status = fx_media_write(&ram_disk,lba,data_pointer);
		data_pointer += 512;
		lba++;
	}
	printf("media write\n");
  return(status);
}

ULONG media_status(VOID *storage, ULONG lun, ULONG media_id, ULONG *media_status)
{
//	printf("media status\n");
  return(UX_SUCCESS);
}



VOID  error_handler(void)
{

    /* Increment error counter.  */
    error_counter++;
		printf("error\n");
    while(1)
    {
    
        /* Error - just spin here!  Look at call tree in debugger 
           to see where the error occurred.  */
    }
}






