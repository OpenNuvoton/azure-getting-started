;/**************************************************************************/
;/*                                                                        */
;/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
;/*                                                                        */
;/*       This software is licensed under the Microsoft Software License   */
;/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
;/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
;/*       and in the root directory of this software.                      */
;/*                                                                        */
;/**************************************************************************/
;
;
;/**************************************************************************/
;/**************************************************************************/
;/**                                                                       */
;/** ThreadX Component                                                     */
;/**                                                                       */
;/**   Initialize                                                          */
;/**                                                                       */
;/**************************************************************************/
;/**************************************************************************/
;
;#define TX_SOURCE_CODE
;
;
;/* Include necessary system files.  */
;
;#include "tx_api.h"
;#include "tx_initialize.h"
;#include "tx_thread.h"
;#include "tx_timer.h"
;
;
    IMPORT      _tx_thread_system_stack_ptr
    IMPORT      _tx_initialize_unused_memory
    IMPORT      __RAM_segment_used_end__
    IMPORT      _tx_timer_interrupt
    IMPORT      __main
    IMPORT      __Vectors
;
;
SYSTEM_CLOCK        EQU     192000000
SYSTICK_CYCLES      EQU     ((SYSTEM_CLOCK / 100) -1)
                
    ;.text 32
    AREA    |.text|, CODE, READONLY
    ALIGN 4
    ;.syntax unified
    THUMB
;/**************************************************************************/
;/*                                                                        */
;/*  FUNCTION                                               RELEASE        */
;/*                                                                        */
;/*    _tx_initialize_low_level                          Cortex-M4/GNU     */
;/*                                                           6.0          */
;/*  AUTHOR                                                                */
;/*                                                                        */
;/*    William E. Lamie, Microsoft Corporation                             */
;/*                                                                        */
;/*  DESCRIPTION                                                           */
;/*                                                                        */
;/*    This function is responsible for any low-level processor            */
;/*    initialization, including setting up interrupt vectors, setting     */
;/*    up a periodic timer interrupt source, saving the system stack       */
;/*    pointer for use in ISR processing later, and finding the first      */
;/*    available RAM memory address for tx_application_define.             */
;/*                                                                        */
;/*  INPUT                                                                 */
;/*                                                                        */
;/*    None                                                                */
;/*                                                                        */
;/*  OUTPUT                                                                */
;/*                                                                        */
;/*    None                                                                */
;/*                                                                        */
;/*  CALLS                                                                 */
;/*                                                                        */
;/*    None                                                                */
;/*                                                                        */
;/*  CALLED BY                                                             */
;/*                                                                        */
;/*    _tx_initialize_kernel_enter           ThreadX entry function        */
;/*                                                                        */
;/*  RELEASE HISTORY                                                       */
;/*                                                                        */
;/*    DATE              NAME                      DESCRIPTION             */
;/*                                                                        */
;/*  05-19-2020     William E. Lamie         Initial Version 6.0           */
;/*                                                                        */
;/**************************************************************************/
;VOID   _tx_initialize_low_level(VOID)
;{
    EXPORT  _tx_initialize_low_level
    ;.thumb_func
_tx_initialize_low_level
;
;    /* Disable interrupts during ThreadX initialization.  */
;
    CPSID   i
;
;    /* Set base of available memory to end of non-initialised RAM area.  */
;     
    LDR     r0, =_tx_initialize_unused_memory       ; Build address of unused memory pointer
    LDR     r1, =__RAM_segment_used_end__           ; Build first free address   
    ADD     r1, r1, #4                              ; 
    STR     r1, [r0]                                ; Setup first unused memory pointer
;
;    /* Setup Vector Table Offset Register.  */
;    
    MOV     r0, #0xE000E000                         ; Build address of NVIC registers
    LDR     r1, =__Vectors                          ; Pickup address of vector table  
    STR     r1, [r0, #0xD08]                        ; Set vector table address   
;
;    /* Set system stack pointer from vector value.  */
;
    LDR     r0, =_tx_thread_system_stack_ptr        ; Build address of system stack pointer
    LDR     r1, =__Vectors                          ; Pickup address of vector table
    LDR     r1, [r1]                                ; Pickup reset stack pointer
    STR     r1, [r0]                                ; Save system stack pointer
;
;    /* Enable the cycle count register.  */
;
    LDR     r0, =0xE0001000                         ; Build address of DWT register
    LDR     r1, [r0]                                ; Pickup the current value
    ORR     r1, r1, #1                              ; Set the CYCCNTENA bit
    STR     r1, [r0]                                ; Enable the cycle count register 
;
;    /* Configure SysTick for 100Hz clock, or 16384 cycles if no reference.  */
;
    MOV     r0, #0xE000E000                         ; Build address of NVIC registers
    LDR     r1, =SYSTICK_CYCLES
    STR     r1, [r0, #0x14]                         ; Setup SysTick Reload Value
    MOV     r1, #0x7                                ; Build SysTick Control Enable Value
    STR     r1, [r0, #0x10]                         ; Setup SysTick Control
;
;    /* Configure handler priorities.  */
;
    LDR     r1, =0x00000000                         ; Rsrv, UsgF, BusF, MemM
    STR     r1, [r0, #0xD18]                        ; Setup System Handlers 4-7 Priority Registers

    LDR     r1, =0xFF000000                         ; SVCl, Rsrv, Rsrv, Rsrv
    STR     r1, [r0, #0xD1C]                        ; Setup System Handlers 8-11 Priority Registers
                                                    ; Note: SVC must be lowest priority, which is 0xFF

    LDR     r1, =0x40FF0000                         ; SysT, PnSV, Rsrv, DbgM
    STR     r1, [r0, #0xD20]                        ; Setup System Handlers 12-15 Priority Registers
                                                    ; Note: PnSV must be lowest priority, which is 0xFF
        
;
;    /* Return to caller.  */
;    
    BX      lr 
;}
;

;/* Define shells for each of the unused vectors.  */
;

; /* System Tick timer interrupt handler */
    EXPORT  __tx_SysTickHandler
    EXPORT  SysTick_Handler
__tx_SysTickHandler
SysTick_Handler
; VOID TimerInterruptHandler (VOID)
; {
;
    PUSH    {r0, lr}
    IF :DEF: TX_ENABLE_EXECUTION_CHANGE_NOTIFY
    BL      _tx_execution_isr_enter             ; Call the ISR enter function
    ENDIF
    BL      _tx_timer_interrupt
    IF :DEF: TX_ENABLE_EXECUTION_CHANGE_NOTIFY
    BL      _tx_execution_isr_exit              ; Call the ISR exit function
    ENDIF
    POP     {r0, lr}
    BX      LR
; }

    END
