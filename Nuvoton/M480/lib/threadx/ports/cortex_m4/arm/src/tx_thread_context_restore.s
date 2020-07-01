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
;/**   Thread                                                              */
;/**                                                                       */
;/**************************************************************************/
;/**************************************************************************/
;
;
;#define TX_SOURCE_CODE
;
;
;/* Include necessary system files.  */
;
;#include "tx_api.h"
;#include "tx_thread.h"
;#include "tx_timer.h"
;
;
    IMPORT      _tx_thread_system_state
    IMPORT      _tx_thread_current_ptr
    IMPORT      _tx_thread_system_stack_ptr
    IMPORT      _tx_thread_execute_ptr
    IMPORT      _tx_timer_time_slice
    IMPORT      _tx_thread_schedule
    IMPORT      _tx_thread_preempt_disable
#ifdef TX_ENABLE_EXECUTION_CHANGE_NOTIFY
    IMPORT      _tx_execution_isr_exit
#endif
;
;
    AREA    |.text|, CODE, READONLY
    ALIGN 4
    THUMB
;/**************************************************************************/
;/*                                                                        */
;/*  FUNCTION                                               RELEASE        */
;/*                                                                        */
;/*    _tx_thread_context_restore                        Cortex-M4/GNU     */
;/*                                                           6.0          */
;/*  AUTHOR                                                                */
;/*                                                                        */
;/*    William E. Lamie, Microsoft Corporation                             */
;/*                                                                        */
;/*  DESCRIPTION                                                           */
;/*                                                                        */
;/*    This function restores the interrupt context if it is processing a  */
;/*    nested interrupt.  If not, it returns to the interrupt thread if no */
;/*    preemption is necessary.  Otherwise, if preemption is necessary or  */
;/*    if no thread was running, the function returns to the scheduler.    */
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
;/*    _tx_thread_schedule                   Thread scheduling routine     */
;/*                                                                        */
;/*  CALLED BY                                                             */
;/*                                                                        */
;/*    ISRs                                  Interrupt Service Routines    */
;/*                                                                        */
;/*  RELEASE HISTORY                                                       */
;/*                                                                        */
;/*    DATE              NAME                      DESCRIPTION             */
;/*                                                                        */
;/*  05-19-2020     William E. Lamie         Initial Version 6.0           */
;/*                                                                        */
;/**************************************************************************/
;VOID   _tx_thread_context_restore(VOID)
;{
    EXPORT  _tx_thread_context_restore
_tx_thread_context_restore
;
; /* Not needed for this port - just return!  */
    BX      lr
;}

    END
