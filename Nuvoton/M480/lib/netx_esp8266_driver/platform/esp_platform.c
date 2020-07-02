/**************************************************************************//**
 * @file     esp_platform.c
 * @version  V1.00
 * @brief    M480 series ESP8266 WiFi platform driver source file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "stdlib.h"

/* ESP includes */
#include "esp_cfg.h"
#include "platform/esp_platform.h"
#include "platform/esp_log.h"

/* Tx includes */
#include "tx_api.h"

/* Expire callback for platform timer */
static VOID timer_expire(ULONG ctx);

/* Saved interrupt-lock state */
static UINT old_interruptlock_posture;

uint64_t ESP_Plat_GetTime_MS(void)
{
    static bool timer_inited = false;       // Platform timer initialized?
    static TX_TIMER timer;                  // Control block of platform timer
    static uint64_t timer_value_ms = 0;     // Value of platform timer in ms
    UINT rc;

    if (!timer_inited) {
        ESP_LOG_DBG("Create platform timer...\r\n");
        rc = tx_timer_create(&timer,                    // Timer control block
                             "ESP platform timer",      // Timer name
                             timer_expire,              // Expire function
                             (ULONG)&timer_value_ms,    // Input to expire function
                             1,                         // Initial ticks
                             1,                         // Reschedule ticks
                             TX_AUTO_ACTIVATE);         // Auto-activate. Needn't tx_timer_activate(...) later
        if (rc == TX_SUCCESS) {
            timer_inited = true;
            ESP_LOG_DBG("Create platform timer...OK\r\n");
        } else {
            ESP_LOG_CRIT("Create platform timer...ERROR: %d\r\n", rc);
        }
    }

    return timer_value_ms;
}

void ESP_Plat_Wait_MS(uint32_t ms)
{
    uint64_t cur_ms = ESP_Plat_GetTime_MS();
    uint64_t end_ms = cur_ms + ms;
    while (ms) {
        /* Round up to ticks */
        uint32_t ticks = (ms * TX_TIMER_TICKS_PER_SECOND + 999) / 1000;
        tx_thread_sleep(ticks);

        /* Extra sleep needed on premature wake-up */
        cur_ms = ESP_Plat_GetTime_MS();
        ms = (end_ms > cur_ms) ? (end_ms - cur_ms) : 0;
    }
}

void ESP_Plat_EnterCriticalSection(void)
{
    /* Lockout interrupts */
    old_interruptlock_posture = tx_interrupt_control(TX_INT_DISABLE);
}

void ESP_Plat_ExitCriticalSection(void)
{
    /* Restore previous interrupt lockout posture. */
    tx_interrupt_control(old_interruptlock_posture);
}

static VOID timer_expire(ULONG ctx)
{
    uint64_t *timer_value_ms = (uint64_t *) ctx;
    *timer_value_ms += 1000 / TX_TIMER_TICKS_PER_SECOND;
}
