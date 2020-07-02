/**************************************************************************//**
 * @file     esp8266_ll.c
 * @version  V1.00
 * @brief    M480 series ESP8266 WiFi low-level driver source file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "stdlib.h"

/* ESP includes */
#include "esp_cfg.h"
#include "driver/esp_ll.h"
#include "platform/esp_platform.h"
#include "platform/esp_log.h"
#include "buffer/esp_buffer.h"

/* BSP includes */
#include "NuMicro.h"

/**
 * @brief   Struct of RX buffer
 * @note    Must be compatible with ESP_Buf_t
 */
typedef struct {
    uint16_t ridx;
    uint16_t widx;
    uint16_t size;
    uint8_t buf[ESP_CFG_RXBUF_SIZE];
} ESP_RxBuf_t;

/** ESP WiFi low-level driver control block
 */
typedef struct {
    ESP_RxBuf_t rxbuf;
    bool rx_irq_enabled;
} ESP_LL_t ;

static ESP_LL_t esp_ll_inst;

/* Enable ESP UART Rx Irq */
static void enable_rx_irq(void);

/* Disable ESP UART Rx Irq */
static void disable_rx_irq(void);

bool ESP_LL_Init(void)
{
    /* Reset ESP UART Rx buffer */
    ESP_Buf_Setup((ESP_Buf_t *) &esp_ll_inst.rxbuf, sizeof(esp_ll_inst.rxbuf.buf) / sizeof(esp_ll_inst.rxbuf.buf[0]));
    
    CLK_EnableModuleClock(UART1_MODULE);
    /* Select UART1 clock source is HXT */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HXT, CLK_CLKDIV0_UART1(1));

    /* Set PH multi-function pins for UART1 RXD, TXD */
    SYS->GPH_MFPH &= ~(SYS_GPH_MFPH_PH8MFP_Msk | SYS_GPH_MFPH_PH9MFP_Msk);
    SYS->GPH_MFPH |= (SYS_GPH_MFPH_PH8MFP_UART1_TXD | SYS_GPH_MFPH_PH9MFP_UART1_RXD);
    SYS_ResetModule(UART1_RST);
    ESP_Plat_Wait_MS(10);
#if ESP_CFG_SERIAL_FC
    /* Set PB multi-function pins for UART1 RTS and CTS */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB8MFP_Msk | SYS_GPB_MFPH_PB9MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB8MFP_UART1_nRTS | SYS_GPB_MFPH_PB9MFP_UART1_nCTS);
#endif

    /* Configure UART */
    UART_Open(UART1, 115200);
    /* Set RX Trigger Level = 8 */
    UART1->FIFO = (UART1->FIFO &~ UART_FIFO_RFITL_Msk) | UART_FIFO_RFITL_8BYTES;
    /* Set Timeout time 0x3E bit-time */
    UART_SetTimeoutCnt(UART1, 0x3E);
    /* enable uart */
    esp_ll_inst.rx_irq_enabled = true;
    UART_EnableInt(UART1, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_TOCNTEN_Msk);
    NVIC_EnableIRQ(UART1_IRQn);

    return true;
}

void ESP_LL_ResetModem(uint32_t assert_delay, uint32_t deassert_delay)
{
    /* H/W reset */
    GPIO_SetMode(PH, BIT3, GPIO_MODE_OUTPUT);
    PH3 = 0;
    ESP_Plat_Wait_MS(assert_delay);
    PH3 = 1;
    ESP_Plat_Wait_MS(deassert_delay);
}

void ESP_LL_ConfigBaudrate(uint32_t baudrate)
{
    UART_Open(UART1, baudrate);
}

void ESP_LL_EnableFlowControl(bool enable)
{
    /* Configure RTS */
    /* Before configuring RTSACTLV, disable TX/RX. */
    UART1->FUNCSEL |= UART_FUNCSEL_TXRXDIS_Msk;
    while (UART1->FIFOSTS & UART_FIFOSTS_TXRXACT_Msk);
    /* nRTS pin output is low level active */
    UART1->MODEM |= UART_MODEM_RTSACTLV_Msk;
    /* After configuring RTSACTLV, re-enable TX/RX. */
    UART1->FUNCSEL &= ~UART_FUNCSEL_TXRXDIS_Msk;
    /* Configure RTS trigger level to 8 bytes */
    UART1->FIFO = (UART1->FIFO & ~UART_FIFO_RTSTRGLV_Msk) | UART_FIFO_RTSTRGLV_8BYTES;

    if (enable) {
        /* Enable RTS */
        UART1->INTEN |= UART_INTEN_ATORTSEN_Msk;
    } else {
        /* Disable RTS */
        UART1->INTEN &= ~UART_INTEN_ATORTSEN_Msk;
        /* Drive nRTS pin output to low-active. Allow the peer to be able to send data
         * even though its CTS is still enabled. */
        UART1->MODEM &= ~UART_MODEM_RTS_Msk;
    }
        
    /* Configure CTS */
    /* Before configuring CTSACTLV, disable TX/RX. */
    UART1->FUNCSEL |= UART_FUNCSEL_TXRXDIS_Msk;
    while (UART1->FIFOSTS & UART_FIFOSTS_TXRXACT_Msk);
    /* nCTS pin input is low level active */
    UART1->MODEMSTS |= UART_MODEMSTS_CTSACTLV_Msk;
    /* Added in M480. After configuring CTSACTLV, re-enable TX/RX. */
    UART1->FUNCSEL &= ~UART_FUNCSEL_TXRXDIS_Msk;

    if (enable)  {
        /* Enable CTS */
        UART1->INTEN |= UART_INTEN_ATOCTSEN_Msk;
    } else {
        /* Disable CTS */
        UART1->INTEN &= ~UART_INTEN_ATOCTSEN_Msk;
    }
}

void ESP_LL_RxStrm_Purge(void)
{
    while (ESP_LL_RxStrm_Readable()) {
        ESP_LL_RxStrm_GetChar();
    }

    ESP_Plat_EnterCriticalSection();
    /* Clear Rx FIFO overflow */
    UART_ClearIntFlag(UART1, UART_INTSTS_BUFERRINT_Msk);
    ESP_Plat_ExitCriticalSection();
}

bool ESP_LL_RxStrm_Readable(void)
{
    bool readable;

    ESP_Plat_EnterCriticalSection();
    readable = !ESP_Buf_IsEmpty((ESP_Buf_t *) &esp_ll_inst.rxbuf);
    ESP_Plat_ExitCriticalSection();

    return readable;
}

uint8_t ESP_LL_RxStrm_GetChar(void)
{
    uint8_t data = (uint8_t) -1;

    ESP_Plat_EnterCriticalSection();
    if (ESP_Buf_IsEmpty((ESP_Buf_t *) &esp_ll_inst.rxbuf)) {
        ESP_LOG_CRIT("ESP Rx buffer underflow\r\n");
    } else {
        data = ESP_Buf_Pop((ESP_Buf_t *) &esp_ll_inst.rxbuf);
        /* Enable back Rx Irq when Rx buffer is not full */
        if (!esp_ll_inst.rx_irq_enabled && !ESP_Buf_IsFull((ESP_Buf_t *) &esp_ll_inst.rxbuf)) {
            enable_rx_irq();
        }
    }
    ESP_Plat_ExitCriticalSection();

    return data;
}

bool ESP_LL_TxStrm_Writable(void)
{
    bool writable;

    ESP_Plat_EnterCriticalSection();
    writable = !UART_IS_TX_FULL(UART1);
    ESP_Plat_ExitCriticalSection();

    return writable;
}

void ESP_LL_TxStrm_PutChar(uint8_t c)
{
    ESP_Plat_EnterCriticalSection();
    UART_WRITE(UART1, c);
    ESP_Plat_ExitCriticalSection();
}

void enable_rx_irq(void)
{
    esp_ll_inst.rx_irq_enabled = true;
    UART_EnableInt(UART1, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_TOCNTEN_Msk);
}

void disable_rx_irq(void)
{
    UART_DisableInt(UART1, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_TOCNTEN_Msk);
    esp_ll_inst.rx_irq_enabled = false;
}

/* UART Irq handler for Rx */
void UART1_IRQHandler(void)
{
    uint8_t data;

    /* Read into Rx buffer as possible */
    while (!ESP_Buf_IsFull((ESP_Buf_t *) &esp_ll_inst.rxbuf) && !UART_GET_RX_EMPTY(UART1)) {
        data = UART_READ(UART1);
        ESP_Buf_Push((ESP_Buf_t *) &esp_ll_inst.rxbuf, data);
    }

    /* Disable Rx Irq when Rx buffer is full */
    if (esp_ll_inst.rx_irq_enabled && ESP_Buf_IsFull((ESP_Buf_t *) &esp_ll_inst.rxbuf)) {
        disable_rx_irq();
    }
}
