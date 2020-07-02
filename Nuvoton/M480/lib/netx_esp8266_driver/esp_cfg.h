/**************************************************************************//**
 * @file     esp_cfg.h
 * @version  V1.00
 * @brief    ESP8266 WiFi driver configuration header file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __ESP_CFG_H__
#define __ESP_CFG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Customize ESP configurations here */
#define ESP_CFG_HW_RST                  1
#define ESP_CFG_SERIAL_BAUDRATE         115200
#define ESP_CFG_SERIAL_FC               1
#define ESP_CFG_TCP_PASSIVE_RECV_MODE   1
#define ESP_CFG_SEND_MAXSIZE            1200
#define ESP_CFG_MULTICONN               1
#define ESP_CFG_LOG                     1
#define ESP_CFG_ASSERT                  1
#define ESP_CFG_LOGLVL_MAX              ESP_LOGLVL_INFO

/* Hardware reset via reset pin or power pin */
#ifndef ESP_CFG_HW_RST
#define ESP_CFG_HW_RST              1
#endif

/* Serial baudrate */
#ifndef ESP_CFG_SERIAL_BAUDRATE
#define ESP_CFG_SERIAL_BAUDRATE     115200
#endif

/* Serial RTS/CTS flow control to avoid UART FIFO overrun */
#ifndef ESP_CFG_SERIAL_FC
#define ESP_CFG_SERIAL_FC           1
#endif

/* Max count of sockets supported */
#ifndef ESP_CFG_SOCKET_COUNT
#define ESP_CFG_SOCKET_COUNT        5
#endif

/* TCP passive receive mode to avoid packet loss due to host MCU OOM */
#ifndef ESP_CFG_TCP_PASSIVE_RECV_MODE
#define ESP_CFG_TCP_PASSIVE_RECV_MODE    1
#endif

/* Max payload size */
#ifndef ESP_CFG_PAYLOAD_MAXSIZE
#define ESP_CFG_PAYLOAD_MAXSIZE     2048
#endif

/* Max data size per send */
#ifndef ESP_CFG_SEND_MAXSIZE
#define ESP_CFG_SEND_MAXSIZE        ESP_CFG_PAYLOAD_MAXSIZE
#endif

/* RX buffer size */
#ifndef ESP_CFG_RXBUF_SIZE
#define ESP_CFG_RXBUF_SIZE          256
#endif

/* Command/data buffer size */
#ifndef ESP_CFG_CMDDATABUF_SIZE
#define ESP_CFG_CMDDATABUF_SIZE     3072
#endif

/* IPD data buffer size */
#ifndef ESP_CFG_IPDBUF_SIZE
#define ESP_CFG_IPDBUF_SIZE         ESP_CFG_PAYLOAD_MAXSIZE
#endif

/* Multiple connections */
#ifndef ESP_CFG_MULTICONN
#define ESP_CFG_MULTICONN           1
#endif

/* Maximum SSID length */
#ifndef ESP_CFG_SSID_SIZE
#define ESP_CFG_SSID_SIZE           100
#endif

/* Enable log */
#ifndef ESP_CFG_LOG
#define ESP_CFG_LOG             0
#endif

/* Enable assert */
#ifndef ESP_CFG_ASSERT
#define ESP_CFG_ASSERT          0
#endif

/* Max log level, the larger, the more verbose. */
#ifndef ESP_CFG_LOGLVL_MAX
#define ESP_CFG_LOGLVL_MAX      ESP_LOGLVL_NONE
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ESP_CFG_H__ */
