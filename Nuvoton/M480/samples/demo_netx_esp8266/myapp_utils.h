/**************************************************************************//**
 * @file     myapp_utils.h.h
 * @version  V1.00
 * @brief    My application utility header file
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __MYAPP_UTILS_H__
#define __MYAPP_UTILS_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>


/* printf-like log output function */
#ifndef MYAPP_CFG_LOG_OUT
#define MYAPP_CFG_LOG_OUT(FMT, ...)                 \
    do {                                            \
        printf(FMT, ## __VA_ARGS__);                \
    } while (0)
#endif

/* Prefixed log output function */
#define MYAPP_LOG_OUT(FMT, ...)                     \
    do {                                            \
        MYAPP_CFG_LOG_OUT("[NXESP]");               \
        MYAPP_CFG_LOG_OUT(FMT, ## __VA_ARGS__);     \
    } while (0)

/* printf message color */
#define MYAPP_MSG_CLR_NONE     "\033[0m"       /* None */
#define MYAPP_MSG_CLR_RED      "\033[1;31m"    /* Red */
#define MYAPP_MSG_CLR_YELLOW   "\033[1;33m"    /* Yellow */
#define MYAPP_MSG_CLR_GREEN    "\033[1;32m"    /* Green */
#define MYAPP_MSG_CLR_CYAN     "\033[1;36m"    /* Cyan */
#define MYAPP_MSG_CLR_PURPLE   "\033[1;35m"    /* Purple */

#define MYAPP_CHK_BOOL(EXPR)                                                \
    do {                                                                    \
        if (!(EXPR)) {                                                      \
            MYAPP_CFG_LOG_OUT(MYAPP_MSG_CLR_YELLOW);                        \
            MYAPP_CFG_LOG_OUT("MYAPP CHECK FAILURE: " #EXPR "\r\n");        \
            MYAPP_CFG_LOG_OUT("FILE: %s+%d\r\n", __FILE__, __LINE__);       \
            MYAPP_CFG_LOG_OUT(MYAPP_MSG_CLR_NONE);                          \
            status = NX_NOT_SUCCESSFUL;                                     \
            goto clean_up;                                                  \
        }                                                                   \
    } while (0)

#define MYAPP_CHK_STATUS(EXPR)                                                      \
    do {                                                                            \
        status = (EXPR);                                                            \
        if (status != NX_SUCCESS) {                                                 \
            MYAPP_CFG_LOG_OUT(MYAPP_MSG_CLR_YELLOW);                                \
            MYAPP_CFG_LOG_OUT("MYAPP CHECK FAILURE: " #EXPR "\r\n");                \
            MYAPP_CFG_LOG_OUT("Expected 0x%08x, Got 0x%08x\r\n", NX_SUCCESS, status);   \
            MYAPP_CFG_LOG_OUT("FILE: %s+%d\r\n", __FILE__, __LINE__);               \
            MYAPP_CFG_LOG_OUT(MYAPP_MSG_CLR_NONE);                                  \
            goto clean_up;                                                          \
        }                                                                           \
    } while (0)

struct NX_PACKET;

/**
 * @brief  Dump NX packet data
 */
UINT myapp_dump_packet(NX_PACKET *packet_ptr);

/**
 * @brief  Dump hex data
 */
void myapp_dump_hex(uint8_t *data, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /*__MYAPP_UTILS_H__*/
