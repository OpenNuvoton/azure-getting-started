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

#include <stdio.h>

#include "nx_api.h"
#include "nx_azure_iot_hub_client.h"
#include "nx_azure_iot_provisioning_client.h"

/* These are sample files, user can build their own certificate and ciphersuites.  */
#include "nx_azure_iot_cert.h"
#include "nx_azure_iot_ciphersuites.h"
#include "sample_config.h"

/* Define Azure RTOS TLS info.  */
static NX_SECURE_X509_CERT root_ca_cert;
static UCHAR nx_azure_iot_tls_metadata_buffer[NX_AZURE_IOT_TLS_METADATA_BUFFER_SIZE];
static ULONG nx_azure_iot_thread_stack[NX_AZURE_IOT_STACK_SIZE / sizeof(ULONG)];

/* Define the prototypes for AZ IoT.  */
static NX_AZURE_IOT                                 nx_azure_iot;
static NX_AZURE_IOT_HUB_CLIENT                      iothub_client;
#ifdef ENABLE_DPS_SAMPLE
static NX_AZURE_IOT_PROVISIONING_CLIENT             prov_client;
#endif /* ENABLE_DPS_SAMPLE */

/* Using X509 certificate authenticate to connect to IoT Hub,
   set the device certificate as your device.  */
#if (USE_DEVICE_CERTIFICATE == 1)
extern const UCHAR sample_device_cert_ptr[];
extern const UINT sample_device_cert_len;
extern const UCHAR sample_device_private_key_ptr[];
extern const UINT sample_device_private_key_len;
NX_SECURE_X509_CERT device_certificate;
#endif /* USE_DEVICE_CERTIFICATE */

/* Define buffer for IoTHub info. */
#ifdef ENABLE_DPS_SAMPLE
static UCHAR sample_iothub_hostname[SAMPLE_MAX_BUFFER];
static UCHAR sample_iothub_device_id[SAMPLE_MAX_BUFFER];
#endif /* ENABLE_DPS_SAMPLE */

/* Define sample threads. */
#ifndef DISABLE_TELEMETRY_SAMPLE
static TX_THREAD sample_telemetry_thread;
static ULONG sample_telemetry_thread_stack[SAMPLE_STACK_SIZE / sizeof(ULONG)];
#endif /* DISABLE_TELEMETRY_SAMPLE */

#ifndef DISABLE_C2D_SAMPLE
static TX_THREAD sample_c2d_thread;
static ULONG sample_c2d_thread_stack[SAMPLE_STACK_SIZE / sizeof(ULONG)];
#endif /* DISABLE_C2D_SAMPLE */

#if !defined(DISABLE_TELEMETRY_SAMPLE) || !defined(DISABLE_C2D_SAMPLE)
/* Define sample properties.  */
static const CHAR *sample_properties[MAX_PROPERTY_COUNT][2] = {{"propertyA", "valueA"},
                                                               {"propertyB", "valueB"}};
#endif /* !defined(DISABLE_TELEMETRY_SAMPLE) && !defined(DISABLE_C2D_SAMPLE) */

#ifndef DISABLE_DIRECT_METHOD_SAMPLE
static CHAR method_response_payload[] = "{\"status\": \"OK\"}";
static TX_THREAD sample_direct_method_thread;
static ULONG sample_direct_method_thread_stack[SAMPLE_STACK_SIZE / sizeof(ULONG)];
#endif /* DISABLE_DIRECT_METHOD_SAMPLE */

#ifndef DISABLE_DEVICE_TWIN_SAMPLE
static CHAR fixed_reported_properties[] = "{\"sample_report\": \"OK\"}";
static TX_THREAD sample_device_twin_thread;
static ULONG sample_device_twin_thread_stack[SAMPLE_STACK_SIZE / sizeof(ULONG)];
#endif /* DISABLE_DEVICE_TWIN_SAMPLE */

void sample_entry(NX_IP *ip_ptr, NX_PACKET_POOL *pool_ptr, NX_DNS *dns_ptr, UINT (*unix_time_callback)(ULONG *unix_time));
#ifdef ENABLE_DPS_SAMPLE
static UINT sample_dps_entry(UCHAR **iothub_hostname, UINT *iothub_hostname_length,
                             UCHAR **iothub_device_id, UINT *iothub_device_id_length);
#endif /* ENABLE_DPS_SAMPLE */
#ifndef DISABLE_TELEMETRY_SAMPLE
static void sample_telemetry_thread_entry(ULONG parameter);
#endif /* DISABLE_TELEMETRY_SAMPLE */

#ifndef DISABLE_C2D_SAMPLE
static void sample_c2d_thread_entry(ULONG parameter);
#endif /* DISABLE_C2D_SAMPLE */

#ifndef DISABLE_DIRECT_METHOD_SAMPLE
static void sample_direct_method_thread_entry(ULONG parameter);
#endif /* DISABLE_DIRECT_METHOD_SAMPLE */

#ifndef DISABLE_DEVICE_TWIN_SAMPLE
static void sample_device_twin_thread_entry(ULONG parameter);
#endif /* DISABLE_DEVICE_TWIN_SAMPLE */

static VOID printf_packet(NX_PACKET *packet_ptr)
{
    while (packet_ptr != NX_NULL)
    {
        printf("%.*s", (INT)(packet_ptr -> nx_packet_append_ptr - packet_ptr -> nx_packet_prepend_ptr),
               (CHAR *)packet_ptr -> nx_packet_prepend_ptr);
        packet_ptr = packet_ptr -> nx_packet_next;
    }
}

static VOID connection_status_callback(NX_AZURE_IOT_HUB_CLIENT *hub_client_ptr, UINT status)
{
    NX_PARAMETER_NOT_USED(hub_client_ptr);
    if (status)
    {
        printf("Disconnected from IoTHub!: error code = 0x%08x\r\n", status);
    }
    else
    {
        printf("Connected to IoTHub.\r\n");
    }
}

static UINT sample_initialize_iothub(NX_AZURE_IOT_HUB_CLIENT *iothub_client_ptr)
{
UINT status;
#ifdef ENABLE_DPS_SAMPLE
UCHAR *iothub_hostname = NX_NULL;
UCHAR *iothub_device_id = NX_NULL;
UINT iothub_hostname_length = 0;
UINT iothub_device_id_length = 0;
#else
UCHAR *iothub_hostname = (UCHAR *)HOST_NAME;
UCHAR *iothub_device_id = (UCHAR *)DEVICE_ID;
UINT iothub_hostname_length = sizeof(HOST_NAME) - 1;
UINT iothub_device_id_length = sizeof(DEVICE_ID) - 1;
#endif /* ENABLE_DPS_SAMPLE */

#ifdef ENABLE_DPS_SAMPLE
    /* Run DPS. */
    if ((status = sample_dps_entry(&iothub_hostname, &iothub_hostname_length,
                                   &iothub_device_id, &iothub_device_id_length)))
    {
        printf("Failed on sample_dps_entry!: error code = 0x%08x\r\n", status);
        return(status);
    }
#endif /* ENABLE_DPS_SAMPLE */

    /* Initialize IoTHub client. */
    if ((status = nx_azure_iot_hub_client_initialize(iothub_client_ptr, &nx_azure_iot,
                                                     iothub_hostname, iothub_hostname_length,
                                                     iothub_device_id, iothub_device_id_length,
                                                     (UCHAR *)MODULE_ID, sizeof(MODULE_ID) - 1,
                                                     _nx_azure_iot_tls_supported_crypto,
                                                     _nx_azure_iot_tls_supported_crypto_size,
                                                     _nx_azure_iot_tls_ciphersuite_map,
                                                     _nx_azure_iot_tls_ciphersuite_map_size,
                                                     nx_azure_iot_tls_metadata_buffer,
                                                     sizeof(nx_azure_iot_tls_metadata_buffer),
                                                     &root_ca_cert)))
    {
        printf("Failed on nx_azure_iot_hub_client_initialize!: error code = 0x%08x\r\n", status);
        return(status);
    }

#if (USE_DEVICE_CERTIFICATE == 1)

    /* Initialize the device certificate.  */
    if ((status = nx_secure_x509_certificate_initialize(&device_certificate,
                                                        (UCHAR *)sample_device_cert_ptr, (USHORT)sample_device_cert_len,
                                                        NX_NULL, 0,
                                                        (UCHAR *)sample_device_private_key_ptr, (USHORT)sample_device_private_key_len,
                                                        DEVICE_KEY_TYPE)))
    {
        printf("Failed on nx_secure_x509_certificate_initialize!: error code = 0x%08x\r\n", status);
    }
    /* Set device certificate.  */
    else if ((status = nx_azure_iot_hub_client_device_cert_set(iothub_client_ptr, &device_certificate)))
    {
        printf("Failed on nx_azure_iot_hub_client_device_cert_set!: error code = 0x%08x\r\n", status);
    }
#else

    /* Set symmetric key.  */
    if ((status = nx_azure_iot_hub_client_symmetric_key_set(iothub_client_ptr,
                                                            (UCHAR *)DEVICE_SYMMETRIC_KEY,
                                                            sizeof(DEVICE_SYMMETRIC_KEY) - 1)))
    {
        printf("Failed on nx_azure_iot_hub_client_symmetric_key_set!\r\n");
    }
#endif /* USE_DEVICE_CERTIFICATE */

    /* Set connection status callback. */
    else if ((status = nx_azure_iot_hub_client_connection_status_callback_set(iothub_client_ptr,
                                                                              connection_status_callback)))
    {
        printf("Failed on connection_status_callback!\r\n");
    }    
#ifndef DISABLE_C2D_SAMPLE
    else if ((status = nx_azure_iot_hub_client_cloud_message_enable(iothub_client_ptr)))
    {
        printf("C2D receive enable failed!: error code = 0x%08x\r\n", status);
    }
#endif /* DISABLE_C2D_SAMPLE */
#ifndef DISABLE_DIRECT_METHOD_SAMPLE
    else if ((status = nx_azure_iot_hub_client_direct_method_enable(iothub_client_ptr)))
    {
        printf("Direct method receive enable failed!: error code = 0x%08x\r\n", status);
    }
#endif /* DISABLE_DIRECT_METHOD_SAMPLE */
#ifndef DISABLE_DEVICE_TWIN_SAMPLE
    else if ((status = nx_azure_iot_hub_client_device_twin_enable(iothub_client_ptr)))
    {
        printf("device twin enabled failed!: error code = 0x%08x\r\n", status);
    }
#endif /* DISABLE_DEVICE_TWIN_SAMPLE */

    if (status)
    {
        nx_azure_iot_hub_client_deinitialize(iothub_client_ptr);
    }
    
    return(status);
}

static void log_callback(az_log_classification classification, UCHAR *msg, UINT msg_len)
{
    if (classification == AZ_LOG_IOT_AZURERTOS)
    {
        printf("%.*s", msg_len, (CHAR *)msg);
    }
}

void sample_entry(NX_IP *ip_ptr, NX_PACKET_POOL *pool_ptr, NX_DNS *dns_ptr, UINT (*unix_time_callback)(ULONG *unix_time))
{
UINT status = 0;
UINT loop = NX_TRUE;

    nx_azure_iot_log_init(log_callback);

    /* Create Azure IoT handler.  */
    if ((status = nx_azure_iot_create(&nx_azure_iot, (UCHAR *)"Azure IoT", ip_ptr, pool_ptr, dns_ptr,
                                      nx_azure_iot_thread_stack, sizeof(nx_azure_iot_thread_stack),
                                      NX_AZURE_IOT_THREAD_PRIORITY, unix_time_callback)))
    {
        printf("Failed on nx_azure_iot_create!: error code = 0x%08x\r\n", status);
        return;
    }

    /* Initialize CA certificate. */
    if ((status = nx_secure_x509_certificate_initialize(&root_ca_cert, (UCHAR *)_nx_azure_iot_root_cert,
                                                        (USHORT)_nx_azure_iot_root_cert_size,
                                                        NX_NULL, 0, NULL, 0, NX_SECURE_X509_KEY_TYPE_NONE)))
    {
        printf("Failed to initialize ROOT CA certificate!: error code = 0x%08x\r\n", status);
        nx_azure_iot_delete(&nx_azure_iot);
        return;
    }
    
    if ((status = sample_initialize_iothub(&iothub_client)))
    {
        printf("Failed to initialize iothub client: error code = 0x%08x\r\n", status);
        nx_azure_iot_delete(&nx_azure_iot);
        return;
    }

    if (nx_azure_iot_hub_client_connect(&iothub_client, NX_TRUE, NX_WAIT_FOREVER))
    {
        printf("Failed on nx_azure_iot_hub_client_connect!\r\n");
        nx_azure_iot_hub_client_deinitialize(&iothub_client);
        nx_azure_iot_delete(&nx_azure_iot);
        return;
    }
    
#ifndef DISABLE_TELEMETRY_SAMPLE
    /* Create Telemetry sample thread. */
    if ((status = tx_thread_create(&sample_telemetry_thread, "Sample Telemetry Thread",
                                   sample_telemetry_thread_entry, 0,
                                   (UCHAR *)sample_telemetry_thread_stack, SAMPLE_STACK_SIZE,
                                   SAMPLE_THREAD_PRIORITY, SAMPLE_THREAD_PRIORITY,
                                   1, TX_AUTO_START)))
    {
        printf("Failed to create telemetry sample thread!: error code = 0x%08x\r\n", status);
    }
#endif /* DISABLE_TELEMETRY_SAMPLE */

#ifndef DISABLE_C2D_SAMPLE
    /* Create C2D sample thread. */
    if ((status = tx_thread_create(&sample_c2d_thread, "Sample C2D Thread",
                                   sample_c2d_thread_entry, 0,
                                   (UCHAR *)sample_c2d_thread_stack, SAMPLE_STACK_SIZE,
                                   SAMPLE_THREAD_PRIORITY, SAMPLE_THREAD_PRIORITY,
                                   1, TX_AUTO_START)))
    {
        printf("Failed to create c2d sample thread!: error code = 0x%08x\r\n", status);
    }
#endif /* DISABLE_C2D_SAMPLE */

#ifndef DISABLE_DIRECT_METHOD_SAMPLE
    /* Create Direct Method sample thread. */
    if ((status = tx_thread_create(&sample_direct_method_thread, "Sample Direct Method Thread",
                                   sample_direct_method_thread_entry, 0,
                                   (UCHAR *)sample_direct_method_thread_stack, SAMPLE_STACK_SIZE,
                                   SAMPLE_THREAD_PRIORITY, SAMPLE_THREAD_PRIORITY,
                                   1, TX_AUTO_START)))
    {
        printf("Failed to create direct method sample thread!: error code = 0x%08x\r\n", status);
    }
#endif /* DISABLE_DIRECT_METHOD_SAMPLE */

#ifndef DISABLE_DEVICE_TWIN_SAMPLE
    /* Create Device twin sample thread. */
    if ((status = tx_thread_create(&sample_device_twin_thread, "Sample Device Twin Thread",
                                   sample_device_twin_thread_entry, 0,
                                   (UCHAR *)sample_device_twin_thread_stack, SAMPLE_STACK_SIZE,
                                   SAMPLE_THREAD_PRIORITY, SAMPLE_THREAD_PRIORITY,
                                   1, TX_AUTO_START)))
    {
        printf("Failed to create device twin sample thread!: error code = 0x%08x\r\n", status);
    }
#endif /* DISABLE_DEVICE_TWIN_SAMPLE */

    /* Simply loop in sample. */
    while (loop)
    {
        tx_thread_sleep(NX_IP_PERIODIC_RATE);
    }
}

#ifdef ENABLE_DPS_SAMPLE
static UINT sample_dps_entry(UCHAR **iothub_hostname, UINT *iothub_hostname_length,
                             UCHAR **iothub_device_id, UINT *iothub_device_id_length)
{
UINT status;

    /* Initialize IoT provisioning client.  */
    if ((status = nx_azure_iot_provisioning_client_initialize(&prov_client, &nx_azure_iot,
                                                              (UCHAR *)ENDPOINT, sizeof(ENDPOINT) - 1,
                                                              (UCHAR *)ID_SCOPE, sizeof(ID_SCOPE) - 1,
                                                              (UCHAR *)REGISTRATION_ID, sizeof(REGISTRATION_ID) - 1,
                                                              _nx_azure_iot_tls_supported_crypto,
                                                              _nx_azure_iot_tls_supported_crypto_size,
                                                              _nx_azure_iot_tls_ciphersuite_map,
                                                              _nx_azure_iot_tls_ciphersuite_map_size,
                                                              nx_azure_iot_tls_metadata_buffer,
                                                              sizeof(nx_azure_iot_tls_metadata_buffer),
                                                              &root_ca_cert)))
    {
        printf("Failed on nx_azure_iot_provisioning_client_initialize!: error code = 0x%08x\r\n", status);
        return(status);
    }

    /* Initialize length of hostname and device ID. */
    *iothub_hostname_length = sizeof(sample_iothub_hostname);
    *iothub_device_id_length = sizeof(sample_iothub_device_id);

#if (USE_DEVICE_CERTIFICATE == 1)

    /* Initialize the device certificate.  */
    if ((status = nx_secure_x509_certificate_initialize(&device_certificate, (UCHAR *)sample_device_cert_ptr, (USHORT)sample_device_cert_len, NX_NULL, 0,
                                                        (UCHAR *)sample_device_private_key_ptr, (USHORT)sample_device_private_key_len, DEVICE_KEY_TYPE)))
    {
        printf("Failed on nx_secure_x509_certificate_initialize!: error code = 0x%08x\r\n", status);
    }

    /* Set device certificate.  */
    else if ((status = nx_azure_iot_provisioning_client_device_cert_set(&prov_client, &device_certificate)))
    {
        printf("Failed on nx_azure_iot_provisioning_client_device_cert_set!: error code = 0x%08x\r\n", status);
    }
#else

    /* Set symmetric key.  */
    if ((status = nx_azure_iot_provisioning_client_symmetric_key_set(&prov_client, (UCHAR *)DEVICE_SYMMETRIC_KEY,
                                                                     sizeof(DEVICE_SYMMETRIC_KEY) - 1)))
    {
        printf("Failed on nx_azure_iot_hub_client_symmetric_key_set!: error code = 0x%08x\r\n", status);
    }
#endif /* USE_DEVICE_CERTIFICATE */

    /* Register device */
    else if ((status = nx_azure_iot_provisioning_client_register(&prov_client, NX_WAIT_FOREVER)))
    {
        printf("Failed on nx_azure_iot_provisioning_client_register!: error code = 0x%08x\r\n", status);
    }

    /* Get Device info */
    else if ((status = nx_azure_iot_provisioning_client_iothub_device_info_get(&prov_client,
                                                                               sample_iothub_hostname, iothub_hostname_length,
                                                                               sample_iothub_device_id, iothub_device_id_length)))
    {
        printf("Failed on nx_azure_iot_provisioning_client_iothub_device_info_get!: error code = 0x%08x\r\n", status);
    }
    else
    {
        *iothub_hostname = sample_iothub_hostname;
        *iothub_device_id = sample_iothub_device_id;
    }

    /* Destroy Provisioning Client.  */
    nx_azure_iot_provisioning_client_deinitialize(&prov_client);

    return(status);
}
#endif /* ENABLE_DPS_SAMPLE */

#ifndef DISABLE_TELEMETRY_SAMPLE
void sample_telemetry_thread_entry(ULONG parameter)
{
UINT i = 0;
UINT status = 0;
CHAR buffer[30];
UINT buffer_length;
UCHAR loop = NX_TRUE;
NX_PACKET *packet_ptr;

    NX_PARAMETER_NOT_USED(parameter);

    /* Loop to send telemetry message.  */
    while (loop)
    {

        /* Create a telemetry message packet. */
        if ((status = nx_azure_iot_hub_client_telemetry_message_create(&iothub_client, &packet_ptr, NX_WAIT_FOREVER)))
        {
            printf("Telemetry message create failed!: error code = 0x%08x\r\n", status);
            break;
        }

        /* Add properties to telemetry message. */
        for (int index = 0; index < MAX_PROPERTY_COUNT; index++)
        {
            if ((status =
                    nx_azure_iot_hub_client_telemetry_property_add(packet_ptr,
                                                                   (UCHAR *)sample_properties[index][0],
                                                                   (USHORT)strlen(sample_properties[index][0]),
                                                                   (UCHAR *)sample_properties[index][1],
                                                                   (USHORT)strlen(sample_properties[index][1]),
                                                                   NX_WAIT_FOREVER)))
            {
                printf("Telemetry property add failed!: error code = 0x%08x\r\n", status);
                break;
            }
        }

        if (status)
        {
            nx_azure_iot_hub_client_telemetry_message_delete(packet_ptr);
            break;
        }

        buffer_length = (UINT)snprintf(buffer, sizeof(buffer), "{\"Message ID\":%u}", i++);
        if (nx_azure_iot_hub_client_telemetry_send(&iothub_client, packet_ptr,
                                                   (UCHAR *)buffer, buffer_length, NX_WAIT_FOREVER))
        {
            printf("Telemetry message send failed!: error code = 0x%08x\r\n", status);
            nx_azure_iot_hub_client_telemetry_message_delete(packet_ptr);
            break;
        }
        printf("Telemetry message send: %s.\r\n", buffer);

        tx_thread_sleep(5 * NX_IP_PERIODIC_RATE);
    }
}
#endif /* DISABLE_TELEMETRY_SAMPLE */

#ifndef DISABLE_C2D_SAMPLE
void sample_c2d_thread_entry(ULONG parameter)
{
UCHAR loop = NX_TRUE;
NX_PACKET *packet_ptr;
UINT status = 0;
USHORT property_buf_size;
UCHAR *property_buf;

    NX_PARAMETER_NOT_USED(parameter);

    /* Loop to receive c2d message.  */
    while (loop)
    {
        if ((status = nx_azure_iot_hub_client_cloud_message_receive(&iothub_client, &packet_ptr, NX_WAIT_FOREVER)))
        {
            printf("C2D receive failed!: error code = 0x%08x\r\n", status);
            break;
        }

        if ((status = nx_azure_iot_hub_client_cloud_message_property_get(&iothub_client, packet_ptr,
                                                                         (UCHAR *)sample_properties[0][0],
                                                                         (USHORT)strlen(sample_properties[0][0]),
                                                                         &property_buf, &property_buf_size)))
        {
            printf("Property [%s] not found: 0x%08x\r\n", sample_properties[0][0], status);
        }
        else
        {
            printf("Receive property: %s = %.*s\r\n", sample_properties[0][0],
                   (INT)property_buf_size, property_buf);
        }

        printf("Receive message:");
        printf_packet(packet_ptr);
        printf("\r\n");

        nx_packet_release(packet_ptr);
    }
}
#endif /* DISABLE_C2D_SAMPLE */

#ifndef DISABLE_DIRECT_METHOD_SAMPLE
void sample_direct_method_thread_entry(ULONG parameter)
{
UCHAR loop = NX_TRUE;
NX_PACKET *packet_ptr;
UINT status = 0;
USHORT method_name_length;
UCHAR *method_name_ptr;
USHORT context_length;
VOID *context_ptr;

    NX_PARAMETER_NOT_USED(parameter);

    /* Loop to receive direct method message.  */
    while (loop)
    {
        if ((status = nx_azure_iot_hub_client_direct_method_message_receive(&iothub_client,
                                                                            &method_name_ptr, &method_name_length,
                                                                            &context_ptr, &context_length,
                                                                            &packet_ptr, NX_WAIT_FOREVER)))
        {
            printf("Direct method receive failed!: error code = 0x%08x\r\n", status);
            break;
        }

        printf("Receive method call: %.*s, with payload:", (INT)method_name_length, (CHAR *)method_name_ptr);
        printf_packet(packet_ptr);
        printf("\r\n");

        if ((status = nx_azure_iot_hub_client_direct_method_message_response(&iothub_client, 200 /* method status */,
                                                                             context_ptr, context_length,
                                                                             (UCHAR *)method_response_payload, sizeof(method_response_payload) - 1,
                                                                             NX_WAIT_FOREVER)))
        {
            printf("Direct method response failed!: error code = 0x%08x\r\n", status);
            nx_packet_release(packet_ptr);
            break;
        }

        nx_packet_release(packet_ptr);
    }
}
#endif /* DISABLE_DIRECT_METHOD_SAMPLE */

#ifndef DISABLE_DEVICE_TWIN_SAMPLE
void sample_device_twin_thread_entry(ULONG parameter)
{
UCHAR loop = NX_TRUE;
NX_PACKET *packet_ptr;
UINT status = 0;
UINT response_status;
UINT request_id;

    NX_PARAMETER_NOT_USED(parameter);

    if ((status = nx_azure_iot_hub_client_device_twin_properties_request(&iothub_client, NX_WAIT_FOREVER)))
    {
        printf("device twin document request failed!: error code = 0x%08x\r\n", status);
        return;
    }

    if ((status = nx_azure_iot_hub_client_device_twin_properties_receive(&iothub_client, &packet_ptr, NX_WAIT_FOREVER)))
    {
        printf("device twin document receive failed!: error code = 0x%08x\r\n", status);
        return;
    }

    printf("Receive twin properties :");
    printf_packet(packet_ptr);
    printf("\r\n");
    nx_packet_release(packet_ptr);

    /* Loop to receive device twin message.  */
    while (loop)
    {
        if ((status = nx_azure_iot_hub_client_device_twin_desired_properties_receive(&iothub_client, &packet_ptr,
                                                                                     NX_WAIT_FOREVER)))
        {
            printf("Receive desired property receive failed!: error code = 0x%08x\r\n", status);
            break;
        }

        printf("Receive desired property call: ");
        printf_packet(packet_ptr);
        printf("\r\n");
        nx_packet_release(packet_ptr);

        if ((status = nx_azure_iot_hub_client_device_twin_reported_properties_send(&iothub_client,
                                                                                   (UCHAR *)fixed_reported_properties, sizeof(fixed_reported_properties) - 1,
                                                                                   &request_id, &response_status,
                                                                                   NX_WAIT_FOREVER)))
        {
            printf("Device twin reported properties failed!: error code = 0x%08x\r\n", status);
            break;
        }

        if ((response_status < 200) || (response_status >= 300))
        {
            printf("device twin report properties failed with code : %d\r\n", response_status);
            break;
        }
    }
}
#endif /* DISABLE_DEVICE_TWIN_SAMPLE */
