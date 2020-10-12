# Getting started with Azure RTOS on Nuvoton platform

In this tutorial, you will experience Azure RTOS ported to Nuvoton platform.

## Support targets

MCU         | Board                 | Connectivity      | Note
------------|-----------------------|-------------------|-------------------
M480        | NuMaker-IoT-M487      | ESP8266 WiFi      |

## Support development tools

-   [Keil uVision 5](http://www2.keil.com/mdk5/uvision/)

## Prepare the development environment

In the following, we choose Keil as development IDE and NuMaker-IoT-M487 as target board to experience Azure RTOS.

### Hardware requirements

-   Host computer running Windows 10 (recommended) 
-   [Nuvoton NuMaker-IoT-M487 board](https://direct.nuvoton.com/tw/numaker-iot-m487)
-   USB 2.0 Micro B connector

### Software requirements

-   [Git](https://git-scm.com/downloads)
-   [Keil uVision 5](http://www2.keil.com/mdk5/uvision/)
-   [Keil Nu-Link debugger driver](https://github.com/OpenNuvoton/Nuvoton_Tools)


### Hardware setup

1.  On NuMaker-IoT-M487 board, switch Nu-Link to Debug mode and enable USB VCOM.
1.  Connect NuMaker-IoT-M487 board to host through USB.
1.  Configure host terminal program with **115200/8-N-1**.

### Install Nuvoton-branched git repository

In git-bash, clone the `azure-rtos/getting-started` repository, branched for Nuvoton port:

```sh
git clone -b nuvoton_azure_rtos https://github.com/cyliangtw/azure-getting-started
```

Navigate into `azure-getting-started` directory:

```sh
cd azure-getting-started
```

Install its dependent git submodules:

```sh
git submodule update --init --recursive
```

## Examples

In the following, we walk through several Azure RTOS examples ported on NuMaker-IoT-M487 board.
They are under `M480/samples` directory.

### demo_threadx

The sample is clone of [threadx sample](https://github.com/azure-rtos/threadx/tree/master/samples) and ported to NuMaker-IoT-M487 board.
It shows usage of threadx components, including thread, queue, semaphore, mutex, event flag, and pool.

1.  Open `demo_threadx/keil/demo_threadx.uvproj` Keil project file.
1.  In Keil, build program through the menu **Project** > **Build Target**.
1.  In Keil, download program through the menu **Flash** > **Download**.
1.  Press **RESET** button on NuMaker-IoT-M487 board to start running.
1.  Check source code to learn threadx components listed above.

### demo_netx_esp8266

This sample shows networking with [adapted netxduo for WiFi](https://github.com/azure-rtos/getting-started/tree/master/STMicroelectronics/STM32L4_L4%2B/lib/netxduo) and ESP8266 WiFi module.
It sends HTTP request to HTTP server and receives HTTP response from it.

1.  Open `demo_netx_esp8266/keil/demo_netx_esp8266.uvproj` Keil project file.
1.  Change **SSID**/**PASSWORD** in `network_config.h` to match your real network configuration.
1.  In Keil, build program through the menu **Project** > **Build Target**.
1.  In Keil, download program through the menu **Flash** > **Download**.
1.  Press **RESET** button on NuMaker-IoT-M487 board to start running.
1.  You would see message from host termiinal program:

```
[NXESP]AT Version: 1.7.0
[NXESP]SDK Version: 3.0.0
[NXESP]Configure to Station mode
[NXESP]Enable Station mode DHCP client
[NXESP]Connect to AP with SSID=HUAWEI-F2C9,PASSWORD=********
[NXESP]SoftAP IP: 0.0.0.0
[NXESP]SoftAP MAC:  0. 0. 0. 0. 0. 0
[NXESP]Station IP: 192.168.8.101
[NXESP]Station MAC: a4.cf.12.b7.82.3b
[NXESP]DNS server address: 208.67.222.222
www.ifconfig.io IP address: 172.67.189.102
[ESP][WARN]Ignore TCP link local port(57803)
HTTP request:
GET /method HTTP/1.1
Host: ifconfig.io
Connection: close

HTTP response:
HTTP/1.1 200 OK
Date: Fri, 14 Aug 2020 08:53:59 GMT
Content-Type: text/plain; charset=utf-8
Content-Length: 4
Connection: close
Set-Cookie: __cfduid=d016aef040987dc00a29f6f27290ffc271597395239; expires=Sun, 13-Sep-20 08:53:59 GMT; path=/; domain=.ifconfig.io; HttpOnly; SameSite=Lax
CF-Cache-Status: DYNAMIC
cf-request-id: 048dc61b36000004e77f244200000001
Server: cloudflare
CF-RAY: 5c2972d85c1f04e7-LAX
alt-svc: h3-27=":443"; ma=86400, h3-28=":443"; ma=86400, h3-29=":443"; ma=86400

GET
Closed by remote
```

### demo_azure_iot

The sample is clone of [azure-iot-preview sample](https://github.com/azure-rtos/azure-iot-preview/tree/master/samples/sample_azure_iot_embedded_sdk) and ported to NuMaker-IoT-M487 board.
It shows connection with Azure IoT Hub.
Being clone of azure-iot-preview sample, all its guide can apply to this sample.

1.  Open `demo_azure_iot/keil/demo_azure_iot.uvproj` Keil project file.
1.  Change **SSID**/**PASSWORD** in `network_config.h` to match your real network configuration.
1.  If you don't have an Azure subscription, [create one](https://azure.microsoft.com/free) for free before you begin.
1.  To register new device in IoT Hub, follow [Register a new device in the IoT hub](https://docs.microsoft.com/en-us/azure/iot-hub/iot-hub-create-through-portal#register-a-new-device-in-the-iot-hub) to create device with Symmetric key authentication.
    After device's registration is complete, copy the connection string for the device with following format `HostName=<>;DeviceId=<>;SharedAccessKey=<>`.
    Add following macros to `sample_config.h`:

    ```C
    #define HOST_NAME                                   "<Hostname from connection string>"
    #define DEVICE_ID                                   "<DeviceId from connection string>"
    #define DEVICE_SYMMETRIC_KEY                        "<SharedAccessKey from connection string>"
    ```

1.  In Keil, build program through the menu **Project** > **Build Target**.
1.  In Keil, download program through the menu **Flash** > **Download**.
1.  Press **RESET** button on NuMaker-IoT-M487 board to start running.
1.  You would see message from host termiinal program:

    ```
    Starting Azure thread

    [NXESP]AT Version: 1.7.0
    [NXESP]SDK Version: 3.0.0
    [NXESP]Configure to Station mode
    [NXESP]Enable Station mode DHCP client
    [NXESP]Connect to AP with SSID=HUAWEI-F2C9,PASSWORD=********
    [NXESP]SoftAP IP: 0.0.0.0
    [NXESP]SoftAP MAC:  0. 0. 0. 0. 0. 0
    [NXESP]Station IP: 192.168.8.101
    [NXESP]Station MAC: a4.cf.12.b7.82.3b
    [NXESP]DNS server address: 208.67.222.222
    Initializing SNTP client
            SNTP IP address: 220.132.17.177
            SNTP time update: Aud 14, 2020 9:52:53.230 UTC
    SUCCESS: SNTP initialized
    
    [ESP][WARN]Ignore TCP link local port(56551)
    Connected to IoTHub.
    Telemetry message send: {"Message ID":0}.
    Receive twin properties :{"desired":{"$version":1},"reported":{"$version":1}}
    Telemetry message send: {"Message ID":1}.
    Telemetry message send: {"Message ID":2}.
    Telemetry message send: {"Message ID":3}.
    ```
1.  By default, the following features are enabled:
    -   [Send device-to-cloud message](https://docs.microsoft.com/en-us/azure/iot-hub/iot-hub-devguide-messages-d2c)
    -   [Receive cloud-to-device messages](https://docs.microsoft.com/en-us/azure/iot-hub/iot-hub-devguide-messages-c2d)
    -   [Device Twins](https://docs.microsoft.com/en-us/azure/iot-hub/iot-hub-devguide-device-twins)
    -   [Direct Methods](https://docs.microsoft.com/en-us/azure/iot-hub/iot-hub-devguide-direct-methods)

    For example, to test **Direct Methods**, in Azure Portal, navigate to your connected device's **Direct method** tab.
    Set **Method Name** to `my-direct-method-001` and **Payload** to `1234`.
    Press **Invoke Method**, and you would see message from host termiinal program:
    
    <pre>
    Telemetry message send: {"Message ID":436}.
    <b>
    Receive method call: my-direct-method-001, with payload:1234
    </b>
    Telemetry message send: {"Message ID":437}.
    </pre>


### demo_usbd_hid

This sample shows usb mouse thru usb hid.

1.  Open `demo_usbd_hid/keil/demo_usbd_hid.uvproj` Keil project file.

### demo_usbd_storage

This sample shows usb disk thru usb mass torage.

1.  Open `demo_usbd_storage/keil/demo_usbd_storage.uvproj` Keil project file.
    
## Known issue or limitation

1.  To use WiFi modulee with its own network stack, the Azure RTOS team has officially given an [adapted netxduo for WiFi](https://github.com/azure-rtos/getting-started/tree/master/STMicroelectronics/STM32L4_L4%2B/lib/netxduo).
    This version is also ported to Nuvoton platform to enable e.g. NuMaker-IoT-M487 on-board ESP8266 WiFi module.
    User needs to follow the initialization sequence given in e.g. `demo_netx_esp8266` to enable networking with adapted netxduo for WiFi and ESP8266 WiFi module.
1.  Not support Azure IoT service device provisioning service (DPS).

## Reference

1.  [Azure RTOS in GitHub](https://github.com/azure-rtos)
1.  [Azure RTOS ThreadX documentation](https://docs.microsoft.com/en-us/azure/rtos/threadx/)
1.  [Azure RTOS NetX Duo documentation](https://docs.microsoft.com/en-us/azure/rtos/netx-duo/)
