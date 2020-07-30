# Azure RTOS sample projects for Nuvoton's M480 series platform

This directory contains sample projects for Nuvoton's M480 series platform.

## Download git submodules

The sample projects need other git submodules (threadx, netxduo, etc.).
Download them first:

```sh
$ git submodule init
$ git submodule update
```
    
## Rebuild libraries

The `lib` directory contains prebuilt libraries with different toolchains.
They are needed by sample projects.

To rebuild them with related toolchains, follow the steps:

### Rebuild keil/ThreadX_Library.lib

1.  Navigate to `core/lib/threadx/ports/cortex_m4/keil/example_build` directory

1.  Open Keil project `ThreadX_Library.uvproj` and build it.
    
1.  Overwrite `lib/keil/ThreadX_Library.lib` with new one.

### Rebuild lib/keil/netxduo_wifi.lib

1.  Navigate to `M480/lib/netxduo_wifi/ports/cortex_m4/keil` directory.

1.  Open Keil project `netxduo_wifi.uvprojx` and build it.

1.  Overwrite `lib/keil/etxduo_wifi.lib` with new one.

### Rebuild lib/keil/netx_esp8266_driver.lib

1.  Navigate to `M480/lib/netx_esp8266_driver/keil` directory.

1.  Open Keil project `netx_esp8266_driver.uvprojx` and build it.

1.  Overwrite `lib/keil/netx_esp8266_driver.lib` with new one.

## Build sample projects

### demo_threadx

The sample is clone of threadx's and ported to Nuvoton platform.
It shows usage of threadx components, including thread, queue, semaphore, mutex, event flag, and pool.

Open `demo_threadx/keil/demo_threadx.uvproj` and follow the normal development flow.

### demo_netx_esp8266

This sample shows networking with reduced netx/ESP8266 WiFi module.

Open `demo_netx_esp8266/keil/demo_netx_esp8266.uvproj` and follow the normal development flow.

**NOTE**: Change **SSID**/**PASSWORD** in `myapp_config.h` to match real network configuration.

## Monitor the application

Otherwise mentioned, configure your terminal program with **115200/8-N-1**, and you could see execution results.
