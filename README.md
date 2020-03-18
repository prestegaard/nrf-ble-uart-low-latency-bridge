# nrf-ble-uart-low-latency-bridge
This repository demonstrates how the nRF52-series can be used be as a UART/BLE bridge, with low latency.
The FW provided in this repository is based on the the ble_app_uart and ble_app_uart_c from the nRF SDK v16.0.0.
In order to build the projects, please place this repo one folder level below the sdk_root level. E.g.;
sdk_root/my_projects/nrf-ble-uart-low-latency-bridge/ble_app_uart/
sdk_root/my_projects/nrf-ble-uart-low-latency-bridge/ble_app_uart_c/

The FW is in its current shape only tested on the nRF52840 DK with the S140 SoftDevice, buildt with Segger Embedded Studio.
To use with another configuration, or device, please add the following file to your project:
nrfx_timer.c

Also remember to update you sdk_config.h for your current setup.
A good place to start is to look in the git commits for the peripheral and central side. 
