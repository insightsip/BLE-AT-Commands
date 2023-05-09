# BLE-AT-Commands
Example of AT Command set for BLE

## Overview

The aim of this archive is to show an example of AT Command set for BLE.
The project currently supports ISP1507-AX, ISP1907-LL and ISP1807-LR modules.
Visit https://www.insightsip.com/products/short-range-modules to know more about these modules.

An application note is available at:\
https://www.insightsip.com/fichiers_insightsip/pdf/ble/ISP1507/isp_ble_AN200301.pdf

## Environment

The examples are ready to use with the Segger Embedded Studio (SES) IDE.

SES provide a free license for development on nRF devices.
Licenses can be requested at https://license.segger.com/Nordic.cgi

For more information regarding Segger Embedded Studio, please visit https://www.segger.com/products/development-tools/embedded-studio/

## Supported modules

The following modules are supported:

| Module  | Softdevice | RX pin | TX pin | CTS pin | RTS pin | SEL pin |
| :-----------: | :-----------: | :-----------: | :-----------: | :-----------: | :-----------: | :-----------: |
| ISP1907-LL | S112 | P0.08 | P0.17 | P0.03 | P0.05 | P0.11 |
| ISP1507-AX | S132 | P0.08 | P0.06 | P0.07 | P0.05 | P0.04 |
| ISP1807-LR | S140 | P0.08 | P0.06 | P0.07 | P0.05 | P0.04 |

## Known issue

AT+BLEPHY? and AT+BLECONNPARAM? return startup values cannot be trusted.\
This is due to the fact these parameters have to be negotiated each time a new BLE connection is established.
When a connection is establish the host needs to manually call AT+BLEPHY and AT+BLECONNPARAM to set these parameters then it can check the result with AT+BLEPHY? and AT+BLECONNPARAM?

## Changelog

### 2023-05-09, v1.2.0

Removed support for ISP1507-AL.\
Added support for ISP1907-LL.\
Added support for central role.\
Reworked project tree.

### 2020-08-04, v1.1.0

Added support for ISP1507-AL.

### 2020-06-22, v1.0.0

Initial version.
