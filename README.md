# BLE-AT-Commands
Example of AT Command set for BLE

## Overview

The aim of this archive is to show an example of AT Command set for BLE.
The project currently supports ISP1507-AX, ISP1907-LL and ISP1807-LR modules.
Visit https://www.insightsip.com/products/short-range-modules to know more about these modules.

## Environment

The examples are ready to use with the Segger Embedded Studio (SES) IDE.

SES provide a free license for development on nRF devices.
Licenses can be requested at https://license.segger.com/Nordic.cgi

For more information regarding Segger Embedded Studio, please visit https://www.segger.com/products/development-tools/embedded-studio/

## Supported modules

The following modules are supported:

| Module  | Softdevice | RX pin | TX pin | CTS pin | RTS pin | SEL pin |
| :-----------: | :-----------: | :-----------: | :-----------: | :-----------: | :-----------: | :-----------: |
| ISP1907-LL | S112 | P0_08 | P0_17 | P0_03 | P0_05 | P0_11 |
| ISP1507-AX | S132 | P0_08 | P0_06 | P0_07 | P0_05 | P0_04 |
| ISP1807-LR | S140 | P0_08 | P0_06 | P0_07 | P0_05 | P0_04 |

## Changelog

### 2023-04-27, v1.x.x

Removed support for ISP1507-AL.
Added support for ISP1907-LL.
Added support for central role.

### 2020-08-04, v1.1.0

Added support for ISP1507-AL.

### 2020-06-22, v1.0.0

Initial version.