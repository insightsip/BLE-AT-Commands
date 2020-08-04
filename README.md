# BLE-AT-Commands
Example of AT Command set for BLE

## Overview

The aim of this archive is to show an example of AT Command set for BLE.
The project currently supports ISP1507-AX, ISP1507-AL and ISP1807-LR modules.
Visit https://www.insightsip.com/products/short-range-modules to know more about these modules.

## Environment

The examples are ready to use with the Segger Embedded Studio (SES) IDE.

SES provide a free license for nRF52832 development. Therefore it can be used freely for ISP4520 development.
Licenses can be requested at https://license.segger.com/Nordic.cgi

For more information regarding Segger Embedded Studio, please visit https://www.segger.com/products/development-tools/embedded-studio/

## Supported modules

The following modules are supported:

| Module  | Needed Softdevice | Logical ports |
| :-----------: | :-----------: | :-----------: |
| ISP1507-AL | S112 | RX: P0_08, TX: P0_17, CTS: P0_03, RTS: P0_05, SEL: P0_11 |
| ISP1507-AX | S132 | RX: P0_08, TX: P0_06, CTS: P0_07, RTS: P0_05, SEL: P0_04 |
| ISP1807-LR | S140 | RX: P0_08, TX: P0_06, CTS: P0_07, RTS: P0_05, SEL: P0_04 |

## Changelog

### 2020-08-04, v1.1.0

Added support for ISP1507-AL.

### 2020-06-22, v1.0.0

Initial version.