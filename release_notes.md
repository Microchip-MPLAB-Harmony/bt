# Microchip MPLAB Harmony 3 Release Notes
## Bluetooth Release v3.3.0 (May, 2019)
### ADDITIONS AND UPDATES FOR 3.3.0:

- **Updated Applications**

The following table provides the list of updated applications for the SAM E54:

| Application | Description |
| --- | --- |
| bm64_ble_comm | added E54 project |


### TESTED WITH:

#### Software Dependencies

Before using MPLAB Harmony Audio, ensure that the following are installed:

- [MPLAB X IDE v5.20](https://www.microchip.com/mplab/mplab-x-ide) or later
- [MPLAB XC32 C/C++ Compiler v2.20](https://www.microchip.com/mplab/compilers) or later
- Harmony bt repository, 3.3.0

In order to regenerate source code for any of the applications, you will also need the following to be installed:

- MPLAB Harmony Configurator (MHC) v.3.3.0
- Harmony mplabx_plugin repository, 3.3.0
- Harmony bsp repository, 3.3.0
- Harmony csp repository, 3.3.0
- Harmony core repository, 3.3.0
- Harmony dev_packs repository, 3.3.0
- Harmony gfx repository, 3.3.0 (if building a project with graphics)
- Harmony usb repository, 3.2.2 (if building a project using USB)
- CMSIS-FreeRTOS repository, 10.2.0 if building a FreeRTOS project (from www.github.com/arm-software/cmsis-freertos)

#### Development Kit Support

This release supports applications for the following development kits

| Development Kits |
| --- |
| [SAM E70 Xplained Ultra Evaluation Kit](https://www.microchip.com/developmenttools/ProductDetails/PartNO/DM320113) |
| [SAM E54 Curiosity Ultra Evaluation Kit](https://www.microchip.com/developmenttools/listing/) |

### KNOWN ISSUES

The current known issues are as follows:

* All code is compliant to MISRA C 2012 Mandatory guidelines, except applications using graphics (Rules R.9.1 and R.17.3)
*  To successfully run the BLE apps, you will need an iPhone® 4s or later which supports Bluetooth 4.0. Although the Harmony BLE applications have been designed to also work with smartphones running Android™ as well as  an  iPhone, the  firmware  installed on  the  BM64  module  currently  does  not  support  connections  to  Android smartphones.  If  you  need  the  applications  to  work  with  an  Android  phone, you can download an update from Microchips [BM64 website](https://www.microchip.com/wwwproducts/en/BM64) (
BM64 Software & Tools (DSPKv2.1).

### RELEASE CONTENTS

This topic lists the contents of this release and identifies each module.

#### Description

This table lists the contents of this release, including a brief description, and the release type (Alpha, Beta, Production, or Vendor).


| Folder | Description | Release Type |
| --- | --- | --- |
| bt\apps\data\bm64_ble_comm | BM64 BLE comm application (E70, non-FreeRTOS project) | Production |
| bt\apps\data\bm64_ble_comm | BM64 BLE comm application (all other projects) | Beta |
| bt\apps\utilities\bm64_bootloader | BM64 boot loader | Production |
| bt\driver\BM64 | BM64 Bluetooth Driver | Production |
| bt\templates\bm64 | Bluetooth application template for SAM E70 Xplained Ultra | Production |

## Bluetooth Release v3.2.0 (March, 2019)
### Additions for 3.2.0:

- **Updated Applications**

The following table provides the list of updated applications for the SAM E70:

| Application | Description |
| --- | --- |
| bm64_ble_comm | added graphics and FreeRTOS projects |

## Bluetooth Release v3.1.0 (January, 2019)
### Additions for 3.1.0:

- **New Drivers**

The following table provides the list of new Bluetooth drivers for the SAM E70:

| Driver | Name | Feature |
| --- | --- | --- |
| BM64| BM64 | Added BM64 Bluetooth Driver|

- **New Bluetooth Application Templates**
 
The following table provides the list of new templates for the SAM E70:

| Template | Description |
| --- | --- |
| bm64 | bm64 with E70 Xplained Ultra  |

- **New Applications**

The following table provides the list of new applications for the SAM E70:

| Application | Description |
| --- | --- |
| bm64_ble_comm | sends and receives BLE data to/from an iPhone or Android smartphone |
| bm64_bootloader | updates the EEPROM and firmware of the BM64 from a PC |



