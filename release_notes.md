# Microchip MPLAB Harmony 3 Release Notes
## Bluetooth Release v3.2.0 (March, 2019)
### Software Requirements

Before using MPLAB Harmony Bluetooth, ensure that the following are installed:

- [MPLAB X IDE v5.15](https://www.microchip.com/mplab/mplab-x-ide) or later
- [MPLAB XC32 C/C++ Compiler v2.15](https://www.microchip.com/mplab/compilers) or later
- MPLAB Harmony Configurator (MHC) v.3.2.0
- Harmony bt repository, 3.2.0
- Harmony bsp repository, 3.2.0
- Harmony csp repository, 3.2.0
- Harmony core repository, 3.2.0
- Harmony dev_packs repository, 3.2.0
- Harmony gfx repository, 3.2.0 (if building an app with graphics)
- Harmony mplabx_plugin repository, 3.2.0
- CMSIS-FreeRTOS repository, 10.0.1 if building a FreeRTOS project (from www.github.com/arm-software/cmsis-freertos)

### Development Kit Support

- **Development kit support** - This release supports applications for the following development kits:

| Development Kits |
| --- |
| [SAM E70 Xplained Ultra Evaluation Kit](https://www.microchip.com/DevelopmentTools/ProductDetails.aspx?PartNO=ATSAME70-XULT) |

### Additions and Updates for 3.2.0:

- **Updated Applications**

The following table provides the list of updated applications for the SAM E70:

| Application | Description |
| --- | --- |
| bm64_ble_comm | added graphics and FreeRTOS projects |


### Additions for 3.1.0:

- **New driver support**

The following table provides the list of new support for Bluetooth drivers:

| Driver | Name | Feature |
| --- | --- | --- |
| BM64| BM64 | Added BM64 Bluetooth Driver|

- **New Audios Application Templates**
 
The following table provides the list templates for different product families:

| Template | Description |
| --- | --- |
| bm64 | bm64 with E70 Xplained Ultra  |

- **New Applications**

The following table provides the list of new applications for the SAM E70:

| Application | Description |
| --- | --- |
| bm64_ble_comm | sends and receives BLE data to/from an iPhone or Android smartphone |
| bm64_bootloader | updates the EEPROM and firmware of the BM64 from a PC |

### KNOWN ISSUES

The current known issues are as follows:

* BM64 Driver
    - Interactive help using the Show User Manual Entry in the Right-click menu for configuration options provided by this module is not yet available from within the MPLAB Harmony Configurator (MHC).  Please see the *Configuring the Library* section in the help documentation in the doc folder for this Harmony 3 module instead.  Help is available in both CHM and PDF formats.

### RELEASE CONTENTS

This topic lists the contents of this release and identifies each module.

#### Description

This table lists the contents of this release, including a brief description, and the release type (Alpha, Beta, Production, or Vendor).


| Folder | Description | Release Type |
| --- | --- | --- |
| bt\apps\data\bm64_ble_comm | BM64 BLE comm application | Beta |
| bt\apps\utilities\bm64_bootloader | BM64 boot loader | Beta |
| bt\driver\BM64 | BM64 Bluetooth Driver | Beta |
| bt\templates\bm64 | Bluetooth application template for SAM E70 Xplained Ultra | Beta |



