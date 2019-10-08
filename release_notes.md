# Microchip MPLAB Harmony 3 Release Notes
## Bluetooth Release v3.4.0 (October, 2019)

### ADDITIONS AND UPDATES FOR 3.4.0:

- **New Applications/Projects**

The following table provides the list of new applications/projects:

| Application | Description | BTADK | MZ/C2 | D21 | E54 CULT | E54 XPRO | E70 | BM64 | BM71 |
| --- | --- | --- | --- | --- | --- |
| bm64_a2dp_hfp | New application | x | x | | | | | x | |
| ble_comm | Added variations for BM64 | x | x | | x | | x | x | |
| ble_comm | New variations for BM71 |  | x | x | x | x | | | x |
| bm64_bootloader | Added variations for BM64 | x | x | | | | | x | |

where:

| |  Development Kit |
| --- | --- | 
|**BTADK**| [PIC32 Bluetooth Audio Development Kit](https://www.microchip.com/developmenttools/ProductDetails/PartNO/DV320032) |
|**MZ/C2**| [Curiosity PIC32MZ EF Development Board 2.0](https://www.microchip.com/developmenttools/ProductDetails/PartNO/DM320209) |
|**D21**| [SAM D21 Xplained Pro Evaluation Kit](https://www.microchip.com/developmenttools/ProductDetails/PartNO/ATSAMD21-XPRO) |
|**E54 CULT**| [SAM E54 Curiosity Ultra Development Board](https://www.microchip.com/developmenttools/ProductDetails/PartNO/DM320210) |
|**E54 XPRO**| [SAM E54 Xplained Pro Evaluation Kit](https://www.microchip.com/developmenttools/ProductDetails/PartNO/ATSAME54-XPRO) |
|**E70**| [SAM E70 Xplained Ultra Evaluation Kit](https://www.microchip.com/developmenttools/ProductDetails/PartNO/DM320113) |
|**BM64**| [BM64 Bluetooth Radio Daughter Board](https://www.microchip.com/developmenttools/ProductDetails/AC320032-3) |
|**BM71**| [BM71 XPRO Board](https://www.microchip.com/developmenttools/ProductDetails/PartNO/DM164146)

- **New Drivers**

The following table provides the list of new Bluetooth drivers for the SAM E70:

| Driver | Description | BTADK | MZ/C2 | D21 | E54 CULT | E54 XPRO | E70 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| BM71 | BM71 Driver |   | x | x | x | x |  x |

- **New Bluetooth Application Templates**

The following table provides the list new Bluetooth templates:

| Template | Description | BTADK | MZ/C2 | D21 | E54 CULT | E54 XPRO |
| --- | --- | --- | --- | --- | 
| BM64 | BLE (no audio) | x | x | | x | |
| BM64 I2S | with audio | x | x | |  | |
| BM71 | BLE (no audio) | | x | x | x | x |


### TESTED WITH:

#### Software Dependencies

Before using MPLAB Harmony Bluetooth, ensure that the following are installed:

- [MPLAB X IDE v5.25](https://www.microchip.com/mplab/mplab-x-ide) or later
- [MPLAB XC32 C/C++ Compiler v2.30](https://www.microchip.com/mplab/compilers) or later
- Harmony bt repository, 3.4.0

In order to regenerate source code for any of the applications, you will also need the following to be installed:

- MPLAB Harmony Configurator (MHC) v.3.3.0
- Harmony mplabx_plugin repository, 3.3.0
- Harmony audio repository, 3.4.0 
- Harmony bsp repository, 3.5.0
- Harmony core repository, 3.5.0
- Harmony csp repository, 3.5.0
- Harmony dev_packs repository, 3.5.0
- Harmony gfx repository, 3.4.0 (if building a project with graphics)
- Harmony usb repository, 3.3.0 (if building a project using USB)
- CMSIS-FreeRTOS repository, 10.2.0 if building a FreeRTOS project (from www.github.com/arm-software/cmsis-freertos)

#### Development Kit Support

This release supports applications for the following development kits

| Development Kits |
| --- |
| [PIC32 Bluetooth Audio Development Kit](https://www.microchip.com/developmenttools/ProductDetails/PartNO/DV320032) |
| [Curiosity PIC32MZ EF Development Board 2.0](https://www.microchip.com/developmenttools/ProductDetails/PartNO/DM320209) |
| [SAM D21 Xplained Pro Evaluation Kit](https://www.microchip.com/developmenttools/ProductDetails/PartNO/ATSAMD21-XPRO) |
| [SAM E54 Curiosity Ultra Development Board](https://www.microchip.com/developmenttools/ProductDetails/PartNO/DM320210) |
| [SAM E54 Xplained Pro Evaluation Kit](https://www.microchip.com/developmenttools/ProductDetails/PartNO/ATSAME54-XPRO) |
|  [SAM E70 Xplained Ultra Evaluation Kit](https://www.microchip.com/developmenttools/ProductDetails/PartNO/DM320113) |

### KNOWN ISSUES

The current known issues are as follows:

* All code is compliant to MISRA C 2012 Mandatory guidelines, except applications using graphics (Rules R.9.1 and R.17.3)
*  To successfully run the BLE apps, you will need an iPhone® 4s or later which supports Bluetooth 4.0. Although the Harmony BLE applications have been designed to also work with smartphones running Android™ as well as  an  iPhone, the  firmware  installed on  the  BM64  module  currently  does  not  support  connections  to  Android smartphones.  If  you  need  the  applications  to  work  with  an  Android  phone, you can download an update from Microchips [BM64 website](https://www.microchip.com/wwwproducts/en/BM64) (
BM64 Software & Tools (DSPKv2.1).
* If you are building a Bluetooth Audio application (like the a2dp_hfp demonstration), you must invoke the audio template (to instantiate the codec and I2S driver) first, before invoking the Bluetooth template.  After doing so, click on the *+* sign in the I2S driver to create a second instance of the driver before you invoke the Bluetooth template.

### RELEASE CONTENTS

This topic lists the contents of this release and identifies each module.

#### Description

This table lists the contents of this release, including a brief description, and the release type (Alpha, Beta, or Production).


| Folder | Description | Release Type |
| --- | --- | --- |
| bt\apps\data\bm64_a2dp_hfp | BM64 A2Dp/HFP application (all projects) | Beta |
| bt\apps\data\ble_comm | BLE comm application (all E70 projects) | Production |
| bt\apps\data\ble_comm | BLE comm application (all other projects) | Beta |
| bt\apps\utilities\bm64_bootloader | BM64 boot loader (E70 project) | Production |
| bt\apps\utilities\bm64_bootloader | BM64 boot loader (all other projects) | Beta |
| bt\driver\BM64 | BM64 Bluetooth Driver | Production |
| bt\driver\BM71 | BM71 Bluetooth Driver | Beta |
| bt\templates\BM64 | Bluetooth application template for SAM E70 XULT | Production |
| bt\templates\BM64 | Bluetooth application template for all other boards | Beta |
| bt\templates\BM64 I2S | Bluetooth application template for SAM E70 XULT | Production |
| bt\templates\BM71 | Bluetooth application template (all boards) | Beta |

## Bluetooth Release v3.3.0 (May, 2019)
### Updates for 3.3.0:

- **Updated Applications**

The following table provides the list of updated applications for the SAM E54:

| Application | Description |
| --- | --- |
| bm64_ble_comm | added E54 project |

## Bluetooth Release v3.2.0 (March, 2019)
### Updates for 3.2.0:

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



