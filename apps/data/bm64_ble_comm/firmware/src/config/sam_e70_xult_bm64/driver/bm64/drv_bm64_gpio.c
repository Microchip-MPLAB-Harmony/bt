/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64_gpio.c

  Summary:
   BM64 Bluetooth Static Driver source file for GPIO functions.

  Description:
    This file is the implementation of the internal functions of the BM64
    driver related to the GPIO pins controlling the BM64.
 
*******************************************************************************/

/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "driver/bm64/drv_bm64_gpio.h"
#include "peripheral/pio/plib_pio.h"

// modified for bt audio dev kit

// this is now done in system_config.h/sys_ports_static.c using SYS_PORT_B_TRIS and SYS_PORT_G_TRIS  #defines
//void GPIO_Initialize ( void )
//{  
//    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0,PORT_CHANNEL_G,PORTS_BIT_POS_15);    // BT RST - RG15
//    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0,PORT_CHANNEL_B,PORTS_BIT_POS_2);    // MFB    - RB2
//}

//also have in system_config.h (defined by BSP)
/*** Functions for BM64_MFB pin ***/
//#define BM64_MFBToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_2)
//#define BM64_MFBOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_2)
//#define BM64_MFBOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_2)
//#define BM64_MFBStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_2)
//#define BM64_MFBStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_2, Value)

void DRV_BM64_MFB_SetHigh(void)
{   
    BM64_MFB_Set();
}

void DRV_BM64_MFB_SetLow(void)
{    
    BM64_MFB_Clear();     
}

void DRV_BM64_MFB_Toggle(void)
{
    BM64_MFB_Toggle();   
}

uint32_t DRV_BM64_MFB_GetValue(void)
{
    return BM64_MFB_Get();
}

// and in bsp.c:
//void BSP_BluetoothPinStateSet(BSP_BT_STATE state)/
//{
//    PLIB_PORTS_PinWrite (PORTS_ID_0 , PORT_CHANNEL_G , PORTS_BIT_POS_15, state );
//}
//void BSP_BluetoothPinSetHigh(void)
//{   
//    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15);   
//}
//void BSP_BluetoothPinSetLow(void)
//{    
//    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15);    
//}
//void BSP_BluetoothPinToggle(void)
//{
//    PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15);    
//}
//uint32_t BSP_BluetoothPinGetValue(void)
//{
//    return PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15);
//}

void DRV_BM64_RESET_SetHigh(void)
{
    BM64_STBYRST_Set();
}

void DRV_BM64_RESET_SetLow(void)
{   
    BM64_STBYRST_Clear();    
}

void DRV_BM64_RESET_Toggle(void)
{
    BM64_STBYRST_Toggle();   
}

uint32_t DRV_BM64_RESET_GetValue(void)
{
    return BM64_STBYRST_Get();
}


/*******************************************************************************
 End of File
 */
