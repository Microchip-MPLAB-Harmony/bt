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

void DRV_BM64_RESET_SetHigh(void)
{
    STBYRST_Set();
}

void DRV_BM64_RESET_SetLow(void)
{   
    STBYRST_Clear();    
}

void DRV_BM64_RESET_Toggle(void)
{
    STBYRST_Toggle();   
}

uint32_t DRV_BM64_RESET_GetValue(void)
{
    return STBYRST_Get();
}


/*******************************************************************************
 End of File
 */
