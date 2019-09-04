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
// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "bt/driver/bm64/drv_bm64_gpio.h"
#include "definitions.h"

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
