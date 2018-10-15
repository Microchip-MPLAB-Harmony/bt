<#----------------------------------------------------------------------------
 Copyright (C) 2017-2018 Microchip Technology Inc. and its subsidiaries.

Subject to your compliance with these terms, you may use Microchip software 
and any derivatives exclusively with Microchip products. It is your 
responsibility to comply with third party license terms applicable to your 
use of third party software (including open source software) that may 
accompany Microchip software.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR 
PURPOSE.

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE 
FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN 
ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, 
THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
----------------------------------------------------------------------------->

/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64_uart.c

  Summary:
   BM64 Bluetooth Static Driver source file for UART functions.

  Description:
    This file is the implementation of the internal functions of the BM64
    driver related to sending and receiving data to/from the BM64 over UART.
 
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

#include <xc.h>
#include "bt/driver/bm64/drv_bm64.h"
#include "bt/driver/bm64/drv_bm64_local.h"
#include "bt/driver/bm64/drv_bm64_command_send.h"
#include "bt/driver/bm64/drv_bm64_uart.h"

// all #defines, enums and non-static functions and variables prefixed by
// DRV_BM64 to avoid name conflicts

static DRV_BM64_UART_DATA uartData;
volatile uint8_t DRV_BM64_eusartRxCount;
volatile uint8_t rxData;

// called from DRV_BM64_Initialize, not the directly by the user
void DRV_BM64_UART_Initialize ( void )
{

    /* Place the App state machine in its initial state. */
    uartData.state = DRV_BM64_UART_STATE_INIT;
    
    uartData.rxHead = 0; 
    
    uartData.rxTail = 0;
    
    DRV_BM64_eusartRxCount=0;  
}


/******************************************************************************
  Function:
    void UartWriteBytesData ( uint8_t* pTxBuffer, uint16_t size )

  Return:
    true - if actual number of bytes transmitted is same as the argument size.
    false - in case of an error
 
  Remarks:
    
 */


void DRV_BM64_EUSART_Write(uint8_t txData)
{ 
    ${DRV_BM64_USART}_Write(&txData,1);  
}

uint8_t DRV_BM64_EUSART_Read(void)
{
    uint8_t readByte;
    
    readByte=uartData.rxBuffer[uartData.rxTail++];
    
    if(uartData.rxTail >= RX_BUFFER_SIZE)
    {
        uartData.rxTail = 0;
    }
   
    DRV_BM64_eusartRxCount--;
    
    return readByte;
    
}

void DRV_BM64_UART_TransmitEventHandler(uintptr_t context)
{
     
    DRV_BM64_UART_TransferNextByte();

}

void DRV_BM64_UART_ReceiveEventHandler(uintptr_t context)
{    
    uartData.rxBuffer[uartData.rxHead++] = rxData;
   ${DRV_BM64_USART}_Read((void*)&rxData, 1);    // set up first read

    if(uartData.rxHead >= RX_BUFFER_SIZE)
    {
        uartData.rxHead = 0;
    }
    
    DRV_BM64_eusartRxCount++;
}
/******************************************************************************
  Function:
    void UART_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

 void DRV_BM64_UART_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( uartData.state )
    {
        /* Application's initial state. */
        case DRV_BM64_UART_STATE_INIT:
        
            // Register an event handler with driver. This is done once
           ${DRV_BM64_USART}_WriteCallbackRegister(DRV_BM64_UART_TransmitEventHandler, 0);
           ${DRV_BM64_USART}_ReadCallbackRegister(DRV_BM64_UART_ReceiveEventHandler, 0);

           ${DRV_BM64_USART}_Read((void*)&rxData, 1);    // set up first read
            
            uartData.state = DRV_BM64_UART_STATE_TRANSMIT;           

            break;
            
        case DRV_BM64_UART_STATE_TRANSMIT:
            if(!${DRV_BM64_USART}_WriteIsBusy())
			{
                DRV_BM64_UART_TransferNextByte();
			}

            break;
        

        /* The default state should never be executed. */
        default:        
            /* TODO: Handle error in application's state machine. */
            break;        
    }
}

/*******************************************************************************
 End of File
 */
