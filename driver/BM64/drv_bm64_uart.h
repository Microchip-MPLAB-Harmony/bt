/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64_uart.h

  Summary:
   BM64 Bluetooth Static Driver header file for UART functions.

  Description:
    This file is the header file for the internal functions of the BM64
    driver related to sending and receiving data to/from the BM64 over UART.
 
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef DRV_BM64_UART_H
#define DRV_BM64_UART_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

//#include <stdint.h>
//#include <stdbool.h>
//#include <stddef.h>
//#include <stdlib.h>
#include "configuration.h"
#include "definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
#define TX_BUFFER_SIZE      50
#define RX_BUFFER_SIZE      50
    
#define UPDATE_BUFFER_INDEX(index, limit)  index = ((index+1) >= limit)? 0 : (index+1)
#define UART_TX_STRING      "This is a UART Test String !"
// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/
typedef enum
{
	/* Application's state machine's initial state. */
	DRV_BM64_UART_STATE_INIT=0,
    DRV_BM64_UART_STATE_TRANSMIT,
    DRV_BM64_UART_STATE_RECEIVE,
    DRV_BM64_UART_STATE_ERROR,	

	/* TODO: Define states used by the application state machine. */

} DRV_BM64_UART_STATES;



typedef struct
{
    /* The application's current state */
    DRV_BM64_UART_STATES state;
    
    DRV_HANDLE drvUSARTHandle;
      
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    
    uint8_t rxHead;
    
    uint8_t rxTail;
    
    /* TODO: Define any additional data used by the application. */

} DRV_BM64_UART_DATA;

void DRV_BM64_UART_Initialize ( void );
void DRV_BM64_UART_Tasks( void );
void DRV_BM64_UART_ReceiveEventHandler(uintptr_t context);
void DRV_BM64_UART_TransmitEventHandler(uintptr_t context);
void DRV_BM64_EUSART_Write(uint8_t txData);
uint8_t DRV_BM64_EUSART_Read(void);

extern volatile uint8_t DRV_BM64_eusartRxCount;

#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

