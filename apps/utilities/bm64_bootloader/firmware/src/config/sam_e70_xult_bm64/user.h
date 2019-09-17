/*******************************************************************************
  User Configuration Header

  File Name:
    user.h

  Summary:
    Build-time configuration header for the user defined by this project.

  Description:
    An MPLAB Project may have multiple configurations.  This file defines the
    build-time options for a single configuration.

  Remarks:
    It only provides macro definitions for build-time configuration options

*******************************************************************************/

#ifndef USER_H
#define USER_H

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: User Configuration macros
// *****************************************************************************
// *****************************************************************************
  
#define UARTn_SerialSetup   USART0_SerialSetup
#define UARTn_WriteCallbackRegister USART0_WriteCallbackRegister
#define UARTn_ReadCallbackRegister  USART0_ReadCallbackRegister
#define UARTn_Read          USART0_Read
#define UARTn_Write         USART0_Write
#define UART_SERIAL_SETUP   USART_SERIAL_SETUP
    
#define SWITCH1_Get             SWITCH_Get
#define SWITCH1_STATE_PRESSED   SWITCH_STATE_PRESSED

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // USER_H
/*******************************************************************************
 End of File
*/
