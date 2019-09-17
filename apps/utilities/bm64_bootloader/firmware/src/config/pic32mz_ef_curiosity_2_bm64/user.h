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
  
#define UARTn_SerialSetup   UART1_SerialSetup
#define UARTn_WriteCallbackRegister UART1_WriteCallbackRegister
#define UARTn_ReadCallbackRegister  UART1_ReadCallbackRegister
#define UARTn_Read          UART1_Read
#define UARTn_Write         UART1_Write
    
#ifndef CHIP_FREQ_CPU_MAX
#define CHIP_FREQ_CPU_MAX   (SYS_TIME_CPU_CLOCK_FREQUENCY/2)
#endif    

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // USER_H
/*******************************************************************************
 End of File
*/
