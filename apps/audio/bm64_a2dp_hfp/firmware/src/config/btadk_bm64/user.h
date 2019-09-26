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

#define STATUS_LED_OFF      { LED4_Off(); LED5_Off(); }
#define STATUS_LED_STOP     { LED4_Off(); LED5_Off(); }
#define STATUS_LED_PLAY     { LED4_Off(); LED5_On();  }
#define STATUS_LED_FF       { LED4_On();  LED5_Off(); }
#define STATUS_LED_REWIND   { LED4_On();  LED5_Off(); }
    
#define LOW_SR_LED_On       LED1_On
#define LOW_SR_LED_Off      LED1_Off
    
#define LINK_LED_On         LED2_On
#define LINK_LED_Off        LED2_Off    
    
#define HI_SR_LED_On        LED3_On
#define HI_SR_LED_Off       LED3_Off
   
#define SWITCHn_STATE_PRESSED SWITCH1_STATE_PRESSED   
#define SWITCH_VOL_UP_Get() SWITCH1_Get()
#define SWITCH_PREV_Get()   SWITCH5_Get()
#define SWITCH_PLAY_Get()   SWITCH4_Get()
#define SWITCH_NEXT_Get()   SWITCH3_Get()    
 

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // USER_H
/*******************************************************************************
 End of File
*/
