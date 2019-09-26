/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "bsp/bsp.h"
#include "configuration.h"
#include "definitions.h"
#include "audio.h"
extern AUDIO_DATA audioData;

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
uint16_t volumeLevels[VOLUME_STEPS] =
{           
    0 /* off */, 33, 66, 100 
};

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/// *****************************************************************************
 /*
  Function:
        static void WM8904_TimerCallback
        (
            uintptr_t context
        )

  Summary:
    Implements the handler for timer callback function.

  Description:
    Called every 1 ms by timer services.  It then decrements WM8904Delay if
    non-zero.

  Parameters:
    context      - Contains index into driver's instance object array

  Remarks:
    None
*/
void updateLEDPlayingStatus();

static void App_TimerCallback( uintptr_t context)
{
    if (appData.buttonDelay)
    {      
        appData.buttonDelay--;
    }
    
    if (appData.volumeDelay)
    {      
        appData.volumeDelay--;
    }
    if (appData.playingStatusDelay)        
    {
        appData.playingStatusDelay--;    
    }
    else
    {
        if (appData.muted)
        {
            updateLEDPlayingStatus(appData.playingStatusToggle);
            appData.playingStatusDelay = 500;
            appData.playingStatusToggle = !appData.playingStatusToggle;            
        }
    }
    //LED2_Toggle();        // uncomment to calibrate timer -- should toggle every 1 ms
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize()

  Remarks:
    See prototype in app.h.
 */
bool firstSync;

void APP_Initialize()
{
    LOW_SR_LED_On();        // all LED's on during initialization    
    LINK_LED_On();
    HI_SR_LED_On();
    STATUS_LED_OFF    
        
    //Place the App state machine in its initial state.
    appData.state = APP_STATE_INIT;
    
    appData.volumeIndex = INIT_VOLUME_IDX;
    appData.volume = volumeLevels[appData.volumeIndex];
    appData.volumeDelay = 0;
    
    appData.buttonDelay = 0;
    appData.muted = false;
    appData.linkStatus = -1;        // force mismatch 
    
    audioInitialize();
    
    LOW_SR_LED_Off();       // all LED's off    
    LINK_LED_Off();
    HI_SR_LED_Off();  

    printf("Starting:\r\n----------------------\r\n");

}  //End APP_Initialize()

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
uint16_t lastappData_state = 0xffff;

void APP_Tasks()
{    
    if (appData.state != lastappData_state)    
    {
        printf("APP_Tasks: appData.state=%d\r\n",appData.state);
        lastappData_state = appData.state;
    }
  
    audioTasks(); 
   
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {            
            /* Open the timer Driver */
            appData.tmrHandle = SYS_TIME_CallbackRegisterMS(App_TimerCallback, 
                    (uintptr_t)0, 1/*ms*/, SYS_TIME_PERIODIC);

            if ( SYS_TIME_HANDLE_INVALID != appData.tmrHandle )
            {
               appData.state = APP_STATE_WAIT_INIT;
            }
        }            
		break;
		
        case APP_STATE_WAIT_INIT:
        {
            if (DRV_BT_STATUS_READY == DRV_BT_GetPowerStatus())
            {           
                appData.state=APP_STATE_IDLE;
                audioStart();
            }
        }
		break;
        
        // Initialized 
        case APP_STATE_IDLE:
        {
            buttonTasks();
            
            break;
        }        
    }
 
    if ((audioData.bt.handle != (DRV_HANDLE)NULL) && 
       (audioData.bt.handle != DRV_HANDLE_INVALID))
    {     
        DRV_BT_LINKSTATUS newLinkStatus = DRV_BT_GetLinkStatus(audioData.bt.handle);          
        // update connection status
        if (newLinkStatus & DRV_BT_A2DP_LINK_STATUS)
        {
            LINK_LED_On();
        }
        else
        {
            LINK_LED_Off();
            STATUS_LED_OFF          
        }
        
        if (newLinkStatus != appData.linkStatus)
        {  
            if ((DRV_BT_GetPowerStatus() == DRV_BT_STATUS_READY))
            {
                if (newLinkStatus == DRV_BT_NO_LINK_STATUS)
                {                                         
                    char buf [DRV_BT_MAXBDNAMESIZE+1];

                    DRV_BT_GetBDName(audioData.bt.handle, buf, DRV_BT_MAXBDNAMESIZE+1);
					printf("BT name: %s\r\n",buf);

                    DRV_BT_GetBDAddress(audioData.bt.handle, buf);
					printf("BT address: %s\r\n",buf);                  
                }

                appData.linkStatus = newLinkStatus;
            }
        }
    }
}

uint16_t lastButton_state = 0xffff;
    
uint8_t DRV_BM64_IsAllowedToSendCommand( void );
void DRV_BM64_InitAckStatus(void);

void _switchSampleFrequency(DRV_BM64_SAMPLE_FREQUENCY sampleFreq);

void buttonTasks( )
{   
    //BUTTON PROCESSING
    /* Check the buttons' current state. */      
    
    if (appData.buttonState != lastButton_state)    
    {
        printf("buttonTasks: buttonState=%d, buttonDelay=%d, playBack=%d\r\n",
                appData.buttonState,appData.buttonDelay,appData.playingStatus);
        lastButton_state = appData.buttonState;
    }
    
    switch ( appData.buttonState )
    {
        case BUTTON_STATE_IDLE:
        {
            if (audioData.bt.handle)
            {
                if ( (appData.buttonDelay==0)&&
                     ((SWITCH_VOL_UP_Get()==SWITCHn_STATE_PRESSED)||
                      (SWITCH_PREV_Get()==SWITCHn_STATE_PRESSED)||
                      (SWITCH_PLAY_Get()==SWITCHn_STATE_PRESSED)||
                      (SWITCH_NEXT_Get()==SWITCHn_STATE_PRESSED)) )
                {
                    appData.buttonDelay=BUTTON_DEBOUNCE;       
                    appData.buttonState=BUTTON_STATE_PRESSED;

                    uint8_t allowed= DRV_BM64_IsAllowedToSendCommand();                
                    if (!allowed)
                    {
                        printf("buttonTasks: forcing allowed=1\r\n");
                        DRV_BM64_InitAckStatus();       // TEMP!!                    
                    }
                }
            }            
            break;
        }
    
        case BUTTON_STATE_PRESSED:
        { 
            if (appData.buttonDelay>0)
            {
                break;      // still debouncing
            }
            
            if(SWITCH_VOL_UP_Get()==SWITCHn_STATE_PRESSED)       // SW1 volume up/pairing
            {
                if (SWITCH_PLAY_Get()==SWITCHn_STATE_PRESSED)
                {
                    // both SW1 and SW3 together
                    appData.buttonState=BUTTON_STATE_FORGET_ALL_LINKS;
                    break;
                }                
                appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                appData.buttonState=BUTTON_STATE_UPPAIRING_PRESSED;                  
            }           
            else if (SWITCH_NEXT_Get()==SWITCHn_STATE_PRESSED)        // SW4 -- next song)
            {
                DRV_BT_PLAYINGSTATUS playingStatus = DRV_BT_GetPlayingStatus(audioData.bt.handle);
                if ((playingStatus==DRV_BT_PLAYING_FF)||(playingStatus==DRV_BT_PLAYING_FR))
                {
                    if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                    {                 
                        DRV_BT_CancelForwardOrRewind(audioData.bt.handle);
                    }
                    appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;
                }
                else
                {
                    appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                    appData.buttonState=BUTTON_STATE_NEXTFF_PRESSED;                
                }
            }           
            else if (SWITCH_PLAY_Get()==SWITCHn_STATE_PRESSED)       // SW3 -- play/pause
            {
                if (SWITCH_VOL_UP_Get()==SWITCHn_STATE_PRESSED)
                {
                    // both SW1 and SW3 together
                    appData.buttonState=BUTTON_STATE_FORGET_ALL_LINKS;
                    break;
                }
                DRV_BT_PLAYINGSTATUS playingStatus = DRV_BT_GetPlayingStatus(audioData.bt.handle);
                if ((playingStatus==DRV_BT_PLAYING_FF)||(playingStatus==DRV_BT_PLAYING_FR))
                {
                    if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                    {                 
                        DRV_BT_CancelForwardOrRewind(audioData.bt.handle);
                    }
                    appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;
                }
                else
                {
                    appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                    appData.buttonState=BUTTON_STATE_PLAYPAUSE_PRESSED;                   
                }
            }            
            else if (SWITCH_PREV_Get()==SWITCHn_STATE_PRESSED)       // SW2 -- previous song
            {
                DRV_BT_PLAYINGSTATUS playingStatus = DRV_BT_GetPlayingStatus(audioData.bt.handle);
                if ((playingStatus==DRV_BT_PLAYING_FF)||(playingStatus==DRV_BT_PLAYING_FR))
                {
                    if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                    {                 
                        DRV_BT_CancelForwardOrRewind(audioData.bt.handle);
                    }
                    appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;
                }
                else
                {               
                    appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                    appData.buttonState=BUTTON_STATE_PREVREWIND_PRESSED;
                }
            }             
            else
            {
                appData.buttonState=BUTTON_STATE_IDLE;                 // button not pressed anymore, assume noise                
            }

            break;
        }
        
        case BUTTON_STATE_UPPAIRING_PRESSED:
        {
            if (SWITCH_PLAY_Get()==SWITCHn_STATE_PRESSED)
            {
                // both SW1 and SW3 together
                appData.buttonState=BUTTON_STATE_FORGET_ALL_LINKS;
                break;
            }            
            if ((appData.buttonDelay>0)&&
                (SWITCH_VOL_UP_Get()!=SWITCHn_STATE_PRESSED))     // SW1 pressed and released < 1 sec -- Next
            {                               
                if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                { 
                    if (0==appData.volumeDelay)
                    {
                        appData.volumeIndex++;
                        if (appData.volumeIndex >= VOLUME_STEPS)
                        {
                            appData.volumeIndex = 0;    
                        }

                        appData.volume = volumeLevels[appData.volumeIndex];                    
                        DRV_BT_volumeSet(audioData.bt.handle,appData.volume);                                                
                    }
                }
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (SWITCH_VOL_UP_Get()==SWITCHn_STATE_PRESSED))       // SW1 still pressed after 1 sec -- start pairing
            {
                DRV_BT_SamplingRateSet(audioData.bt.handle, 8000);
                SetCodecSamplingRate(8000);                
                DRV_BT_EnterBTPairingMode(audioData.bt.handle);               
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
            }           
            break;
        } 
       
        case BUTTON_STATE_NEXTFF_PRESSED:
        {
            if ((appData.buttonDelay>0)&&
                (SWITCH_NEXT_Get()!=SWITCHn_STATE_PRESSED))     // SW4 pressed and released < 1 sec -- Next
            {
                if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                {                 
                    DRV_BT_PlayNextSong(audioData.bt.handle);
                }
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (SWITCH_NEXT_Get()==SWITCHn_STATE_PRESSED))       // SW3 still pressed after 1 sec -- fast forward
            {
                if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)                
                {                 
                    DRV_BT_FastForward(audioData.bt.handle);
                }
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                    
            }
            
            break;
        }                      
        
        case BUTTON_STATE_PLAYPAUSE_PRESSED:
        {
            if (SWITCH_VOL_UP_Get()==SWITCHn_STATE_PRESSED)
            {
                // both SW1 and SW3 together
                appData.buttonState=BUTTON_STATE_FORGET_ALL_LINKS;
                break;
            }            
            if ((appData.buttonDelay>0)&&
                (SWITCH_PLAY_Get()!=SWITCHn_STATE_PRESSED))     // SW3 pressed and released < 1 sec -- play/pause
            {
                DRV_BT_PLAYINGSTATUS playingStatus = DRV_BT_GetPlayingStatus(audioData.bt.handle);
                if ((playingStatus==DRV_BT_PLAYING_FF)||(playingStatus==DRV_BT_PLAYING_FR))
                {
                    if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                    {                 
                        DRV_BT_CancelForwardOrRewind(audioData.bt.handle);
                    }                                       
                }
                else
                {
                    if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                    {                                                                                       
                        DRV_BT_PlayPause(audioData.bt.handle);                            
                    }
                }              
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (SWITCH_PLAY_Get()==SWITCHn_STATE_PRESSED))       // SW4 still pressed after 1 sec -- disconnect
            {
                if (DRV_BT_GetLinkStatus(audioData.bt.handle)!=0)
                {                 
                    DRV_BT_DisconnectAllLinks(audioData.bt.handle);
                }
                else
                {
                    DRV_BT_SamplingRateSet(audioData.bt.handle, 8000);
                    SetCodecSamplingRate(8000);                    
                    DRV_BT_LinkLastDevice(audioData.bt.handle);
                }                                               
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
            }
            
            break;
        }

        case BUTTON_STATE_PREVREWIND_PRESSED:
        {
            if ((appData.buttonDelay>0)&&
                (SWITCH_PREV_Get()!=SWITCHn_STATE_PRESSED))     // SW3 pressed and released < 1 sec -- previous
            {
                if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                {                 
                    DRV_BT_PlayPreviousSong(audioData.bt.handle);
                }              
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (SWITCH_PREV_Get()==SWITCHn_STATE_PRESSED))       // SW3 still pressed after 1 sec -- fast forward
            {
                if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)                
                {                 
                    DRV_BT_Rewind(audioData.bt.handle);
                }                               
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;    
            }
            
            break;
        } 
        
        case BUTTON_STATE_FORGET_ALL_LINKS:
        {
            if (DRV_BT_GetLinkStatus(audioData.bt.handle)!=0)
            {                 
                DRV_BT_DisconnectAllLinks(audioData.bt.handle);
            }            
            if (audioData.bt.handle)
            {                 
                DRV_BT_ForgetAllLinks(audioData.bt.handle);               
            }             
            appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;             
            break;
        }          

        case BUTTON_STATE_WAIT_FOR_RELEASE:
        {
            if ((SWITCH_VOL_UP_Get()!=SWITCHn_STATE_PRESSED)&&
                (SWITCH_PREV_Get()!=SWITCHn_STATE_PRESSED)&&
                (SWITCH_PLAY_Get()!=SWITCHn_STATE_PRESSED)&&
                (SWITCH_NEXT_Get()!=SWITCHn_STATE_PRESSED))
            {
                appData.buttonDelay=BUTTON_DEBOUNCE;
                appData.buttonState=BUTTON_STATE_IDLE;
            }
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
    
}
