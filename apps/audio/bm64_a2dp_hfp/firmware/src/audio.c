/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    audio.c

  Summary:
    Source code for audio operations for both A2DP and HFP streaming modes.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's audio state machine and calls
    API routines of other MPLAB Harmony modules in the system, including the
    Bluetooth driver, Codec driver, and system services.

 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

#include <stdio.h> 
#include <stdint.h> 
#include "audio.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************


//BT Audio RX Buffer
DRV_BT_DATA32 __attribute__((coherent)) __attribute__((aligned(32)))  
        audioBuffer[AUDIO_QUEUE_SIZE]
                   [AUDIO_BUFFER_SAMPLES];

//Buffer being used for read or write.
static bool     _bufferUsed[AUDIO_QUEUE_SIZE] = {false};
static uint32_t _appAudioContext;
static uint32_t _writeCounter=0;
//static uint32_t _readCounter=0;
//static int32_t  _counterDiff=0;
static uint8_t  _txBufferLevel = 0;
static bool     _initialReadBuffersReady = false;
static uint8_t  _nextTxCompleteIndex = 0;

AUDIO_DATA audioData;
extern APP_DATA appData;

static void _audioEventHandler(DRV_BT_EVENT event, uint32_t param, uintptr_t context);

//#undef SYS_DEBUG
//#define SYS_DEBUG(x,y)  printf(y)

// *****************************************************************************
// *****************************************************************************
// Section: Local Functions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
void audioInitialize()
{
    audioData.state = AUDIO_STATE_OPEN;    

    //????: What kind of context is this???
    audioData.bt.context = (uintptr_t) &_appAudioContext;

    audioData.bt.bufferHandler = 
        (DRV_BT_BUFFER_EVENT_HANDLER) AUDIO_BT_RxBufferEventHandler;

    audioData.bt.readBufHandle = DRV_BT_BUFFER_HANDLE_INVALID;     
    audioData.bt.bufferSize = AUDIO_BUFFER_SAMPLES*sizeof(DRV_BT_DATA32);         

    audioData.codec.context = (uintptr_t) &_appAudioContext;

    audioData.codec.bufferHandler = 
        (DRV_CODEC_BUFFER_EVENT_HANDLER) AUDIO_CODEC_TxBufferEventHandler;    

    audioData.codec.writeBufHandle = DRV_CODEC_BUFFER_HANDLE_INVALID;
    audioData.codec.bufferSize = AUDIO_BUFFER_SAMPLES*sizeof(DRV_BT_DATA32);            
       
    audioData.readIndex = 0;
    audioData.writeIndex = 0;
    audioData.readyIndex = 0;
    audioData.txCompleteIndex =0;
    
    audioData.bt.eventHandler = 
        (DRV_BT_EVENT_HANDLER) _audioEventHandler;    
   
} //End audioInitialize()


//******************************************************************************
// AUDIO_Tasks())
//******************************************************************************

void audioStart()
{
    /* If codec option in MHC "Delay driver initialization (due to shared RESET pin)"
       is checked, then we have to wait until after the BT module has finished initializing
       before we begin initializing the codec.  */
    if (true == DRV_CODEC_IsInitializationDelayed(sysObjdrvCodec0))
    {
        DRV_CODEC_EnableInitialization(sysObjdrvCodec0);    // BT module has finished initializing, okay to start codec now
        audioData.state = AUDIO_STATE_CODEC_OPEN;
    }
    else
    {
		// otherwise we have just been waiting for the BT to initialize before transferring data
        audioData.state = AUDIO_STATE_BT_SUBMIT_INITIAL_READS;
    }
}

void audioTasks()
{ 
    static uint16_t lastaudioData_state = 0xffff;
    
    if (audioData.state != lastaudioData_state)    
    {
        if ((audioData.state != AUDIO_STATE_BT_WAIT_FOR_BUFFER_COMPLETE) && (audioData.state != AUDIO_STATE_BT_BUFFER_COMPLETE))
        {
            printf("AUDIO_Tasks: audioData.state=%d\r\n",audioData.state);
        }
        lastaudioData_state = audioData.state;
    }    
            
    switch(audioData.state)
    {
        //----------------------------------------------------------------------
        // Open BT module 
        //----------------------------------------------------------------------               
        case AUDIO_STATE_OPEN:
        {
            if (SYS_STATUS_READY == DRV_BT_Status())
            {            
                // open BT module, including RX audio stream
                audioData.bt.handle = DRV_BT_Open(DRV_IO_INTENT_READ, DRV_BT_PROTOCOL_ALL);

                if(audioData.bt.handle != DRV_HANDLE_INVALID)
                {
                    audioData.state = AUDIO_STATE_SET_BT_BUFFER_HANDLER;
                }
                else
                {
                    /* Got an Invalid Handle.  Wait for BT module to Initialize */;
                }
            }
        }
        break;
        
        //----------------------------------------------------------------------
        //Set the BT RX Buffer Handler
        //----------------------------------------------------------------------
        case AUDIO_STATE_SET_BT_BUFFER_HANDLER:
        {
            DRV_BT_BufferEventHandlerSet(audioData.bt.handle,
                                          audioData.bt.bufferHandler,
                                          audioData.bt.context); 
            
            DRV_BT_EventHandlerSet(audioData.bt.handle,
                                          audioData.bt.eventHandler,
                                          (uintptr_t)0);
            
            if (true == DRV_CODEC_IsInitializationDelayed(sysObjdrvCodec0))
            {
				// if delayed initialization is enabled, go into a wait state
				// until BT module has finished its initialization
                audioData.state = AUDIO_STATE_WAIT_OPEN;
            }
            else
            {
				// otherwise we can continue
                audioData.state = AUDIO_STATE_CODEC_OPEN;
            }
        }
        break;
        
        case AUDIO_STATE_WAIT_OPEN:
        {
            // waits in this state until BT initialization done and app state machine
            // calls audioStart()
            break;
        }         
        
        //----------------------------------------------------------------------
        // Open CODEC Client
        // audio Tx stream 
        //----------------------------------------------------------------------
        case AUDIO_STATE_CODEC_OPEN:
        {
            audioData.codec.handle = 
                DRV_CODEC_Open(DRV_CODEC_INDEX_0, 
                               DRV_IO_INTENT_WRITE | DRV_IO_INTENT_EXCLUSIVE);

            if(audioData.codec.handle != DRV_HANDLE_INVALID)
            {
                audioData.state = AUDIO_STATE_CODEC_SET_BUFFER_HANDLER;
            }
            else
            {
                /* Got an Invalid Handle.  Wait for AK4384 to Initialize */;
            }
        }
        break;

        //----------------------------------------------------------------------
        // Set a handler for CODEC audio stream TX buffer completion event 
        //----------------------------------------------------------------------
        case AUDIO_STATE_CODEC_SET_BUFFER_HANDLER:
        {            
            //NOTE: CODEC sets Master Clock Rate (MCLK)
            SetCodecSamplingRate(DRV_BT_AUDIO_SAMPLING_RATE);

            DRV_CODEC_BufferEventHandlerSet(audioData.codec.handle,
                                            audioData.codec.bufferHandler,
                                            audioData.codec.context);

            if (true == DRV_CODEC_IsInitializationDelayed(sysObjdrvCodec0))
            {
				// if delayed initialzation is enabled, BT module has already
				// finished if we are here, and we can continue
                audioData.state = AUDIO_STATE_BT_SUBMIT_INITIAL_READS;      
            }
            else
            {
				// otherwise go into a wait state until BT module is ready
                audioData.state = AUDIO_STATE_INIT_DONE;     
            }  
        }
        break;
        
        // Initialized 
        case AUDIO_STATE_INIT_DONE:
        {
            // waits in this state until BT initialization done and app state machine
            // calls audioStart() to set state to AUDIO_STATE_BT_SUBMIT_INITIAL_READS
            break;
        }        
        
        case AUDIO_STATE_BT_SUBMIT_INITIAL_READS:
        {        
            bool isInvalidHandle;
            isInvalidHandle = false;
            
            //Queuing half of TX/RX buffers 
            //for(index=0; index < AUDIO_QUEUE_SIZE; index++)
            //{
                //BT Rx Buffers
                DRV_BT_BufferAddRead(audioData.bt.handle, 
                                      &audioData.bt.readBufHandle,
                                      audioBuffer[audioData.readIndex], 
                                      audioData.bt.bufferSize);                                                                                 
                if(audioData.bt.readBufHandle != DRV_BT_BUFFER_HANDLE_INVALID)
                {
                    _bufferUsed[audioData.readIndex] = true;
                    audioData.bt.readBufHandle = DRV_BT_BUFFER_HANDLE_INVALID;                    

                    audioData.readIndex++;  //HEAD of buffer queue      
                    if(audioData.readIndex >= AUDIO_QUEUE_SIZE)
                    {
                        audioData.readIndex = 0;
                    }                                 
                }
                else
                {
                    isInvalidHandle = true;
                    break;
                }
            //}      
            
            if(false == isInvalidHandle)
            {
                /* Queuing Done. Reception has begun. */
            audioData.state = AUDIO_STATE_BT_WAIT_FOR_BUFFER_COMPLETE;
            }

        }
        break;
        
        //----------------------------------------------------------------------
        // Wait for BT Buffer Complete
        // -- AUDIO_BT_RxBufferEventHandler  handles all operations after
        //    this point.
        //----------------------------------------------------------------------
        case AUDIO_STATE_BT_WAIT_FOR_BUFFER_COMPLETE:
        {
            //Wait here for initial buffer complete
            Nop();
        }
        break;

        case AUDIO_STATE_BT_BUFFER_COMPLETE:
        {
            //BT RX
            if (!_bufferUsed[audioData.readIndex])
            {
                //AUDIO RX
                //Next BT Read Queued
                DRV_BT_BufferAddRead(audioData.bt.handle, 
                                      &audioData.bt.readBufHandle, 
                                      audioBuffer[audioData.readIndex], 
                                      audioData.bt.bufferSize);
                
                if(audioData.bt.readBufHandle != DRV_BT_BUFFER_HANDLE_INVALID)
                {
                    audioData.bt.readBufHandle = DRV_BT_BUFFER_HANDLE_INVALID; 
                    _bufferUsed[audioData.readIndex] = true;

                    //QUEUE HEAD Index (for next BT read)  
                    audioData.readIndex++;      
                    if(audioData.readIndex >= AUDIO_QUEUE_SIZE)
                    {
                        audioData.readIndex = 0;
                    }                                 
                    audioData.state = AUDIO_STATE_BT_WAIT_FOR_BUFFER_COMPLETE;
                }
                else
                {
                    SYS_DEBUG(0, "BT Buffer Read FAILED!!!");
                    //audioData.state = AUDIO_STATE_BT_BUFFER_COMPLETE;
                }
            }
            else
            {
                //Overrun -- Wait for Read buffer to become available.
                SYS_DEBUG(0, "Buffer Overrun\r\n");
                //audioData.state = AUDIO_STATE_BT_BUFFER_COMPLETE;
            }                 
            
            //CODEC TX
	        if (_initialReadBuffersReady)
	        {

                //CODEC TX
                //As BT reads complete the CODEC write is queued.
                if (_bufferUsed[audioData.writeIndex]) //Valid write data
                {                             
                
                    //AUDIO TX 
                    //--New Audio buffer is available to send
                    DRV_CODEC_BufferAddWrite(audioData.codec.handle, 
                                             &audioData.codec.writeBufHandle,
                                             &audioBuffer[audioData.writeIndex], 
                                             audioData.codec.bufferSize);
                      
                    if (audioData.codec.writeBufHandle != DRV_CODEC_BUFFER_HANDLE_INVALID)
                    {
                        audioData.codec.writeBufHandle = DRV_CODEC_BUFFER_HANDLE_INVALID;                    

                        //QUEUE TAIL INDEX is the last item written
                        //--Point to the next item to write if valid.
                        audioData.writeIndex++;  //Next buffer to TX if available    
                        if(audioData.writeIndex >= AUDIO_QUEUE_SIZE)
                        {
                            audioData.writeIndex = 0;                    
                        }                 
                    }
                }
                audioData.state = AUDIO_STATE_BT_WAIT_FOR_BUFFER_COMPLETE;

	        } //End _initialReadBuffersReady
            else 
            {
                //Read Data not Available for Write.
                audioData.state = AUDIO_STATE_BT_WAIT_FOR_BUFFER_COMPLETE;
                //SYS_DEBUG(0, "Codec Buffer Write Underrun!!!");
            }                  

        } //End case AUDIO_STATE_BT_BUFFER_COMPLETE:
        break;

        default:
        {
            SYS_DEBUG(0, "Invalid Audio State\r\n");
        }
        break;

    } //End switch(audioData.state)

} //End Audio_Task()


//******************************************************************************
// AUDIO_BT_RxBufferEventHandler()
// --ISR brings you here.
//******************************************************************************
void AUDIO_BT_RxBufferEventHandler(DRV_BT_BUFFER_EVENT event,
                                   DRV_BT_BUFFER_HANDLE handle,
                                   uintptr_t context )
{           
    switch(event)
    {
        //----------------------------------------------------------------------
        // RX Buffer from BT audio stream
        //----------------------------------------------------------------------
        case DRV_BT_BUFFER_EVENT_COMPLETE:
        {
            _txBufferLevel++;

	    //Buffer Ready for CODEC TX
            if (_txBufferLevel >= AUDIO_QUEUE_SIZE)
            {
                _initialReadBuffersReady = true;
            }

	        if (_initialReadBuffersReady)
            {
                audioData.state = AUDIO_STATE_BT_BUFFER_COMPLETE;
            }
            else
            {
                audioData.state = AUDIO_STATE_BT_SUBMIT_INITIAL_READS;
            }
        }
        break;
        default:
        break;
    } //End switch(event)

} //End AUDIO_BT_RxBufferEventHandler()


//******************************************************************************
// AUDIO_CODEC_TxBufferEventHandler()
//******************************************************************************
void AUDIO_CODEC_TxBufferEventHandler(DRV_CODEC_BUFFER_EVENT event,
                                     DRV_CODEC_BUFFER_HANDLE handle, 
                                     uintptr_t context )
{         
    switch(event)
    {
        default:
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        { 
            //Buffers Written
            _writeCounter++; 

            //BT Read Buffer available after CODEC Write completes
            //_bufferUsed[audioData.writeIndex] = false;
            _bufferUsed[_nextTxCompleteIndex] = false;
   	        _nextTxCompleteIndex++;
	        if (_nextTxCompleteIndex >= AUDIO_QUEUE_SIZE)
            {
	            _nextTxCompleteIndex = 0;
            }                 

            _txBufferLevel--; //Queue Buffer becomes available

            audioData.state = AUDIO_STATE_BT_BUFFER_COMPLETE;
        }
        break;

        case DRV_CODEC_BUFFER_EVENT_ERROR:
        {
            SYS_DEBUG(0, "CODEC Buffer Event Error\r\n");
        } 
        break;

        case DRV_CODEC_BUFFER_EVENT_ABORT:
        {
            SYS_DEBUG(0, "CODEC Buffer Event Abort\r\n");
        } 
        break;
    } //End switch(event)

} //End AUDIO_CODEC_TxBufferEventHandler()

//****************************************************************************** 
void updateLEDPlayingStatus(bool on)
{
    if (on)
    {
        // update GUI based on playing status
        switch(appData.playingStatus)
        {
            case DRV_BT_PLAYING_STOPPED:
            case DRV_BT_PLAYING_PAUSED:
            case DRV_BT_PLAYING_ERROR:
                STATUS_LED_STOP     // red       
                break;
            case DRV_BT_PLAYING_PLAYING:                                      
                STATUS_LED_PLAY     // green
                break;
            case DRV_BT_PLAYING_FF:
                STATUS_LED_FF       // blue
                break;
            case DRV_BT_PLAYING_FR:
                STATUS_LED_REWIND   // yellow                     
                break;
        }
    }
    else
    {
        STATUS_LED_OFF          
    }    
}
static void _audioEventHandler(DRV_BT_EVENT event, uint32_t param, uintptr_t context)
{
    switch(event)
    {
        case DRV_BT_EVENT_VOLUME_CHANGED:
        {
            if ((audioData.codec.handle != (DRV_HANDLE)NULL) && 
                (audioData.codec.handle != DRV_HANDLE_INVALID) && 
                (audioData.bt.handle != (DRV_HANDLE)NULL) && 
                (audioData.bt.handle != DRV_HANDLE_INVALID))
            {
                uint16_t volume7bits = (127*param)/100;     // convert to 7 bits
                volume7bits += 100;     // gives a range of 100 to 227 for AK4954
                if (volume7bits < 105)
                {
                    volume7bits = 0;
                }
                // if volume change made from phone, we need to sync up our incremental value
                
                appData.muted = false;          // default case
                if (param > 95)                 // 100% - 5 steps
                {
                    appData.volumeIndex = 3;    // will jump back to 0 next time    
                }                
                else if (param > 55)            // 2/3 minus 12 steps
                {
                    appData.volumeIndex = 2;    // will jump to 3 next time    
                }
                else if (param > 25)            // 1/3 minus 8 steps
                {
                    appData.volumeIndex = 1;    // will jump to 2 next time     
                }
                else
                {
                    appData.volumeIndex = 0;    // will jump to 1 next time
                }
                if (0==volume7bits)
                {
                    appData.muted = true;
                    appData.playingStatusDelay = LED_TOGGLE_DELAY;  // 1/2 sec
                    appData.playingStatusToggle = false;                    
                }                
                updateLEDPlayingStatus(true);                
                
                printf("phone vol at %d%%, changing codec volume to %d\r\n",param,volume7bits);
                DRV_CODEC_VolumeSet(audioData.codec.handle, 
                                     DRV_CODEC_CHANNEL_LEFT_RIGHT, volume7bits);
                
                appData.volumeDelay = DELAY_AFTER_VOLME_CHG;     // don't allow changes for another second
            }
            break;
        }       
        
        case DRV_BT_EVENT_SAMPLERATE_CHANGED:
        {
            SetCodecSamplingRate(param);
            break;
        }
        
        case DRV_BT_EVENT_PLAYBACK_STATUS_CHANGED:
        {   
            appData.playingStatus = (DRV_BT_PLAYINGSTATUS) param;
            
            updateLEDPlayingStatus(true);     // update GUI based on playing status

            switch(appData.playingStatus)
            {
                case DRV_BT_PLAYING_FR:                                      
                case DRV_BT_PLAYING_FF:                 
                    break;
                default:
                    if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                    {
                        // This is needed in case rewind is underway, and playback goes back to the beginning
                        // of the song.  Otherwise the app is stuck with playback mode on, but not actually playing.
                        DRV_BT_CancelForwardOrRewind(audioData.bt.handle);
                    }                     
                    break;
            }            
            break;
        }
        default:
            break;
    }
}

void SetCodecSamplingRate(uint32_t sampleFreq)
{
    LOW_SR_LED_Off();
    LINK_LED_Off();
    HI_SR_LED_Off();
   
    if ((audioData.codec.handle != (DRV_HANDLE)NULL) && 
        (audioData.codec.handle != DRV_HANDLE_INVALID))
    {                
        switch (sampleFreq)
        {
            case 8000:          // 8 kHz    
            case 16000:         // 16 kHz                
                LOW_SR_LED_On();
                break;

            case 44100:          // 44.1 kHz
            case 48000:          // 48 kHz
                HI_SR_LED_On();
                break;
        }
        DRV_CODEC_SamplingRateSet(audioData.codec.handle,sampleFreq);        
    }        
}
