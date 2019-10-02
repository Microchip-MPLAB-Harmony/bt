/*******************************************************************************
  System Configuration Header

  File Name:
    configuration.h

  Summary:
    Build-time configuration header for the system defined by this project.

  Description:
    An MPLAB Project may have multiple configurations.  This file defines the
    build-time options for a single configuration.

  Remarks:
    This configuration header must not define any prototypes or data
    definitions (or include any files that do).  It only provides macro
    definitions for build-time configuration options

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

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section Includes other configuration headers necessary to completely
    define this configuration.
*/

#include "user.h"
#include "toolchain_specifics.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: System Configuration
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: System Service Configuration
// *****************************************************************************
// *****************************************************************************
/* TIME System Service Configuration Options */
#define SYS_TIME_INDEX_0                     0
#define SYS_TIME_MAX_TIMERS                  5
#define SYS_TIME_HW_COUNTER_WIDTH            32
#define SYS_TIME_HW_COUNTER_PERIOD           4294967295U
#define SYS_TIME_HW_COUNTER_HALF_PERIOD		 (SYS_TIME_HW_COUNTER_PERIOD>>1)
#define SYS_TIME_CPU_CLOCK_FREQUENCY         198000000
#define SYS_TIME_COMPARE_UPDATE_EXECUTION_CYCLES      (620)



// *****************************************************************************
// *****************************************************************************
// Section: Driver Configuration
// *****************************************************************************
// *****************************************************************************
/*** Bluetooth Driver Configuration ***/

#define DRV_BM64_CLIENTS_NUMBER                 1

#define DRV_BM64_DRV_I2S_INDEX                  DRV_I2S_INDEX_0
#define DRV_BM64_READ_QUEUE_SIZE                QUEUE_SIZE_RX_IDX1 
#define DRV_BM64_AUDIO_SAMPLING_RATE            8000      
#define DRV_BM64_MCLK_SAMPLE_FREQ_MULTPLIER     256
#define DRV_BM64_BCLK_BIT_CLK_DIVISOR	        4

#define DRV_BM64_I2S_DRIVER_MODULE_INDEX_IDX0   DRV_I2S_INDEX_0

#define APP_CODEC_DRV_I2S_INDEX                 DRV_I2S_INDEX_1
#define APP_CODEC_WRITE_QUEUE_SIZE              QUEUE_SIZE_TX_IDX0
#define APP_CODEC_INPUT_REFCLOCK    	         6
#define DRV_BT_MAXBDNAMESIZE                    DRV_BM64_MAXBDNAMESIZE
#define DRV_BT_AUDIO_SAMPLING_RATE              DRV_BM64_AUDIO_SAMPLING_RATE
    
/* Bluetooth Driver Abstraction definition */
#define DRV_BT_Initialize                       DRV_BM64_Initialize
#define DRV_BT_Status                           DRV_BM64_Status
#define DRV_BT_Tasks                            DRV_BM64_Tasks
#define DRV_BT_Open                             DRV_BM64_Open
#define DRV_BT_Close                            DRV_BM64_Close

#define DRV_BT_DATA32                           DRV_BM64_DATA32
#define DRV_BT_BUFFER_HANDLE                    DRV_BM64_BUFFER_HANDLE
#define DRV_BT_BUFFER_HANDLE_INVALID            DRV_BM64_BUFFER_HANDLE_INVALID
#define DRV_BT_BUFFER_EVENT                     DRV_BM64_BUFFER_EVENT
#define DRV_BT_BUFFER_EVENT_COMPLETE            DRV_BM64_BUFFER_EVENT_COMPLETE
#define DRV_BT_BUFFER_EVENT_HANDLER             DRV_BM64_BUFFER_EVENT_HANDLER
#define DRV_BT_EVENT_HANDLER                    DRV_BM64_EVENT_HANDLER

#define DRV_BT_EVENT                            DRV_BM64_EVENT
#define DRV_BT_EVENT_VOLUME_CHANGED             DRV_BM64_EVENT_VOLUME_CHANGED
#define DRV_BT_EVENT_SAMPLERATE_CHANGED         DRV_BM64_EVENT_SAMPLERATE_CHANGED
#define DRV_BT_EVENT_PLAYBACK_STATUS_CHANGED    DRV_BM64_EVENT_PLAYBACK_STATUS_CHANGED
#define DRV_BT_EVENT_BLESPP_MSG_RECEIVED        DRV_BM64_EVENT_BLESPP_MSG_RECEIVED
#define DRV_BT_EVENT_BLE_STATUS_CHANGED         DRV_BM64_EVENT_BLE_STATUS_CHANGED

#define DRV_BT_BufferEventHandlerSet            DRV_BM64_BufferEventHandlerSet
#define DRV_BT_BufferAddRead                    DRV_BM64_BufferAddRead
#define DRV_BT_EventHandlerSet                  DRV_BM64_EventHandlerSet

#define DRV_BT_GetAudioMode                     DRV_BM64_GetAudioMode

#define DRV_BT_SAMPLE_FREQUENCY                 DRV_BM64_SAMPLE_FREQUENCY
#define DRV_BT_SAMPLEFREQ_8000                  DRV_BM64_SAMPLEFREQ_8000
#define DRV_BT_SAMPLEFREQ_44100                 DRV_BM64_SAMPLEFREQ_44100
#define DRV_BT_SAMPLEFREQ_48000                 DRV_BM64_SAMPLEFREQ_48000

#define DRV_BT_PROTOCOL_A2DP                    DRV_BM64_PROTOCOL_A2DP
#define DRV_BT_PROTOCOL_AVRCP                   DRV_BM64_PROTOCOL_AVRCP
#define DRV_BT_PROTOCOL_HFP_HSP                 DRV_BM64_PROTOCOL_HFP_HSP
#define DRV_BT_PROTOCOL_SPP                     DRV_BM64_PROTOCOL_SPP        
#define DRV_BT_PROTOCOL_BLE                     DRV_BM64_PROTOCOL_BLE        
#define DRV_BT_PROTOCOL_ALL                     DRV_BM64_PROTOCOL_ALL        
#define DRV_BT_PROTOCOL                         DRV_BM64_PROTOCOL

#define DRV_BT_STATUS_READY                     DRV_BM64_STATUS_READY

#define DRV_BT_LINKSTATUS                       DRV_BM64_LINKSTATUS
#define DRV_BT_NO_LINK_STATUS                   DRV_BM64_NO_LINK_STATUS
#define DRV_BT_SCO_LINK_STATUS                  DRV_BM64_SCO_LINK_STATUS
#define DRV_BT_ACL_LINK_STATUS                  DRV_BM64_ACL_LINK_STATUS
#define DRV_BT_HFP_LINK_STATUS                  DRV_BM64_HFP_LINK_STATUS
#define DRV_BT_A2DP_LINK_STATUS                 DRV_BM64_A2DP_LINK_STATUS
#define DRV_BT_AVRCP_LINK_STATUS                DRV_BM64_AVRCP_LINK_STATUS

#define DRV_BT_PLAYINGSTATUS                    DRV_BM64_PLAYINGSTATUS
#define DRV_BT_PLAYING_STOPPED                  DRV_BM64_PLAYING_STOPPED
#define DRV_BT_PLAYING_PLAYING                  DRV_BM64_PLAYING_PLAYING
#define DRV_BT_PLAYING_PAUSED                   DRV_BM64_PLAYING_PAUSED
#define DRV_BT_PLAYING_FF                       DRV_BM64_PLAYING_FF
#define DRV_BT_PLAYING_FR                       DRV_BM64_PLAYING_FR
#define DRV_BT_PLAYING_ERROR                    DRV_BM64_PLAYING_ERROR

#define DRV_BT_BLE_STATUS                       DRV_BM64_BLE_STATUS
#define DRV_BT_BLE_STATUS_STANDBY               DRV_BM64_BLE_STATUS_STANDBY
#define DRV_BT_BLE_STATUS_ADVERTISING           DRV_BM64_BLE_STATUS_ADVERTISING
#define DRV_BT_BLE_STATUS_SCANNING              DRV_BM64_BLE_STATUS_SCANNING
#define DRV_BT_BLE_STATUS_CONNECTED             DRV_BM64_BLE_STATUS_CONNECTED

#define DRV_BT_GetPowerStatus                   DRV_BM64_GetPowerStatus

#define DRV_BT_volumeUp                         DRV_BM64_volumeUp
#define DRV_BT_volumeDown                       DRV_BM64_volumeDown
#define DRV_BT_volumeGet                        DRV_BM64_volumeGet
#define DRV_BT_volumeSet                        DRV_BM64_volumeSet

#define DRV_BT_Play                             DRV_BM64_Play
#define DRV_BT_Pause                            DRV_BM64_Pause
#define DRV_BT_Stop                             DRV_BM64_Stop
#define DRV_BT_PlayNextSong                     DRV_BM64_PlayNextSong
#define DRV_BT_PlayPreviousSong                 DRV_BM64_PlayPreviousSong
#define DRV_BT_PlayPause                        DRV_BM64_PlayPause
#define DRV_BT_Rewind                           DRV_BM64_Rewind
#define DRV_BT_FastForward                      DRV_BM64_FastForward
#define DRV_BT_CancelForwardOrRewind            DRV_BM64_CancelForwardOrRewind
#define DRV_BT_GetPlayingStatus                 DRV_BM64_GetPlayingStatus

#define DRV_BT_DisconnectAllLinks               DRV_BM64_DisconnectAllLinks
#define DRV_BT_LinkLastDevice                   DRV_BM64_LinkLastDevice
#define DRV_BT_EnterBTPairingMode               DRV_BM64_EnterBTPairingMode
#define DRV_BT_ForgetAllLinks                   DRV_BM64_ForgetAllLinks
#define DRV_BT_GetLinkStatus                    DRV_BM64_GetLinkStatus
#define DRV_BT_SamplingRateSet                  DRV_BM64_SamplingRateSet

#define DRV_BT_GetBDAddress                     DRV_BM64_GetBDAddress
#define DRV_BT_GetBDName                        DRV_BM64_GetBDName
#define DRV_BT_SetBDName                        DRV_BM64_SetBDName

#define DRV_BT_ClearBLEData                     DRV_BM64_ClearBLEData
#define DRV_BT_ReadDataFromBLE                  DRV_BM64_ReadDataFromBLE
#define DRV_BT_SendDataOverBLE                  DRV_BM64_SendDataOverBLE

#define DRV_BT_BLE_QueryStatus                  DRV_BM64_BLE_QueryStatus
#define DRV_BT_BLE_EnableAdvertising            DRV_BM64_BLE_EnableAdvertising

/* I2C Driver Instance 0 Configuration Options */
#define DRV_I2C_INDEX_0                       0
#define DRV_I2C_CLIENTS_NUMBER_IDX0           1
#define DRV_I2C_QUEUE_SIZE_IDX0               8
#define DRV_I2C_CLOCK_SPEED_IDX0              50000

/* I2S Driver Instance 0 Configuration Options */
#define DRV_I2S_INDEX_0                       0
#define DRV_I2S_CLIENTS_NUMBER_IDX0           1
#define DRV_I2S_QUEUE_DEPTH_COMBINED          8
#define DRV_I2S_QUEUE_SIZE_IDX0               8
#define DRV_I2S_DATA_LENGTH_IDX0              32
#define DRV_I2S_INT_SRC_IDX0                  _DMA0_VECTOR
#define DRV_I2S_XMIT_DMA_CH_IDX0              SYS_DMA_CHANNEL_0
#define DRV_I2S_RCV_DMA_CH_IDX0               SYS_DMA_CHANNEL_1


/* I2C Driver Common Configuration Options */
#define DRV_I2C_INSTANCES_NUMBER              1


/* I2S Driver Instance 1 Configuration Options */
#define DRV_I2S_INDEX_1                       1
#define DRV_I2S_CLIENTS_NUMBER_IDX1           1
#define DRV_I2S_QUEUE_DEPTH_COMBINED          8
#define DRV_I2S_QUEUE_SIZE_IDX1               8
#define DRV_I2S_DATA_LENGTH_IDX1              32
#define DRV_I2S_INT_SRC_IDX1                  _DMA0_VECTOR
#define DRV_I2S_XMIT_DMA_CH_IDX1              SYS_DMA_CHANNEL_2
#define DRV_I2S_RCV_DMA_CH_IDX1               SYS_DMA_CHANNEL_3


/*** Codec Driver Configuration ***/

#define DRV_AK4954_CLIENTS_NUMBER                           1
#define DRV_AK4954_INSTANCES_NUMBER                         1

#define DRV_AK4954_MASTER_MODE                              false
#define DRV_AK4954_AUDIO_SAMPLING_RATE                      44100
#define DRV_AK4954_VOLUME	                      	        200
#define DRV_AK4954_AUDIO_DATA_FORMAT_MACRO             	    DRV_AK4954_AUDIO_DATA_FORMAT_I2S_32BIT
#define DRV_AK4954_WHICH_MIC_INPUT                          MIC1
#define DRV_AK4954_ENABLE_MIC_BIAS                          true
#define DRV_AK4954_MIC_GAIN	                      	        20
// KEEP NEXT 2 LINES UNTIL JIRA MH3-22284 IS RESOLVED
#define DRV_AK4954_MCLK_SAMPLE_FREQ_MULTPLIER               256
#define DRV_AK4954_BCLK_BIT_CLK_DIVISOR                     4
// KEEP NEXT LINE UNTIL JIRA MH3-22285 IS RESOLVED
#define DRV_AK4954_DELAY_INITIALIZATION                     true

// KEEP NEXT LINE AS IS UNTIL JIRA MH3-22284 IS RESOLVED
#define DRV_AK4954_I2S_DRIVER_MODULE_INDEX_IDX0             DRV_I2S_INDEX_1
#define DRV_AK4954_I2C_DRIVER_MODULE_INDEX_IDX0             DRV_I2C_INDEX_0
/* CODEC Driver Abstraction definition */

#define DRV_CODEC_INDEX_0                                   DRV_AK4954_INDEX_0
#define sysObjdrvCodec0                                     sysObj.drvak4954Codec0
#define DRV_CODEC_BUFFER_HANDLE                             DRV_AK4954_BUFFER_HANDLE
#define DRV_CODEC_BUFFER_HANDLE_INVALID                     DRV_AK4954_BUFFER_HANDLE_INVALID
#define DRV_CODEC_BUFFER_EVENT_HANDLER                      DRV_AK4954_BUFFER_EVENT_HANDLER
#define DRV_CODEC_BUFFER_EVENT                              DRV_AK4954_BUFFER_EVENT
#define DRV_CODEC_BUFFER_EVENT_COMPLETE                     DRV_AK4954_BUFFER_EVENT_COMPLETE
#define DRV_CODEC_BUFFER_EVENT_ERROR                        DRV_AK4954_BUFFER_EVENT_ERROR
#define DRV_CODEC_BUFFER_EVENT_ABORT                        DRV_AK4954_BUFFER_EVENT_ABORT
#define DRV_CODEC_COMMAND_EVENT_HANDLER                     DRV_AK4954_COMMAND_EVENT_HANDLER

#define DRV_CODEC_CHANNEL_LEFT                              DRV_AK4954_CHANNEL_LEFT
#define DRV_CODEC_CHANNEL_RIGHT                             DRV_AK4954_CHANNEL_RIGHT
#define DRV_CODEC_CHANNEL_LEFT_RIGHT                        DRV_AK4954_CHANNEL_LEFT_RIGHT

#define DRV_CODEC_Initialize                                DRV_AK4954_Initialize
#define DRV_CODEC_Deinitialize                              DRV_AK4954_Deinitialize
#define DRV_CODEC_Status                                    DRV_AK4954_Status
#define DRV_CODEC_Tasks                                     DRV_AK4954_Tasks
#define DRV_CODEC_Open                                      DRV_AK4954_Open
#define DRV_CODEC_Close                                     DRV_AK4954_Close
#define DRV_CODEC_BufferEventHandlerSet                     DRV_AK4954_BufferEventHandlerSet
#define DRV_CODEC_CommandEventHandlerSet                    DRV_AK4954_CommandEventHandlerSet
#define DRV_CODEC_BufferAddWrite                            DRV_AK4954_BufferAddWrite
#define DRV_CODEC_BufferAddRead                             DRV_AK4954_BufferAddRead
#define DRV_CODEC_BufferAddWriteRead                        DRV_AK4954_BufferAddWriteRead
#define DRV_CODEC_WriteQueuePurge                           DRV_AK4954_WriteQueuePurge
#define DRV_CODEC_ReadQueuePurge                            DRV_AK4954_ReadQueuePurge
#define DRV_CODEC_SamplingRateSet                           DRV_AK4954_SamplingRateSet
#define DRV_CODEC_SamplingRateGet                           DRV_AK4954_SamplingRateGet
#define DRV_CODEC_VolumeSet                                 DRV_AK4954_VolumeSet
#define DRV_CODEC_VolumeGet                                 DRV_AK4954_VolumeGet
#define DRV_CODEC_MuteOn                                    DRV_AK4954_MuteOn
#define DRV_CODEC_MuteOff                                   DRV_AK4954_MuteOff
#define DRV_CODEC_MicGainSet                                DRV_AK4954_MicGainSet
#define DRV_CODEC_MicGainGet                                DRV_AK4954_MicGainGet
#define DRV_CODEC_MicMuteOn                                 DRV_AK4954_MicMuteOn
#define DRV_CODEC_MicMuteOff                                DRV_AK4954_MicMuteOff
#define DRV_CODEC_GetI2SDriver                              DRV_AK4954_GetI2SDriver
#define DRV_CODEC_LRCLK_Sync                                DRV_AK4954_LRCLK_Sync
// KEEP NEXT 2 LINES UNTIL JIRA MH3-22285 IS RESOLVED
#define DRV_CODEC_EnableInitialization                      DRV_AK4954_EnableInitialization    
#define DRV_CODEC_IsInitializationDelayed                   DRV_AK4954_IsInitializationDelayed         

/* I2S Driver Common Configuration Options */
#define DRV_I2S_INSTANCES_NUMBER              2



// *****************************************************************************
// *****************************************************************************
// Section: Middleware & Other Library Configuration
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // CONFIGURATION_H
/*******************************************************************************
 End of File
*/
