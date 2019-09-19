/*** Bluetooth Driver Configuration ***/

#define DRV_BM64_CLIENTS_NUMBER                 1

<#if INCLUDE_BM64_I2S == true>
#define DRV_BM64_DRV_I2S_INDEX                  DRV_I2S_INDEX_0
#define DRV_BM64_READ_QUEUE_SIZE                QUEUE_SIZE_RX_IDX1 
#define DRV_BM64_AUDIO_SAMPLING_RATE            8000      
#define DRV_BM64_MCLK_SAMPLE_FREQ_MULTPLIER     256
#define DRV_BM64_BCLK_BIT_CLK_DIVISOR	        4

#define DRV_BM64_I2S_DRIVER_MODULE_INDEX_IDX0   DRV_I2S_INDEX_0

#define APP_CODEC_DRV_I2S_INDEX                 DRV_I2S_INDEX_1
#define APP_CODEC_WRITE_QUEUE_SIZE              QUEUE_SIZE_TX_IDX0
#define APP_CODEC_INPUT_REFCLOCK    	         6
</#if>
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
