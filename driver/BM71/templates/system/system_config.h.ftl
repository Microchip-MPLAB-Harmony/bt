/*** Bluetooth Driver Configuration ***/

#define DRV_BM71_CLIENTS_NUMBER                 1
   
/* Bluetooth Driver Abstraction definition */
#define DRV_BT_Initialize                       DRV_BM71_Initialize
#define DRV_BT_Status                           DRV_BM71_Status
#define DRV_BT_Tasks                            DRV_BM71_Tasks
#define DRV_BT_Open                             DRV_BM71_Open
#define DRV_BT_Close                            DRV_BM71_Close
#define DRV_BT_EventHandlerSet                  DRV_BM71_EventHandlerSet
#define DRV_BT_GetPowerStatus                   DRV_BM71_GetPowerStatus

#define DRV_BT_EVENT_HANDLER                    DRV_BM71_EVENT_HANDLER
#define DRV_BT_EVENT                            DRV_BM71_EVENT
#define DRV_BT_EVENT_BLESPP_MSG_RECEIVED        DRV_BM71_EVENT_BLESPP_MSG_RECEIVED
#define DRV_BT_EVENT_BLE_STATUS_CHANGED         DRV_BM71_EVENT_BLE_STATUS_CHANGED

#define DRV_BT_PROTOCOL_BLE                     DRV_BM71_PROTOCOL_BLE               
#define DRV_BT_PROTOCOL                         DRV_BM71_PROTOCOL

#define DRV_BT_STATUS_READY                     DRV_BM71_STATUS_READY

#define DRV_BT_BLE_STATUS                       DRV_BM71_BLE_STATUS
#define DRV_BT_BLE_STATUS_STANDBY               DRV_BM71_BLE_STATUS_STANDBY
#define DRV_BT_BLE_STATUS_ADVERTISING           DRV_BM71_BLE_STATUS_ADVERTISING
#define DRV_BT_BLE_STATUS_SCANNING              DRV_BM71_BLE_STATUS_SCANNING
#define DRV_BT_BLE_STATUS_CONNECTED             DRV_BM71_BLE_STATUS_CONNECTED

#define DRV_BT_ClearBLEData                     DRV_BM71_ClearBLEData
#define DRV_BT_ReadDataFromBLE                  DRV_BM71_ReadDataFromBLE
#define DRV_BT_SendDataOverBLE                  DRV_BM71_SendDataOverBLE

#define DRV_BT_BLE_QueryStatus                  DRV_BM71_BLE_QueryStatus
#define DRV_BT_BLE_EnableAdvertising            DRV_BM71_BLE_EnableAdvertising
