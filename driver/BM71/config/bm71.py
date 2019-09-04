# coding: utf-8
################################################################################
# Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
# 
# Subject to your compliance with these terms, you may use Microchip software 
# and any derivatives exclusively with Microchip products. It is your 
# responsibility to comply with third party license terms applicable to your 
# use of third party software (including open source software) that may 
# accompany Microchip software.
# 
# THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
# EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
# WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR 
# PURPOSE.
# 
# IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
# INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
# WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
# BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE 
# FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN 
# ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, 
# THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
################################################################################
def instantiateComponent(bm71Component):
    Log.writeInfoMessage("BM71 instantiated")

    bm71Index = bm71Component.createIntegerSymbol("BM71_INDEX", None)
    bm71Index.setVisible(False)
    bm71Index.setDefaultValue(0)

    bm71Use = bm71Component.createBooleanSymbol("USE_DRV_BM71", None)
    bm71Use.setVisible(False)
    bm71Use.setDefaultValue(True)

    bm71Idx0 = bm71Component.createBooleanSymbol("DRV_BM71_INST_IDX0", None)
    bm71Idx0.setVisible(False)
    bm71Idx0.setDefaultValue(True)

    bm71USARTDriver = bm71Component.createStringSymbol("DRV_BM71_USART", None)
    bm71USARTDriver.setVisible(True)
    bm71USARTDriver.setLabel("USART Driver used")
    bm71USARTDriver.setReadOnly(True)
    bm71USARTDriver.setDefaultValue("USART")

    bm71Clients = bm71Component.createIntegerSymbol("DRV_BM71_CLIENTS_NUMBER", None)
    bm71Clients.setVisible(True)
    bm71Clients.setLabel("Number of BM71 Driver Clients")
    bm71Clients.setReadOnly(True)
    bm71Clients.setDefaultValue(1)

    #bm71IncludeBLE = bm71Component.createBooleanSymbol("INCLUDE_BM71_BLE", None)
    #bm71IncludeBLE.setVisible(True)
    #bm71IncludeBLE.setLabel("Include BLE Features?")
    #bm71IncludeBLE.setDefaultValue(True)

    # Enable "Generate Harmony Application Files" option in MHC
    Database.setSymbolValue("HarmonyCore", "ENABLE_APP_FILE", True, 1)

    # Enable "Generate Harmony Driver Common Files" option in MHC
    Database.setSymbolValue("HarmonyCore", "ENABLE_DRV_COMMON", True, 1)

    # Enable "Generate Harmony System Service Common Files" option in MHC
    Database.setSymbolValue("HarmonyCore", "ENABLE_SYS_COMMON", True, 1)

    # Enable "Enable System Interrupt" option in MHC
    Database.setSymbolValue("HarmonyCore", "ENABLE_SYS_INT", True, 1)

    # Enable "Enable System Ports" option in MHC
    Database.setSymbolValue("HarmonyCore", "ENABLE_SYS_PORTS", True, 1)

    # Enable "Enable System DMA" option in MHC
    Database.setSymbolValue("HarmonyCore", "ENABLE_SYS_DMA", True, 1)

    # Enable "Enable OSAL" option in MHC
    Database.setSymbolValue("HarmonyCore", "ENABLE_OSAL", True, 1)

    ############################################################################
    #### Code Generation ####
    ############################################################################
    configName = Variables.get("__CONFIGURATION_NAME")  # e.g. "default"
     
    bm71SymHeaderFile = bm71Component.createFileSymbol("DRV_BM71_H", None)
    bm71SymHeaderFile.setSourcePath("drv_bm71.h")
    bm71SymHeaderFile.setOutputName("drv_bm71.h")
    bm71SymHeaderFile.setDestPath("bt/driver/bm71/")
    bm71SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm71/")
    bm71SymHeaderFile.setType("HEADER")
    bm71SymHeaderFile.setOverwrite(True)

    bm71SymHeaderFile = bm71Component.createFileSymbol("DRV_BM71_BLE_H", None)
    bm71SymHeaderFile.setSourcePath("drv_bm71_ble.h")
    bm71SymHeaderFile.setOutputName("drv_bm71_ble.h")
    bm71SymHeaderFile.setDestPath("bt/driver/bm71/")
    bm71SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm71/")
    bm71SymHeaderFile.setType("HEADER")
    bm71SymHeaderFile.setOverwrite(True)

    bm71SymHeaderFile = bm71Component.createFileSymbol("DRV_BM71_COMMAND_DECODE_H", None)
    bm71SymHeaderFile.setSourcePath("drv_bm71_command_decode.h")
    bm71SymHeaderFile.setOutputName("drv_bm71_command_decode.h")
    bm71SymHeaderFile.setDestPath("bt/driver/bm71/")
    bm71SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm71/")
    bm71SymHeaderFile.setType("HEADER")
    bm71SymHeaderFile.setOverwrite(True)

    bm71SymHeaderFile = bm71Component.createFileSymbol("DRV_BM71_COMMAND_SEND_H", None)
    bm71SymHeaderFile.setSourcePath("drv_bm71_command_send.h")
    bm71SymHeaderFile.setOutputName("drv_bm71_command_send.h")
    bm71SymHeaderFile.setDestPath("bt/driver/bm71/")
    bm71SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm71/")
    bm71SymHeaderFile.setType("HEADER")
    bm71SymHeaderFile.setOverwrite(True)

    bm71SymHeaderFile = bm71Component.createFileSymbol("DRV_BM71_GPIO_H", None)
    bm71SymHeaderFile.setSourcePath("drv_bm71_gpio.h")
    bm71SymHeaderFile.setOutputName("drv_bm71_gpio.h")
    bm71SymHeaderFile.setDestPath("bt/driver/bm71/")
    bm71SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm71/")
    bm71SymHeaderFile.setType("HEADER")
    bm71SymHeaderFile.setOverwrite(True)

    bm71SymHeaderFile = bm71Component.createFileSymbol("DRV_BM71_UART_H", None)
    bm71SymHeaderFile.setSourcePath("drv_bm71_uart.h")
    bm71SymHeaderFile.setOutputName("drv_bm71_uart.h")
    bm71SymHeaderFile.setDestPath("bt/driver/bm71/")
    bm71SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm71/")
    bm71SymHeaderFile.setType("HEADER")
    bm71SymHeaderFile.setOverwrite(True)

    bm71SymHeaderLocalFile = bm71Component.createFileSymbol("DRV_BM71_HEADER_LOCAL", None) 
    bm71SymHeaderLocalFile.setSourcePath("src/drv_bm71_local.h")
    bm71SymHeaderLocalFile.setOutputName("drv_bm71_local.h")
    bm71SymHeaderLocalFile.setDestPath("bt/driver/bm71/")
    bm71SymHeaderLocalFile.setProjectPath("config/" + configName + "/bt/driver/bm71/")
    bm71SymHeaderLocalFile.setType("SOURCE")
    bm71SymHeaderLocalFile.setOverwrite(True)

    bm71SymSourceFile = bm71Component.createFileSymbol("DRV_BM71_SOURCE", None)
    bm71SymSourceFile.setSourcePath("src/drv_bm71.c")
    bm71SymSourceFile.setOutputName("drv_bm71.c")
    bm71SymSourceFile.setDestPath("bt/driver/bm71/")
    bm71SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm71/")
    bm71SymSourceFile.setType("SOURCE")
    bm71SymSourceFile.setOverwrite(True)
   
    bm71SymSourceFile = bm71Component.createFileSymbol("DRV_BM71_BLE_C", None)
    bm71SymSourceFile.setSourcePath("src/drv_bm71_ble.c")
    bm71SymSourceFile.setOutputName("drv_bm71_ble.c")
    bm71SymSourceFile.setDestPath("bt/driver/bm71/")
    bm71SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm71/")
    bm71SymSourceFile.setType("SOURCE")
    bm71SymSourceFile.setOverwrite(True)

    bm71SymSourceFile = bm71Component.createFileSymbol("DRV_BM71_COMMAND_DECODE_C", None)
    bm71SymSourceFile.setSourcePath("src/drv_bm71_command_decode.c")
    bm71SymSourceFile.setOutputName("drv_bm71_command_decode.c")
    bm71SymSourceFile.setDestPath("bt/driver/bm71/")
    bm71SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm71/")
    bm71SymSourceFile.setType("SOURCE")
    bm71SymSourceFile.setOverwrite(True)

    bm71SymSourceFile = bm71Component.createFileSymbol("DRV_BM71_COMMAND_SEND_C", None)
    bm71SymSourceFile.setSourcePath("src/drv_bm71_command_send.c")
    bm71SymSourceFile.setOutputName("drv_bm71_command_send.c")
    bm71SymSourceFile.setDestPath("bt/driver/bm71/")
    bm71SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm71/")
    bm71SymSourceFile.setType("SOURCE")
    bm71SymSourceFile.setOverwrite(True)

    bm71SymSourceFile = bm71Component.createFileSymbol("DRV_BM71_GPIO_C", None)
    bm71SymSourceFile.setSourcePath("src/drv_bm71_gpio.c")
    bm71SymSourceFile.setOutputName("drv_bm71_gpio.c")
    bm71SymSourceFile.setDestPath("bt/driver/bm71/")
    bm71SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm71/")
    bm71SymSourceFile.setType("SOURCE")
    bm71SymSourceFile.setOverwrite(True)

    bm71SymSourceFile = bm71Component.createFileSymbol("DRV_BM71_UART_C", None)
    bm71SymSourceFile.setMarkup(True) 
    bm71SymSourceFile.setSourcePath("templates/drv_bm71_uart.c.ftl")
    bm71SymSourceFile.setOutputName("drv_bm71_uart.c")
    bm71SymSourceFile.setDestPath("bt/driver/bm71/")
    bm71SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm71/")
    bm71SymSourceFile.setType("SOURCE")
    bm71SymSourceFile.setOverwrite(True)
   
    bm71SymSystemDefIncFile = bm71Component.createFileSymbol("DRV_BM71_SYSTEM_DEF", None)
    bm71SymSystemDefIncFile.setType("STRING")
    bm71SymSystemDefIncFile.setOutputName("core.LIST_SYSTEM_DEFINITIONS_H_INCLUDES")
    bm71SymSystemDefIncFile.setSourcePath("templates/system/system_definitions.h.ftl")
    bm71SymSystemDefIncFile.setMarkup(True)
    
    #bm71SymSystemDefObjFile = bm71Component.createFileSymbol("DRV_BM71_SYSTEM_DEF_OBJECT", None)
    #bm71SymSystemDefObjFile.setType("STRING")
    #bm71SymSystemDefObjFile.setOutputName("core.LIST_SYSTEM_DEFINITIONS_H_OBJECTS")
    #bm71SymSystemDefObjFile.setSourcePath("templates/system/system_definitions_objects.h.ftl")
    #bm71SymSystemDefObjFile.setMarkup(True)

    bm71SymSystemConfigFile = bm71Component.createFileSymbol("DRV_BM71_SYSTEM_CONFIG", None)
    bm71SymSystemConfigFile.setType("STRING")
    bm71SymSystemConfigFile.setOutputName("core.LIST_SYSTEM_CONFIG_H_DRIVER_CONFIGURATION")
    bm71SymSystemConfigFile.setSourcePath("templates/system/system_config.h.ftl")
    bm71SymSystemConfigFile.setMarkup(True)

    #bm71SymSystemInitDataFile = bm71Component.createFileSymbol("DRV_BM71_INIT_DATA", None)
    #bm71SymSystemInitDataFile.setType("STRING")
    #bm71SymSystemInitDataFile.setOutputName("core.LIST_SYSTEM_INIT_C_DRIVER_INITIALIZATION_DATA")
    #bm71SymSystemInitDataFile.setSourcePath("templates/system/system_initialize_data.c.ftl")
    #bm71SymSystemInitDataFile.setMarkup(True)

    bm71SymSystemInitFile = bm71Component.createFileSymbol("DRV_BM71_SYS_INIT", None)
    bm71SymSystemInitFile.setType("STRING")
    bm71SymSystemInitFile.setOutputName("core.LIST_SYSTEM_INIT_C_SYS_INITIALIZE_DRIVERS")  
    bm71SymSystemInitFile.setSourcePath("templates/system/system_initialize.c.ftl")
    bm71SymSystemInitFile.setMarkup(True)

    bm71SystemTaskFile = bm71Component.createFileSymbol("DRV_BM71_SYSTEM_TASKS_C", None)
    bm71SystemTaskFile.setType("STRING")
    bm71SystemTaskFile.setOutputName("core.LIST_SYSTEM_TASKS_C_CALL_DRIVER_TASKS")
    bm71SystemTaskFile.setSourcePath("templates/system/system_tasks.c.ftl")
    bm71SystemTaskFile.setMarkup(True)

# this callback occurs when user connects I2S or USART driver to BM71 driver block in Project Graph    
def onDependencyConnected(info):

    if info["dependencyID"] == "I2S driver":
        plibUsed = info["localComponent"].getSymbolByID("DRV_BM71_I2S")
    elif info["dependencyID"] == "USART PLIB":
        plibUsed = info["localComponent"].getSymbolByID("DRV_BM71_USART")
    i2sOrUartId = info["remoteComponent"].getID().upper()
    if (info["dependencyID"] == "USART PLIB") and ("SERCOM" in i2sOrUartId):
        plibUsed.setValue(i2sOrUartId+"_USART") # SERCOM PLIB doesn't include USART string
    else:
        plibUsed.setValue(i2sOrUartId)
