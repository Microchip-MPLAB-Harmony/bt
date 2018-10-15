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
def instantiateComponent(bm64Component):
    Log.writeInfoMessage("BM64 instantiated")

    bm64Index = bm64Component.createIntegerSymbol("BM64_INDEX", None)
    bm64Index.setVisible(False)
    bm64Index.setDefaultValue(0)

    bm64Use = bm64Component.createBooleanSymbol("USE_DRV_BM64", None)
    bm64Use.setVisible(False)
    bm64Use.setDefaultValue(True)

    bm64Idx0 = bm64Component.createBooleanSymbol("DRV_BM64_INST_IDX0", None)
    bm64Idx0.setVisible(False)
    bm64Idx0.setDefaultValue(True)

    bm64I2SDriver = bm64Component.createStringSymbol("DRV_BM64_I2S", None)
    bm64I2SDriver.setVisible(True)
    bm64I2SDriver.setLabel("I2S Driver used")
    bm64I2SDriver.setReadOnly(True)
    bm64I2SDriver.setDefaultValue("I2S")

    bm64USARTDriver = bm64Component.createStringSymbol("DRV_BM64_USART", None)
    bm64USARTDriver.setVisible(True)
    bm64USARTDriver.setLabel("USART Driver used")
    bm64USARTDriver.setReadOnly(True)
    bm64USARTDriver.setDefaultValue("USART")

    bm64Clients = bm64Component.createIntegerSymbol("DRV_BM64_CLIENTS_NUMBER", None)
    bm64Clients.setVisible(True)
    bm64Clients.setLabel("Number of BM64 Driver Clients")
    bm64Clients.setReadOnly(True)
    bm64Clients.setDefaultValue(1)

    bm64IncludeDeprecated = bm64Component.createBooleanSymbol("INCLUDE_DEPRECATED_MMI_COMMANDS", None)
    bm64IncludeDeprecated.setVisible(True)
    bm64IncludeDeprecated.setLabel("Use Deprecated MMI Commands?")
    bm64IncludeDeprecated.setDefaultValue(True)

    bm64IncludeI2S = bm64Component.createBooleanSymbol("INCLUDE_BM64_I2S", None)
    bm64IncludeI2S.setVisible(True)
    bm64IncludeI2S.setLabel("Use HFP,A2DP,AVRCP Protocols?")
    bm64IncludeI2S.setDefaultValue(False)

    bm64IncludeBLE = bm64Component.createBooleanSymbol("INCLUDE_BM64_BLE", None)
    bm64IncludeBLE.setVisible(True)
    bm64IncludeBLE.setLabel("Include BLE Features?")
    bm64IncludeBLE.setDefaultValue(True)

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
     
    bm64SymHeaderFile = bm64Component.createFileSymbol("DRV_BM64_H", None)
    bm64SymHeaderFile.setMarkup(True) 
    bm64SymHeaderFile.setSourcePath("templates/drv_bm64.h.ftl")
    bm64SymHeaderFile.setOutputName("drv_bm64.h")
    bm64SymHeaderFile.setDestPath("bt/driver/bm64/")
    bm64SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymHeaderFile.setType("HEADER")
    bm64SymHeaderFile.setOverwrite(True)

    bm64SymHeaderFile = bm64Component.createFileSymbol("DRV_BM64_BLE_H", None)
    bm64SymHeaderFile.setSourcePath("drv_bm64_ble.h")
    bm64SymHeaderFile.setOutputName("drv_bm64_ble.h")
    bm64SymHeaderFile.setDestPath("bt/driver/bm64/")
    bm64SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymHeaderFile.setType("HEADER")
    bm64SymHeaderFile.setOverwrite(True)

    bm64SymHeaderFile = bm64Component.createFileSymbol("DRV_BM64_COMMAND_DECODE_H", None)
    bm64SymHeaderFile.setSourcePath("drv_bm64_command_decode.h")
    bm64SymHeaderFile.setOutputName("drv_bm64_command_decode.h")
    bm64SymHeaderFile.setDestPath("bt/driver/bm64/")
    bm64SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymHeaderFile.setType("HEADER")
    bm64SymHeaderFile.setOverwrite(True)

    bm64SymHeaderFile = bm64Component.createFileSymbol("DRV_BM64_COMMAND_SEND_H", None)
    bm64SymHeaderFile.setSourcePath("drv_bm64_command_send.h")
    bm64SymHeaderFile.setOutputName("drv_bm64_command_send.h")
    bm64SymHeaderFile.setDestPath("bt/driver/bm64/")
    bm64SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymHeaderFile.setType("HEADER")
    bm64SymHeaderFile.setOverwrite(True)

    bm64SymHeaderFile = bm64Component.createFileSymbol("DRV_BM64_GPIO_H", None)
    bm64SymHeaderFile.setSourcePath("drv_bm64_gpio.h")
    bm64SymHeaderFile.setOutputName("drv_bm64_gpio.h")
    bm64SymHeaderFile.setDestPath("bt/driver/bm64/")
    bm64SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymHeaderFile.setType("HEADER")
    bm64SymHeaderFile.setOverwrite(True)

    bm64SymHeaderFile = bm64Component.createFileSymbol("DRV_BM64_LINE_IN_H", None)
    bm64SymHeaderFile.setSourcePath("drv_bm64_line_in.h")
    bm64SymHeaderFile.setOutputName("drv_bm64_line_in.h")
    bm64SymHeaderFile.setDestPath("bt/driver/bm64/")
    bm64SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymHeaderFile.setType("HEADER")
    bm64SymHeaderFile.setOverwrite(True)

    bm64SymHeaderFile = bm64Component.createFileSymbol("DRV_BM64_SHA1_H", None)
    bm64SymHeaderFile.setSourcePath("drv_bm64_sha1.h")
    bm64SymHeaderFile.setOutputName("drv_bm64_sha1.h")
    bm64SymHeaderFile.setDestPath("bt/driver/bm64/")
    bm64SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymHeaderFile.setType("HEADER")
    bm64SymHeaderFile.setOverwrite(True)

    bm64SymHeaderFile = bm64Component.createFileSymbol("DRV_BM64_UART_H", None)
    bm64SymHeaderFile.setSourcePath("drv_bm64_uart.h")
    bm64SymHeaderFile.setOutputName("drv_bm64_uart.h")
    bm64SymHeaderFile.setDestPath("bt/driver/bm64/")
    bm64SymHeaderFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymHeaderFile.setType("HEADER")
    bm64SymHeaderFile.setOverwrite(True)

    bm64SymHeaderLocalFile = bm64Component.createFileSymbol("DRV_BM64_HEADER_LOCAL", None)
    bm64SymHeaderLocalFile.setMarkup(True) 
    bm64SymHeaderLocalFile.setSourcePath("templates/drv_bm64_local.h.ftl")
    bm64SymHeaderLocalFile.setOutputName("drv_bm64_local.h")
    bm64SymHeaderLocalFile.setDestPath("bt/driver/bm64/")
    bm64SymHeaderLocalFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymHeaderLocalFile.setType("SOURCE")
    bm64SymHeaderLocalFile.setOverwrite(True)

    bm64SymSourceFile = bm64Component.createFileSymbol("DRV_BM64_SOURCE", None)
    bm64SymSourceFile.setMarkup(True) 
    bm64SymSourceFile.setSourcePath("templates/drv_bm64.c.ftl")
    bm64SymSourceFile.setOutputName("drv_bm64.c")
    bm64SymSourceFile.setDestPath("bt/driver/bm64/")
    bm64SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymSourceFile.setType("SOURCE")
    bm64SymSourceFile.setOverwrite(True)
   
    bm64SymSourceFile = bm64Component.createFileSymbol("DRV_BM64_BLE_C", None)
    bm64SymSourceFile.setSourcePath("src/drv_bm64_ble.c")
    bm64SymSourceFile.setOutputName("drv_bm64_ble.c")
    bm64SymSourceFile.setDestPath("bt/driver/bm64/")
    bm64SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymSourceFile.setType("SOURCE")
    bm64SymSourceFile.setOverwrite(True)

    bm64SymSourceFile = bm64Component.createFileSymbol("DRV_BM64_COMMAND_DECODE_C", None)
    bm64SymSourceFile.setSourcePath("src/drv_bm64_command_decode.c")
    bm64SymSourceFile.setOutputName("drv_bm64_command_decode.c")
    bm64SymSourceFile.setDestPath("bt/driver/bm64/")
    bm64SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymSourceFile.setType("SOURCE")
    bm64SymSourceFile.setOverwrite(True)

    bm64SymSourceFile = bm64Component.createFileSymbol("DRV_BM64_COMMAND_SEND_C", None)
    bm64SymSourceFile.setSourcePath("src/drv_bm64_command_send.c")
    bm64SymSourceFile.setOutputName("drv_bm64_command_send.c")
    bm64SymSourceFile.setDestPath("bt/driver/bm64/")
    bm64SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymSourceFile.setType("SOURCE")
    bm64SymSourceFile.setOverwrite(True)

    bm64SymSourceFile = bm64Component.createFileSymbol("DRV_BM64_GPIO_C", None)
    bm64SymSourceFile.setSourcePath("src/drv_bm64_gpio.c")
    bm64SymSourceFile.setOutputName("drv_bm64_gpio.c")
    bm64SymSourceFile.setDestPath("bt/driver/bm64/")
    bm64SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymSourceFile.setType("SOURCE")
    bm64SymSourceFile.setOverwrite(True)

    bm64SymSourceFile = bm64Component.createFileSymbol("DRV_BM64_LINE_IN_C", None)
    bm64SymSourceFile.setSourcePath("src/drv_bm64_line_in.c")
    bm64SymSourceFile.setOutputName("drv_bm64_line_in.c")
    bm64SymSourceFile.setDestPath("bt/driver/bm64/")
    bm64SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymSourceFile.setType("SOURCE")
    bm64SymSourceFile.setOverwrite(True)

    bm64SymSourceFile = bm64Component.createFileSymbol("DRV_BM64_SHA1_C", None)
    bm64SymSourceFile.setSourcePath("src/drv_bm64_sha1.c")
    bm64SymSourceFile.setOutputName("drv_bm64_sha1.c")
    bm64SymSourceFile.setDestPath("bt/driver/bm64/")
    bm64SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymSourceFile.setType("SOURCE")
    bm64SymSourceFile.setOverwrite(True)

    bm64SymSourceFile = bm64Component.createFileSymbol("DRV_BM64_UART_C", None)
    bm64SymSourceFile.setMarkup(True) 
    bm64SymSourceFile.setSourcePath("templates/drv_bm64_uart.c.ftl")
    bm64SymSourceFile.setOutputName("drv_bm64_uart.c")
    bm64SymSourceFile.setDestPath("bt/driver/bm64/")
    bm64SymSourceFile.setProjectPath("config/" + configName + "/bt/driver/bm64/")
    bm64SymSourceFile.setType("SOURCE")
    bm64SymSourceFile.setOverwrite(True)
   
    bm64SymSystemDefIncFile = bm64Component.createFileSymbol("DRV_BM64_SYSTEM_DEF", None)
    bm64SymSystemDefIncFile.setType("STRING")
    bm64SymSystemDefIncFile.setOutputName("core.LIST_SYSTEM_DEFINITIONS_H_INCLUDES")
    bm64SymSystemDefIncFile.setSourcePath("templates/system/system_definitions.h.ftl")
    bm64SymSystemDefIncFile.setMarkup(True)
    
    #bm64SymSystemDefObjFile = bm64Component.createFileSymbol("DRV_BM64_SYSTEM_DEF_OBJECT", None)
    #bm64SymSystemDefObjFile.setType("STRING")
    #bm64SymSystemDefObjFile.setOutputName("core.LIST_SYSTEM_DEFINITIONS_H_OBJECTS")
    #bm64SymSystemDefObjFile.setSourcePath("templates/system/system_definitions_objects.h.ftl")
    #bm64SymSystemDefObjFile.setMarkup(True)

    bm64SymSystemConfigFile = bm64Component.createFileSymbol("DRV_BM64_SYSTEM_CONFIG", None)
    bm64SymSystemConfigFile.setType("STRING")
    bm64SymSystemConfigFile.setOutputName("core.LIST_SYSTEM_CONFIG_H_DRIVER_CONFIGURATION")
    bm64SymSystemConfigFile.setSourcePath("templates/system/system_config.h.ftl")
    bm64SymSystemConfigFile.setMarkup(True)

    #bm64SymSystemInitDataFile = bm64Component.createFileSymbol("DRV_BM64_INIT_DATA", None)
    #bm64SymSystemInitDataFile.setType("STRING")
    #bm64SymSystemInitDataFile.setOutputName("core.LIST_SYSTEM_INIT_C_DRIVER_INITIALIZATION_DATA")
    #bm64SymSystemInitDataFile.setSourcePath("templates/system/system_initialize_data.c.ftl")
    #bm64SymSystemInitDataFile.setMarkup(True)

    bm64SymSystemInitFile = bm64Component.createFileSymbol("DRV_BM64_SYS_INIT", None)
    bm64SymSystemInitFile.setType("STRING")
    bm64SymSystemInitFile.setOutputName("core.LIST_SYSTEM_INIT_C_SYS_INITIALIZE_DRIVERS")  
    bm64SymSystemInitFile.setSourcePath("templates/system/system_initialize.c.ftl")
    bm64SymSystemInitFile.setMarkup(True)

    bm64SystemTaskFile = bm64Component.createFileSymbol("DRV_BM64_SYSTEM_TASKS_C", None)
    bm64SystemTaskFile.setType("STRING")
    bm64SystemTaskFile.setOutputName("core.LIST_SYSTEM_TASKS_C_CALL_DRIVER_TASKS")
    bm64SystemTaskFile.setSourcePath("templates/system/system_tasks.c.ftl")
    bm64SystemTaskFile.setMarkup(True)

# this callback occurs when user connects I2S or USART driver to BM64 driver block in Project Graph    
def onDependencyConnected(info):
    if info["dependencyID"] == "I2S driver":
        plibUsed = info["localComponent"].getSymbolByID("DRV_BM64_I2S")
    elif info["dependencyID"] == "USART PLIB":
        plibUsed = info["localComponent"].getSymbolByID("DRV_BM64_USART")
    i2sOrUartId = info["remoteComponent"].getID().upper()
    plibUsed.setValue(i2sOrUartId, 1)
