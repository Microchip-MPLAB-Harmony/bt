# coding: utf-8
##############################################################################
# Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
#
# Subject to your compliance with these terms, you may use Microchip software
# and any derivatives exclusively with Microchip products. It is your
# responsibility to comply with third party license terms applicable to your
# use of third party software (including open source software) that may
# accompany Microchip software.
#
# THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
# EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
# WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
# PARTICULAR PURPOSE.
#
# IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
# INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
# WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
# BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
# FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
# ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
# THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
##############################################################################

componentsIDTable = ["HarmonyCore", "bluetooth_bm71"]
deactivateIDTable = ["FreeRTOS"]

execfile(Module.getPath() + "../common/pin_config.py")
execfile(Module.getPath() + "../common/bsp_utils.py")

#Add BSP support
execfile(Module.getPath() + "Support_BSP_PIC32MZ_EF_Curiosity_2_0.py")
execfile(Module.getPath() + "Support_BSP_SAM_D21_Xplained_Pro.py")
execfile(Module.getPath() + "Support_BSP_SAM_E54_Curiosity_Ultra.py")
execfile(Module.getPath() + "Support_BSP_SAM_E54_Xplained_Pro.py")

def enableE54_XPLAINED_PROPins(bspID, enable):
    pinConfigs = getBSPSupportNode(bspID, "E54_XPLAINED_PRO").getPinConfig()
    if pinConfigs:
        resetPins(pinConfigs)
        if (enable == True):
            configurePins(pinConfigs)

def enableE54_CURIOSITY_ULTRAPins(bspID, enable):
    pinConfigs = getBSPSupportNode(bspID, "E54_CURIOSITY_ULTRA").getPinConfig()
    if pinConfigs:
        resetPins(pinConfigs)
        if (enable == True):
            configurePins(pinConfigs)

def enableD21_XPLAINED_PROPins(bspID, enable):
    pinConfigs = getBSPSupportNode(bspID, "D21_XPLAINED_PRO").getPinConfig()
    if pinConfigs:
        resetPins(pinConfigs)
        if (enable == True):
            configurePins(pinConfigs)

def enablePIC32MZ_CURIOSITY_20Pins(bspID, enable):
    pinConfigs = getBSPSupportNode(bspID, "PIC32MZ_CURIOSITY_20").getPinConfig()
    if pinConfigs:
        resetPinsPIC32M(pinConfigs)
        if (enable == True):
            configurePinsPIC32M(pinConfigs)

def enableE54_XPLAINED_PROInterface(bspID, enable):
    componentIDTable = getBSPSupportNode(bspID, "E54_XPLAINED_PRO").getComponentActivateList()
    autoConnectTable = getBSPSupportNode(bspID, "E54_XPLAINED_PRO").getComponentAutoConnectList()
    if (enable == True):
        res = Database.activateComponents(componentIDTable)
        res = Database.connectDependencies(autoConnectTable)
    elif (enable == False):
        res = Database.deactivateComponents(componentIDTable)
    enableE54_XPLAINED_PROPins(bspID, enable)

def enableE54_CURIOSITY_ULTRAInterface(bspID, enable):
    componentIDTable = getBSPSupportNode(bspID, "E54_CURIOSITY_ULTRA").getComponentActivateList()
    autoConnectTable = getBSPSupportNode(bspID, "E54_CURIOSITY_ULTRA").getComponentAutoConnectList()
    if (enable == True):
        res = Database.activateComponents(componentIDTable)
        res = Database.connectDependencies(autoConnectTable)
    elif (enable == False):
        res = Database.deactivateComponents(componentIDTable)
    enableE54_CURIOSITY_ULTRAPins(bspID, enable)

def enableD21_XPLAINED_PROInterface(bspID, enable):
    componentIDTable = getBSPSupportNode(bspID, "D21_XPLAINED_PRO").getComponentActivateList()
    autoConnectTable = getBSPSupportNode(bspID, "D21_XPLAINED_PRO").getComponentAutoConnectList()
    if (enable == True):
        res = Database.activateComponents(componentIDTable)
        res = Database.connectDependencies(autoConnectTable)
    elif (enable == False):
        res = Database.deactivateComponents(componentIDTable)
    enableD21_XPLAINED_PROPins(bspID, enable)

def enablePIC32MZ_CURIOSITY_20Interface(bspID, enable):
    componentIDTable = getBSPSupportNode(bspID, "PIC32MZ_CURIOSITY_20").getComponentActivateList()
    autoConnectTable = getBSPSupportNode(bspID, "PIC32MZ_CURIOSITY_20").getComponentAutoConnectList()
    if (enable == True):
        res = Database.activateComponents(componentIDTable)
        res = Database.connectDependencies(autoConnectTable)
    elif (enable == False):
        res = Database.deactivateComponents(componentIDTable)
    enablePIC32MZ_CURIOSITY_20Pins(bspID, enable)

def configureBM71Interface(bspID, interface):
    print("Configuring for " + str(interface) + " Interface.")
    if (bspID == None):
        print("No BSP used, will not configure")
    else:
        if (str(interface) == "E54_XPLAINED_PRO"):
            enableE54_XPLAINED_PROInterface(bspID, True)
        elif (str(interface) == "E54_CURIOSITY_ULTRA"):
            enableE54_CURIOSITY_ULTRAInterface(bspID, True)
        elif (str(interface) == "D21_XPLAINED_PRO"):
            enableD21_XPLAINED_PROInterface(bspID, True)
        elif (str(interface) == "PIC32MZ_CURIOSITY_20"):
            enablePIC32MZ_CURIOSITY_20Interface(bspID, True)

def instantiateComponent(bspComponent):
    global componentsIDTable
    global autoConnectTable

    #Check if a supported BSP is loaded
    bspID = getSupportedBSP()
    if bspID == None:
        print("No compatible BSP selected, will not configure")
        return

    res = Database.activateComponents(componentsIDTable)
    #res = Database.connectDependencies(autoConnectTable)
    res = Database.deactivateComponents(deactivateIDTable);
	
    if getBSPSupportNode(bspID, "E54_XPLAINED_PRO"):
        configureBM71Interface(bspID, "E54_XPLAINED_PRO")	
    elif getBSPSupportNode(bspID, "E54_CURIOSITY_ULTRA"):
        configureBM71Interface(bspID, "E54_CURIOSITY_ULTRA")
    elif getBSPSupportNode(bspID, "D21_XPLAINED_PRO"):
        configureBM71Interface(bspID, "D21_XPLAINED_PRO")
    elif getBSPSupportNode(bspID, "PIC32MZ_CURIOSITY_20"):
        configureBM71Interface(bspID, "PIC32MZ_CURIOSITY_20")


