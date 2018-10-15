# coding: utf-8
##############################################################################
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

componentsIDTable = ["HarmonyCore", "bluetooth_bm64"]
deactivateIDTable = ["FreeRTOS"]

execfile(Module.getPath() + "../common/pin_config.py")
execfile(Module.getPath() + "../common/bsp_utils.py")

#Add BSP support
execfile(Module.getPath() + "Support_BSP_SAM_E70_Xplained_Ultra.py")

def enableI2SCPins(bspID, enable):
    I2SCPinConfigs = getBSPSupportNode(bspID, "I2SC").getPinConfig()
    resetPins(I2SCPinConfigs)
    if (enable == True):
		configurePins(I2SCPinConfigs)

def enableNoaudioPins(bspID, enable):
    noaudioPinConfigs = getBSPSupportNode(bspID, "No Audio").getPinConfig()
    resetPins(noaudioPinConfigs)
    if (enable == True):
        configurePins(noaudioPinConfigs)

def enableI2SCInterface(bspID, enable):
    componentIDTable = getBSPSupportNode(bspID, "I2SC").getComponentActivateList()
    autoConnectTable = getBSPSupportNode(bspID, "I2SC").getComponentAutoConnectList()
    if (enable == True):
        res = Database.activateComponents(componentIDTable)
        res = Database.connectDependencies(autoConnectTable)
    elif (enable == False):
        res = Database.deactivateComponents(componentIDTable)
    enableI2SCPins(bspID, enable)
	
def enableNoaudioInterface(bspID, enable):
    componentIDTable = getBSPSupportNode(bspID, "No Audio").getComponentActivateList()
    autoConnectTable = getBSPSupportNode(bspID, "No Audio").getComponentAutoConnectList()
    if (enable == True):
        res = Database.activateComponents(componentIDTable)
        res = Database.connectDependencies(autoConnectTable)
    elif (enable == False):
        res = Database.deactivateComponents(componentIDTable)
    enableNoaudioPins(bspID, enable)

def configureBM64Interface(bspID, interface):
    print("Configuring for " + str(interface) + " Interface.")
    if (bspID == None):
        print("No BSP used, will not configure")
    else:
        if (str(interface) == "No Audio"):
            enableI2SCInterface(bspID, False)
            enableNoaudioInterface(bspID, True)
        elif (str(interface) == "I2SC"):
            enableNoaudioInterface(bspID, False)
            enableI2SCInterface(bspID, True)

def onBM64InterfaceSelected(interfaceSelected, event):
    bspID = getSupportedBSP()
    newBM64Interface= interfaceSelected.getComponent().getSymbolByID("BM64Interface").getValue()
    currBM64Interface = interfaceSelected.getComponent().getSymbolByID("currBM64Interface").getValue()
    interfaceSelected.getComponent().getSymbolByID("currBM64Interface").setValue(event["value"], 1)
    configureBM64Interface(bspID, str(newBM64Interface))

def instantiateComponent(bspComponent):
    global componentsIDTable
    global autoConnectTable
    global supportedBSPsIDList
	
    #Check if a supported BSP is loaded
    bspID = getSupportedBSP()

    res = Database.activateComponents(componentsIDTable)
    #res = Database.connectDependencies(autoConnectTable)
    res = Database.deactivateComponents(deactivateIDTable);
	
    BM64Interface = bspComponent.createComboSymbol("BM64Interface", None, ["No Audio", "I2SC"])
    BM64Interface.setLabel("BM64 Interface")
    BM64Interface.setDescription("Configures the interface to the BM64 codec.")
    BM64Interface.setDefaultValue("No Audio")
    BM64Interface.setDependencies(onBM64InterfaceSelected, ["BM64Interface"])
    BM64Interface.setVisible(True)
	
	# Shadow display interface symbol
    currBM64Interface = bspComponent.createComboSymbol("currBM64Interface", None, ["No Audio", "I2SC"])
    currBM64Interface.setDefaultValue("No Audio")
    currBM64Interface.setVisible(False)
	
    if (bspID != None):
        configureBM64Interface(bspID, str(currBM64Interface.getValue()))
    else:
        print("No BSP used, only software components are configured. Please add board-specific components.")

