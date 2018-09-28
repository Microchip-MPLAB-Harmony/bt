bspSupportList = []
bspSupportObjList = None

class bspSupportObj:
	def __init__(self, pinConfig, componentActivateList, componentDeactivateList, componentAutoConnectList, eventCallbackFxn):
		self.pinConfig = pinConfig
		self.componentActivateList = componentActivateList
		self.componentDeactivateList = componentDeactivateList
		self.componentAutoConnectList = componentAutoConnectList
		self.eventCallbackFxn = eventCallbackFxn

	def getPinConfig(self):
		return self.pinConfig

	def getComponentActivateList(self):
		return self.componentActivateList

	def getComponentDeactivateList(self):
		return self.componentActivateList
	
	def getComponentAutoConnectList(self):
		return self.componentAutoConnectList
	
	def getEventCallbackFxn(self):
		return self.eventCallbackFxn

def addBSPSupport(bspID, configID, bspSupportObject):
	global bspSupportList
	global bspSupportObjList
	
	keyID = bspID
	if (configID != None):
		keyID += configID
	
	if (bspSupportObjList == None):
		bspSupportObjList = dict([(keyID, bspSupportObject)])
	else:
		bspSupportObjList[keyID] = bspSupportObject
	
	if bspID not in bspSupportList:
		bspSupportList += [bspID]
	
	print("added support for BSP " + bspID + " keyID " + keyID)

def getBSPSupportNode(bspID, configID):
	global bspSupportObjList
	
	keyID = bspID
	if (configID != None):
		keyID += configID

	return bspSupportObjList[keyID]

# Gets the ID of the first supported BSP component in the project
# If none is supported, returns "None"
def getSupportedBSP():
	global bspSupportList
	
	if (bspSupportList == None):
		return None

	activeComponentsList = Database.getActiveComponentIDs()

	print("bsp list " + str(bspSupportList))
	print("component list " + str(activeComponentsList))
	for keyID in bspSupportList:
		if keyID in activeComponentsList:
			return str(keyID)
	
	return None
