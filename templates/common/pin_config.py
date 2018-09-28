def resetPins(pinConfigs):
	for pinConfig in pinConfigs:
		print("Resetting " + pinConfig["name"])
		Database.clearSymbolValue("core", "PIN_" + str(pinConfig["pin"]) + "_FUNCTION_NAME")
		Database.setSymbolValue("core", "PIN_" + str(pinConfig["pin"]) + "_FUNCTION_TYPE", "Available", 1)
		Database.clearSymbolValue("core", "PIN_" + str(pinConfig["pin"]) + "_DIR")
		Database.clearSymbolValue("core", "PIN_" + str(pinConfig["pin"]) + "_LAT")

def configurePins(pinConfigs):
	for pinConfig in pinConfigs:
		print("Configuring " + pinConfig["name"])
		Database.clearSymbolValue("core", "PIN_" + str(pinConfig["pin"]) + "_FUNCTION_NAME")
		Database.clearSymbolValue("core", "PIN_" + str(pinConfig["pin"]) + "_FUNCTION_TYPE")
		Database.clearSymbolValue("core", "PIN_" + str(pinConfig["pin"]) + "_DIR")
		Database.clearSymbolValue("core", "PIN_" + str(pinConfig["pin"]) + "_LAT")
		
		Database.setSymbolValue("core", "PIN_" + str(pinConfig["pin"]) + "_FUNCTION_TYPE", pinConfig["type"], 1)
		if (pinConfig["type"] == "GPIO"):
			Database.setSymbolValue("core", "PIN_" + str(pinConfig["pin"]) + "_FUNCTION_NAME", pinConfig["name"], 1)
		Database.setSymbolValue("core", "PIN_" + str(pinConfig["pin"]) + "_DIR", pinConfig["direction"], 1)
		Database.setSymbolValue("core", "PIN_" + str(pinConfig["pin"]) + "_LAT", pinConfig["latch"], 1)